#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake_jaco_driver/lcmt_jaco_extended_status.hpp"

#include "kinova_driver_common.h"

namespace {
// Kinova says 100Hz is the proper frequency for joint velocity
// updates.  See
// https://github.com/Kinovarobotics/kinova-ros#velocity-control-joint-space-and-cartesian-space
const int kKinovaUpdateIntervalUs = 10000;
const char* kLcmCommandChannel = "KINOVA_JACO_COMMAND";
const char* kLcmStatusChannel = "KINOVA_JACO_STATUS";
const char* kLcmExtendedStatusChannel = "KINOVA_JACO_EXTENDED_STATUS";
}  // namespace

DEFINE_string(lcm_command_channel, kLcmCommandChannel,
              "Channel to receive LCM command messages on");
DEFINE_string(lcm_status_channel, kLcmStatusChannel,
              "Channel to send LCM status messages on");
DEFINE_string(lcm_extended_status_channel, kLcmExtendedStatusChannel,
              "Channel to send LCM extended status messages on");
DEFINE_string(lcm_url, "", "LCM URL for Jaco driver");
DEFINE_string(optimal_z, "",
              "A file containing the optimal z parameters for this robot.");
DEFINE_double(joint_command_factor, 1.,
              "A multiplier to apply to received joint velocity commands.");
DEFINE_double(joint_status_factor, 1.,
              "A multiplier to apply to all reported joint velocities.");

namespace {

void HandleWatchdogSignal(int signal) noexcept {
  std::cerr << "Lost communication with robot.  Exiting.";
  abort();
}

timer_t CreateWatchdog() {
  char err_buf[1024];

  // TODO(sam.creasey) If we ever want to create more than one of these, we
  // may need a mechanism to either have them choose different signals or be
  // smart about sharing one and properly registering/unregistering the signal
  // handler.
  const int signum = SIGRTMIN;

  struct sigaction sa{};
  memset(&sa, 0, sizeof(sa));

  // Check to make sure something else hasn't grabbed our signal.
  if (sigaction(signum, nullptr, &sa) == -1) {
    char* err = strerror_r(errno, err_buf, sizeof(err_buf));
    throw std::runtime_error(
        std::string("Unable to check signal for watchdog: ") + err);
  }

  if (sa.sa_handler != SIG_DFL) {
    throw std::runtime_error(
        "Not creating watchdog: signal handler already registered.");
  }

  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = HandleWatchdogSignal;
  sigemptyset(&sa.sa_mask);
  if (sigaction(signum, &sa, nullptr) == -1) {
    char* err = strerror_r(errno, err_buf, sizeof(err_buf));
    throw std::runtime_error(
        std::string("Unable to register watchdog handler: ") + err);
  }

  struct sigevent sev{};
  memset(&sev, 0, sizeof(sev));
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = signum;

  timer_t timer_id{};
  if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id) == -1) {
    char* err = strerror_r(errno, err_buf, sizeof(err_buf));
    throw std::runtime_error(
        std::string("Unable to create watchdog timer: ") + err);
  }

  return timer_id;
}

int64_t GetTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000000L + tv.tv_usec;
}

// The Kinova SDK communicates joint positions in degrees, which would
// be unusual in drake.  Convert to/from radians appropriately.
double to_degrees(double radians) { return radians * (180.0 / M_PI); }
double to_radians(double degrees) { return degrees * (M_PI / 180.0); }
int glbl_deviceid = 0;
class KinovaDriver {
 public:

  int devicetype;

  KinovaDriver(int devicetype_in)
      : lcm_(FLAGS_lcm_url) {

    // TODO(sam.creasey) figure out how to detect the correct number
    glbl_deviceid=devicetype = devicetype_in;

    if(KinovaDriver::devicetype ==eRobotType_Spherical_7DOF_Service )
    {
     std::cout<<"number of joints on robot are 7\n";
     lcm_status_.num_joints = 7;
    }
    else
    {
     std::cout<<"number of joints on robot are 6\n";
     lcm_status_.num_joints = 6;
    }
    std::cout<<"setting joints to zero\n";
    lcm_status_.joint_position.resize(lcm_status_.num_joints, 0);
    lcm_status_.joint_velocity.resize(lcm_status_.num_joints, 0);
    lcm_status_.joint_torque.resize(lcm_status_.num_joints, 0);
    lcm_status_.joint_torque_external.resize(lcm_status_.num_joints, 0);
    lcm_status_.joint_current.resize(lcm_status_.num_joints, 0);

    // TODO(sam-creasey) Find a way to determine if this should be set to zero
    // when the hand is detached.  GetGripperStatus doesn't seem to return
    // meaningfully different output (e.g. the gripper model may be reported
    // as "No init" if a gripper is attached or not.)
    lcm_status_.num_fingers = 3;
    if (lcm_status_.num_fingers) {
      std::cout<<"setting fingertips to zero\n";
      lcm_status_.finger_position.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_velocity.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_torque.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_torque_external.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_current.resize(lcm_status_.num_fingers, 0);
    }
    else
    {
      std::cout<<"no finger tips\n";
    }


    lcm_extended_status_.num_joints = lcm_status_.num_joints;
    lcm_extended_status_.actuator_temperature.resize(
        lcm_extended_status_.num_joints, 0);
    lcm_extended_status_.num_fingers = lcm_status_.num_fingers;
    if (lcm_extended_status_.num_fingers) {
      lcm_extended_status_.finger_temperature.resize(
        lcm_extended_status_.num_fingers, 0);
    }

    commanded_velocity_.InitStruct();
    commanded_velocity_.LimitationsActive = 0;
    commanded_velocity_.SynchroType = 1;
    commanded_velocity_.Position.Type = ANGULAR_VELOCITY;
    commanded_velocity_.Position.HandMode = VELOCITY_MODE;

    std::cout<<"subscribe to incomming messages\n";
    lcm::Subscription* sub =
        lcm_.subscribe(FLAGS_lcm_command_channel,
                       &KinovaDriver::HandleCommandMessage,
                       this);
    sub->setQueueCapacity(2);

    // We're trying to determine that the robot went away entirely, not
    // monitor realtime performance.  Set the watchdog to something large.
    const int timeout_usec = 1000000;
    memset(&timer_value_, 0, sizeof(timer_value_));
    timer_value_.it_interval.tv_sec = timeout_usec / 1000000;
    timer_value_.it_interval.tv_nsec = (timeout_usec % 1000000) * 1000;
    timer_value_.it_value.tv_sec = timer_value_.it_interval.tv_sec;
    timer_value_.it_value.tv_nsec = timer_value_.it_interval.tv_nsec;

    timer_id_ = CreateWatchdog();
  }

  void Run() {
    // Reading the USB values actually takes a while, so keep track of
    // how long until we want to publish again.
    int64_t time_for_next_step = GetTime();
    while (true) {
      Pet();

      time_for_next_step += kKinovaUpdateIntervalUs;
      
      PublishStatus();

      int64_t remaining_time = time_for_next_step - GetTime();
      int inside_loop = 0;
      do {
        if (remaining_time < 0) {
          remaining_time = 0;
        }
        ++inside_loop;
        lcm_.handleTimeout(remaining_time / 1000);
        remaining_time = time_for_next_step - GetTime();
      } while (remaining_time > 0);
    }
  }

 private:
  void Pet() {
    char err_buf[1024];

    if (timer_settime(timer_id_, 0, &timer_value_, NULL) == -1) {
      char* err = strerror_r(errno, err_buf, sizeof(err_buf));
      throw std::runtime_error(
          std::string("Unable to start watchdog timer: ") + err);
    }
  }

  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const drake::lcmt_jaco_command* command) {
    // TODO(sam.creasey) figure out how to detect the correct number
    // of joints/fingers.
    std::cout<<"enter HandleCommandMessage "<<command->joint_velocity[0] * FLAGS_joint_command_factor << "\n";
    if (command->num_joints > 0) {
      if(glbl_deviceid == eRobotType_Spherical_7DOF_Service)
      {
        assert(command->num_joints == 7);
      }
      if(glbl_deviceid == eRobotType_Spherical_6DOF_Service)
      {
        assert(command->num_joints == 6);
      }
      AngularInfo* actuators = &commanded_velocity_.Position.Actuators;
      actuators->Actuator1 = to_degrees(
          command->joint_velocity[0] * FLAGS_joint_command_factor);
      actuators->Actuator2 = to_degrees(
          command->joint_velocity[1] * FLAGS_joint_command_factor);
      actuators->Actuator3 = to_degrees(
          command->joint_velocity[2] * FLAGS_joint_command_factor);
      actuators->Actuator4 = to_degrees(
          command->joint_velocity[3] * FLAGS_joint_command_factor);
      actuators->Actuator5 = to_degrees(
          command->joint_velocity[4] * FLAGS_joint_command_factor);
      actuators->Actuator6 = to_degrees(
          command->joint_velocity[5] * FLAGS_joint_command_factor);
      actuators->Actuator7 = to_degrees(
          command->joint_velocity[6] * FLAGS_joint_command_factor);

      if(glbl_deviceid == eRobotType_Spherical_6DOF_Service)
      {
        actuators->Actuator7 = 0;
      }
    }

    if (command->num_fingers > 0) {
      assert(command->num_fingers == 3);
      FingersPosition* fingers = &commanded_velocity_.Position.Fingers;
      fingers->Finger1 = to_degrees(command->finger_velocity[0]);
      fingers->Finger2 = to_degrees(command->finger_velocity[1]);
      fingers->Finger3 = to_degrees(command->finger_velocity[2]);
    }
    std::cout<<"act1:"<<command->joint_velocity[0]<<"\n";
    std::cout<<"act2:"<<command->joint_velocity[1]<<"\n";
    std::cout<<"act3:"<<command->joint_velocity[2]<<"\n";
    std::cout<<"act4:"<<command->joint_velocity[3]<<"\n";
    std::cout<<"act5:"<<command->joint_velocity[4]<<"\n";
    std::cout<<"act6:"<<command->joint_velocity[5]<<"\n";
    std::cout<<"act7:"<<command->joint_velocity[6]<<"\n";
    SdkSendBasicTrajectory(commanded_velocity_);
  }

  void PublishStatus() {

    AngularPosition current_position;
    SdkGetAngularPosition(current_position);
    lcm_status_.joint_position[0] =
        to_radians(current_position.Actuators.Actuator1);
    lcm_status_.joint_position[1] =
        to_radians(current_position.Actuators.Actuator2);
    lcm_status_.joint_position[2] =
        to_radians(current_position.Actuators.Actuator3);
    lcm_status_.joint_position[3] =
        to_radians(current_position.Actuators.Actuator4);
    lcm_status_.joint_position[4] =
        to_radians(current_position.Actuators.Actuator5);
    lcm_status_.joint_position[5] =
        to_radians(current_position.Actuators.Actuator6);

    lcm_status_.joint_position[6] =
        to_radians(current_position.Actuators.Actuator7);
    if(KinovaDriver::devicetype == eRobotType_Spherical_6DOF_Service)
    {
    lcm_status_.joint_position[6] = 0;
    }

    if (lcm_status_.num_fingers) {
      lcm_status_.finger_position[0] =
          to_radians(current_position.Fingers.Finger1);
      lcm_status_.finger_position[1] =
          to_radians(current_position.Fingers.Finger2);
      lcm_status_.finger_position[2] =
          to_radians(current_position.Fingers.Finger3);
    }

    AngularPosition current_velocity;
    SdkGetAngularVelocity(current_velocity);
    lcm_status_.joint_velocity[0] = to_radians(
        current_velocity.Actuators.Actuator1 * FLAGS_joint_status_factor);
    lcm_status_.joint_velocity[1] = to_radians(
        current_velocity.Actuators.Actuator2 * FLAGS_joint_status_factor);
    lcm_status_.joint_velocity[2] = to_radians(
        current_velocity.Actuators.Actuator3 * FLAGS_joint_status_factor);
    lcm_status_.joint_velocity[3] = to_radians(
        current_velocity.Actuators.Actuator4 * FLAGS_joint_status_factor);
    lcm_status_.joint_velocity[4] = to_radians(
        current_velocity.Actuators.Actuator5 * FLAGS_joint_status_factor);
    lcm_status_.joint_velocity[5] = to_radians(
        current_velocity.Actuators.Actuator6 * FLAGS_joint_status_factor);
    lcm_status_.joint_velocity[6] = to_radians(
        current_velocity.Actuators.Actuator7 * FLAGS_joint_status_factor);
    if(glbl_deviceid == eRobotType_Spherical_6DOF_Service)
    {
    lcm_status_.joint_velocity[6] = 0;
    }

    if (lcm_status_.num_fingers) {
      lcm_status_.finger_velocity[0] =
          to_radians(current_velocity.Fingers.Finger1);
      lcm_status_.finger_velocity[1] =
          to_radians(current_velocity.Fingers.Finger2);
      lcm_status_.finger_velocity[2] =
          to_radians(current_velocity.Fingers.Finger3);
    }

    AngularPosition current_torque;
    SdkGetAngularForceGravityFree(current_torque);
    lcm_status_.joint_torque_external[0] = current_torque.Actuators.Actuator1;
    lcm_status_.joint_torque_external[1] = current_torque.Actuators.Actuator2;
    lcm_status_.joint_torque_external[2] = current_torque.Actuators.Actuator3;
    lcm_status_.joint_torque_external[3] = current_torque.Actuators.Actuator4;
    lcm_status_.joint_torque_external[4] = current_torque.Actuators.Actuator5;
    lcm_status_.joint_torque_external[5] = current_torque.Actuators.Actuator6;
    lcm_status_.joint_torque_external[6] = current_torque.Actuators.Actuator7;
    if(glbl_deviceid == eRobotType_Spherical_6DOF_Service)
    {
     lcm_status_.joint_torque_external[6] = 0;
    }

    if (lcm_status_.num_fingers) {
      lcm_status_.finger_torque_external[0] = current_torque.Fingers.Finger1;
      lcm_status_.finger_torque_external[1] = current_torque.Fingers.Finger2;
      lcm_status_.finger_torque_external[2] = current_torque.Fingers.Finger3;
    }

    // Send some fields at a lower rate to account for the delay imposed by
    // reading additional fields from the robot.
    if (msgs_sent_ % 2 == 0) {
      SdkGetAngularForce(current_torque);
      lcm_status_.joint_torque[0] = current_torque.Actuators.Actuator1;
      lcm_status_.joint_torque[1] = current_torque.Actuators.Actuator2;
      lcm_status_.joint_torque[2] = current_torque.Actuators.Actuator3;
      lcm_status_.joint_torque[3] = current_torque.Actuators.Actuator4;
      lcm_status_.joint_torque[4] = current_torque.Actuators.Actuator5;
      lcm_status_.joint_torque[5] = current_torque.Actuators.Actuator6;
      lcm_status_.joint_torque[6] = current_torque.Actuators.Actuator7;
      if(glbl_deviceid == eRobotType_Spherical_6DOF_Service)
      {
       lcm_status_.joint_torque[6] = 0;
      }

      if (lcm_status_.num_fingers) {
        lcm_status_.finger_torque[0] = current_torque.Fingers.Finger1;
        lcm_status_.finger_torque[1] = current_torque.Fingers.Finger2;
        lcm_status_.finger_torque[2] = current_torque.Fingers.Finger3;
      }
    }

    if (msgs_sent_ % 10 == 0) {
      AngularPosition current_current;
      SdkGetAngularCurrent(current_current);
      lcm_status_.joint_current[0] = current_current.Actuators.Actuator1;
      lcm_status_.joint_current[1] = current_current.Actuators.Actuator2;
      lcm_status_.joint_current[2] = current_current.Actuators.Actuator3;
      lcm_status_.joint_current[3] = current_current.Actuators.Actuator4;
      lcm_status_.joint_current[4] = current_current.Actuators.Actuator5;
      lcm_status_.joint_current[5] = current_current.Actuators.Actuator6;
      lcm_status_.joint_current[6] = current_current.Actuators.Actuator7;
      if(glbl_deviceid == eRobotType_Spherical_6DOF_Service)
      {
       lcm_status_.joint_current[6] = 0;
      }

      if (lcm_status_.num_fingers) {
        lcm_status_.finger_current[0] = current_current.Fingers.Finger1;
        lcm_status_.finger_current[1] = current_current.Fingers.Finger2;
        lcm_status_.finger_current[2] = current_current.Fingers.Finger3;
      }
    }

    if (msgs_sent_ % 10 == 5) {
      GeneralInformations general_info;
      SdkGetGeneralInformations(general_info);
      lcm_extended_status_.time_from_startup = general_info.TimeFromStartup;
      lcm_extended_status_.supply_voltage = general_info.SupplyVoltage;
      lcm_extended_status_.total_current = general_info.TotalCurrent;
      lcm_extended_status_.power = general_info.Power;
      lcm_extended_status_.average_power = general_info.AveragePower;

      lcm_extended_status_.acceleration[0] = general_info.AccelerationX;
      lcm_extended_status_.acceleration[1] = general_info.AccelerationY;
      lcm_extended_status_.acceleration[2] = general_info.AccelerationZ;

      for (int i = 0; i < 4; ++i) {
        lcm_extended_status_.peripherals_connected[i] =
            general_info.PeripheralsConnected[i];
        lcm_extended_status_.peripherals_device_id[i] =
            general_info.PeripheralsDeviceID[i];
      }


      for (int i = 0; i < lcm_extended_status_.num_joints; ++i) {
        lcm_extended_status_.actuator_temperature[i] =
            general_info.ActuatorsTemperatures[i];
      }

      for (int i = 0; i < lcm_extended_status_.num_fingers; ++i) {
        lcm_extended_status_.finger_temperature[i] =
            general_info.FingersTemperatures[i];
      }

      lcm_extended_status_.utime = GetTime();
      lcm_.publish(FLAGS_lcm_extended_status_channel, &lcm_extended_status_);
    }

    lcm_status_.utime = GetTime();
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status_);
    ++msgs_sent_;

  }

  lcm::LCM lcm_;
  drake::lcmt_jaco_status lcm_status_{};
  drake_jaco_driver::lcmt_jaco_extended_status lcm_extended_status_{};
  TrajectoryPoint commanded_velocity_;
  int64_t msgs_sent_{0};
  timer_t timer_id_{};
  struct itimerspec timer_value_{};
};

}  // namespace

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout<<"*************************************\n";
  std::cout<<"***** Welcome to kinova driver*******\n";
  std::cout<<"*************************************\n";

  if (InitializeApi() != NO_ERROR_KINOVA) {
    return 1;
  }

  if (!FLAGS_optimal_z.empty()) {
    std::ifstream z_file(FLAGS_optimal_z);
    if (!z_file.is_open()) {
      std::cerr << "Unable to open: " << FLAGS_optimal_z << "\n";
      return 1;
    }
    float optimal_z[GRAVITY_PARAM_SIZE];
    memset(optimal_z, 0, sizeof(optimal_z));

    std::cout << "optimal z: ";
    for (int i = 0; i < OPTIMAL_Z_PARAM_SIZE_7DOF; ++i) {
      std::string line;
      if (!std::getline(z_file, line)) {
        std::cerr << "Failed reading z parameters\n";
        return 1;
      }
      optimal_z[i] = std::atof(line.c_str());
      std::cout << optimal_z[i] << " ";
    }
    std::cout <<  "\n";

    // I'm not sure what to make of the arm returning JACO_NACK_NORMAL, but
    // the parameters still seem to improve gravity estimation even when we
    // get that result code.
    int result = SdkSetGravityOptimalZParam(optimal_z);
    if (result != NO_ERROR_KINOVA && result != JACO_NACK_NORMAL) {
      std::cerr << "Failed to set optimal z parameters: " << result << "\n";
      //return 1;
    }

    result = SdkSetGravityType(OPTIMAL);
    if (result != NO_ERROR_KINOVA) {
      std::cerr << "Failed to set gravity type: " << result << "\n";
      return 1;
    }
    std::cout << "Set optimal Z parameters.\n";
  } else {
    int result = SdkSetGravityType(MANUAL_INPUT);
    if (result != NO_ERROR_KINOVA) {
      std::cerr << "Failed to set gravity type: " << result << "\n";
      return 1;
    }
  }
  std::cout << "launching the kinova run command\n";
  int devicetype = 0;
  devicetype = GetSelectedDeviceType();
  KinovaDriver(devicetype).Run();
  std::cout << "termination the kinova run command\n";

  return 0;
}
