#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"

#include "kinova_driver_common.h"

namespace {
// Kinova says 100Hz is the proper frequency for joint velocity
// updates.  See
// https://github.com/Kinovarobotics/kinova-ros#velocity-control-joint-space-and-cartesian-space
const int kKinovaUpdateIntervalUs = 10000;
const char* kLcmCommandChannel = "KINOVA_JACO_COMMAND";
const char* kLcmStatusChannel = "KINOVA_JACO_STATUS";
}  // namespace

DEFINE_string(lcm_command_channel, kLcmCommandChannel,
              "Channel to receive LCM command messages on");
DEFINE_string(lcm_status_channel, kLcmStatusChannel,
              "Channel to send LCM status messages on");
DEFINE_string(lcm_url, "", "LCM URL for Jaco driver");
DEFINE_string(optimal_z, "",
              "A file containing the optimal z parameters for this robot.");
DEFINE_double(joint_command_factor, 1.,
              "A multiplier to apply to received joint velocity commands.");
DEFINE_double(joint_status_factor, 1.,
              "A multiplier to apply to all reported joint velocities.");

namespace {

int64_t GetTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000000L + tv.tv_usec;
}

// The Kinova SDK communicates joint positions in degrees, which would
// be unusual in drake.  Convert to/from radians appropriately.
double to_degrees(double radians) { return radians * (180.0 / M_PI); }
double to_radians(double degrees) { return degrees * (M_PI / 180.0); }

class KinovaDriver {
 public:
  KinovaDriver(int devicetype)
      : lcm_(FLAGS_lcm_url) {
    // TODO(sam.creasey) figure out how to detect the correct number
    if(devicetype ==eRobotType_Spherical_7DOF_Service )
    {
     std::cout<<"number of joints on robot are 7\n";
     lcm_status_.num_joints = 7;
    }
    else
    {
     std::cout<<"number of joints on robot are 6\n";
     lcm_status_.num_joints = 6;
    }
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
      lcm_status_.finger_position.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_velocity.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_torque.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_torque_external.resize(lcm_status_.num_fingers, 0);
      lcm_status_.finger_current.resize(lcm_status_.num_fingers, 0);
    }

    commanded_velocity_.InitStruct();
    commanded_velocity_.LimitationsActive = 0;
    commanded_velocity_.SynchroType = 1;
    commanded_velocity_.Position.Type = ANGULAR_VELOCITY;
    commanded_velocity_.Position.HandMode = VELOCITY_MODE;

    lcm::Subscription* sub =
        lcm_.subscribe(FLAGS_lcm_command_channel,
                       &KinovaDriver::HandleCommandMessage,
                       this);
    sub->setQueueCapacity(2);
  }

  void Run() {
    // Reading the USB values actually takes a while, so keep track of
    // how long until we want to publish again.
    int64_t time_for_next_step = GetTime();
    while (true) {
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
  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const drake::lcmt_jaco_command* command) {
    // TODO(sam.creasey) figure out how to detect the correct number
    // of joints/fingers.
    if (command->num_joints > 0) {
      assert(command->num_joints == 7);
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
    }

    if (command->num_fingers > 0) {
      assert(command->num_fingers == 3);
      FingersPosition* fingers = &commanded_velocity_.Position.Fingers;
      fingers->Finger1 = to_degrees(command->finger_velocity[0]);
      fingers->Finger2 = to_degrees(command->finger_velocity[1]);
      fingers->Finger3 = to_degrees(command->finger_velocity[2]);
    }

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
      if (lcm_status_.num_fingers) {
        lcm_status_.finger_torque[0] = current_torque.Fingers.Finger1;
        lcm_status_.finger_torque[1] = current_torque.Fingers.Finger2;
        lcm_status_.finger_torque[2] = current_torque.Fingers.Finger3;
      }
    }

    if (msgs_sent_ % 5 == 0) {
      AngularPosition current_current;
      SdkGetAngularCurrent(current_current);
      lcm_status_.joint_current[0] = current_current.Actuators.Actuator1;
      lcm_status_.joint_current[1] = current_current.Actuators.Actuator2;
      lcm_status_.joint_current[2] = current_current.Actuators.Actuator3;
      lcm_status_.joint_current[3] = current_current.Actuators.Actuator4;
      lcm_status_.joint_current[4] = current_current.Actuators.Actuator5;
      lcm_status_.joint_current[5] = current_current.Actuators.Actuator6;
      lcm_status_.joint_current[6] = current_current.Actuators.Actuator7;
      if (lcm_status_.num_fingers) {
        lcm_status_.finger_current[0] = current_current.Fingers.Finger1;
        lcm_status_.finger_current[1] = current_current.Fingers.Finger2;
        lcm_status_.finger_current[2] = current_current.Fingers.Finger3;
      }
    }

    lcm_status_.utime = GetTime();
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status_);
    ++msgs_sent_;

  }

  lcm::LCM lcm_;
  drake::lcmt_jaco_status lcm_status_{};
  TrajectoryPoint commanded_velocity_;
  int64_t msgs_sent_{0};
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
