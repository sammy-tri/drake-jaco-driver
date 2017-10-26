#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cmath>
#include <iostream>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

// Even though we don't use the ethernet layer, some versions of the
// SDK refer to it from the USB command layer header but don't
// actually include the definition...  So we depend on the include
// order here.
#include <Kinova.API.EthCommLayerUbuntu.h>
#include <Kinova.API.UsbCommandLayerUbuntu.h>

#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"

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
  KinovaDriver() {
    lcm_.subscribe(FLAGS_lcm_command_channel,
                   &KinovaDriver::HandleCommandMessage,
                   this);
  }

  void Run() {
    // Reading the USB values actually takes a while, so keep track of
    // how long until we want to publish again.
    int64_t time_for_next_step = GetTime();
    while (true) {
      time_for_next_step += kKinovaUpdateIntervalUs;
      PublishStatus();

      int64_t remaining_time = time_for_next_step - GetTime();
      do {
        if (remaining_time < 0) {
          remaining_time = 0;
        }
        lcm_.handleTimeout(remaining_time / 1000);
        remaining_time = time_for_next_step - GetTime();
      } while (remaining_time > 0);
    }
  }

 private:
  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const drake::lcmt_jaco_command* command) {
    TrajectoryPoint commanded_velocity;
    commanded_velocity.InitStruct();
    commanded_velocity.LimitationsActive = 0;
    commanded_velocity.SynchroType = 0;
    commanded_velocity.Position.Type = ANGULAR_VELOCITY;
    commanded_velocity.Position.HandMode = VELOCITY_MODE;

    // TODO(sam.creasey) figure out how to detect the correct number
    // of joints/fingers.
    if (command->num_joints > 0) {
      assert(command->num_joints == 7);
      AngularInfo* actuators = &commanded_velocity.Position.Actuators;
      actuators->Actuator1 = to_degrees(command->joint_velocity[0]);
      actuators->Actuator2 = to_degrees(command->joint_velocity[1]);
      actuators->Actuator3 = to_degrees(command->joint_velocity[2]);
      actuators->Actuator4 = to_degrees(command->joint_velocity[3]);
      actuators->Actuator5 = to_degrees(command->joint_velocity[4]);
      actuators->Actuator6 = to_degrees(command->joint_velocity[5]);
      actuators->Actuator7 = to_degrees(command->joint_velocity[6]);
    }

    if (command->num_fingers > 0) {
      assert(command->num_fingers == 3);
      FingersPosition* fingers = &commanded_velocity.Position.Fingers;
      fingers->Finger1 = to_degrees(command->finger_velocity[0]);
      fingers->Finger2 = to_degrees(command->finger_velocity[1]);
      fingers->Finger3 = to_degrees(command->finger_velocity[2]);
    }
    SendBasicTrajectory(commanded_velocity);
  }

  void PublishStatus() {
    drake::lcmt_jaco_status lcm_status;

    // TODO(sam.creasey) figure out how to detect the correct number
    // of joints/fingers.
    lcm_status.num_joints = 7;
    lcm_status.joint_position.resize(lcm_status.num_joints, 0);
    lcm_status.joint_velocity.resize(lcm_status.num_joints, 0);
    lcm_status.joint_torque.resize(lcm_status.num_joints, 0);
    lcm_status.joint_current.resize(lcm_status.num_joints, 0);

    lcm_status.num_fingers = 3;
    lcm_status.finger_position.resize(lcm_status.num_fingers, 0);
    lcm_status.finger_velocity.resize(lcm_status.num_fingers, 0);
    lcm_status.finger_torque.resize(lcm_status.num_fingers, 0);
    lcm_status.finger_current.resize(lcm_status.num_fingers, 0);

    AngularPosition current_position;
    GetAngularPosition(current_position);
    lcm_status.joint_position[0] =
        to_radians(current_position.Actuators.Actuator1);
    lcm_status.joint_position[1] =
        to_radians(current_position.Actuators.Actuator2);
    lcm_status.joint_position[2] =
        to_radians(current_position.Actuators.Actuator3);
    lcm_status.joint_position[3] =
        to_radians(current_position.Actuators.Actuator4);
    lcm_status.joint_position[4] =
        to_radians(current_position.Actuators.Actuator5);
    lcm_status.joint_position[5] =
        to_radians(current_position.Actuators.Actuator6);
    lcm_status.joint_position[6] =
        to_radians(current_position.Actuators.Actuator7);
    lcm_status.finger_position[0] =
        to_radians(current_position.Fingers.Finger1);
    lcm_status.finger_position[1] =
        to_radians(current_position.Fingers.Finger2);
    lcm_status.finger_position[2] =
        to_radians(current_position.Fingers.Finger3);

    AngularPosition current_velocity;
    GetAngularVelocity(current_velocity);
    lcm_status.joint_velocity[0] =
        to_radians(current_velocity.Actuators.Actuator1);
    lcm_status.joint_velocity[1] =
        to_radians(current_velocity.Actuators.Actuator2);
    lcm_status.joint_velocity[2] =
        to_radians(current_velocity.Actuators.Actuator3);
    lcm_status.joint_velocity[3] =
        to_radians(current_velocity.Actuators.Actuator4);
    lcm_status.joint_velocity[4] =
        to_radians(current_velocity.Actuators.Actuator5);
    lcm_status.joint_velocity[5] =
        to_radians(current_velocity.Actuators.Actuator6);
    lcm_status.joint_velocity[6] =
        to_radians(current_velocity.Actuators.Actuator7);
    lcm_status.finger_velocity[0] =
        to_radians(current_velocity.Fingers.Finger1);
    lcm_status.finger_velocity[1] =
        to_radians(current_velocity.Fingers.Finger2);
    lcm_status.finger_velocity[2] =
        to_radians(current_velocity.Fingers.Finger3);

    AngularPosition current_current;
    GetAngularCurrent(current_current);
    lcm_status.joint_current[0] = current_current.Actuators.Actuator1;
    lcm_status.joint_current[1] = current_current.Actuators.Actuator2;
    lcm_status.joint_current[2] = current_current.Actuators.Actuator3;
    lcm_status.joint_current[3] = current_current.Actuators.Actuator4;
    lcm_status.joint_current[4] = current_current.Actuators.Actuator5;
    lcm_status.joint_current[5] = current_current.Actuators.Actuator6;
    lcm_status.joint_current[6] = current_current.Actuators.Actuator7;
    lcm_status.finger_current[0] = current_current.Fingers.Finger1;
    lcm_status.finger_current[1] = current_current.Fingers.Finger2;
    lcm_status.finger_current[2] = current_current.Fingers.Finger3;

    AngularPosition current_torque;
    GetAngularForce(current_torque);
    lcm_status.joint_torque[0] = current_torque.Actuators.Actuator1;
    lcm_status.joint_torque[1] = current_torque.Actuators.Actuator2;
    lcm_status.joint_torque[2] = current_torque.Actuators.Actuator3;
    lcm_status.joint_torque[3] = current_torque.Actuators.Actuator4;
    lcm_status.joint_torque[4] = current_torque.Actuators.Actuator5;
    lcm_status.joint_torque[5] = current_torque.Actuators.Actuator6;
    lcm_status.joint_torque[6] = current_torque.Actuators.Actuator7;
    lcm_status.finger_torque[0] = current_torque.Fingers.Finger1;
    lcm_status.finger_torque[1] = current_torque.Fingers.Finger2;
    lcm_status.finger_torque[2] = current_torque.Fingers.Finger3;

    lcm_status.utime = GetTime();
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status);
  }

  lcm::LCM lcm_;
};

}  // namespace

int main(int argc, char** argv) {
  int result = InitAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Initialization failed: " << result << std::endl;
    return 1;
  }

  result = InitFingers();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Finger initialization failed: " << result << std::endl;
    return 1;
  }

  KinovaDevice list[MAX_KINOVA_DEVICE];
  int device_count = GetDevices(list, result);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Unable to list devices: " << result << std::endl;
    return 1;
  }

  if (device_count <= 0) {
    std::cerr << "No devices found." << std::endl;
    return 1;
  }

  if (device_count > 1) {
    std::cerr << "Only one arm supported at this time." << std::endl;
    return 1;
  }

  // TODO(sam.creasey) Support selecting a different arm.
  const int device_id = 0;
  std::cout << "Found USB robot: " << list[device_id].Model << " ("
            << list[device_id].SerialNumber << ")"
            << " Firmware: " << list[device_id].VersionMajor << "."
            << list[device_id].VersionMinor << "."
            << list[device_id].VersionRelease
            << " Type: " << list[device_id].DeviceType
            << " ID: " << list[device_id].DeviceID << std::endl;

  if (list[device_id].DeviceType != eRobotType_Spherical_7DOF_Service) {
    std::cerr << "Untested robot type: " << list[device_id].DeviceType
              << std::endl;
  }

  KinovaDriver().Run();

  return device_count;
}
