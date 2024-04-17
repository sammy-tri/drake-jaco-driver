#include <sys/time.h>
#include <unistd.h>

#include <cassert>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_jaco_command.hpp"
#include "drake/lcmt_jaco_status.hpp"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake_jaco_driver/lcmt_jaco_extended_status.hpp"

#include "kinova_driver_common.h"

namespace {

// This should be a value lower than the point where the Kinova API will start
// blocking (and reducing the effective command rate), which may be ~300Hz.
constexpr double kMaxSendReceiveFrequencyHz = 250.0;
constexpr double kMinSendReceiveIntervalSeconds =
    1.0 / kMaxSendReceiveFrequencyHz;
const char* kLcmCommandChannel = "KINOVA_JACO_COMMAND";
const char* kLcmStatusChannel = "KINOVA_JACO_STATUS";
const char* kLcmExtendedStatusChannel = "KINOVA_JACO_EXTENDED_STATUS";
constexpr double kJointLimitSafetyMarginDegree = 1;
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
DEFINE_bool(limit_actuator_range, false,
            "Limit actuator range to between zero and 360 degrees");
DEFINE_double(actuator_buffer_range, kJointLimitSafetyMarginDegree,
              "When limiting the actuator range, how much to reduce the "
              "limits by (in degrees)");

namespace {

constexpr int kKinovaSDKPackageLength = 50;
constexpr int kJointAddressOffset = 16;

int joint_to_address(int joint_index) {
  if (joint_index == 6) {
    return 25;
  }
  return joint_index + 16;
}

int address_to_joint(int joint_address) {
  if (joint_address == 25) {
    return 6;
  }
  return joint_address - 16;
}

class KinovaDriver {
 public:
  KinovaDriver()
      : lcm_(FLAGS_lcm_url) {
    // TODO(sam.creasey) figure out how to detect the correct number
    // of joints/fingers.
    lcm_status_.num_joints = 7;
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

    lcm_extended_status_.num_joints = lcm_status_.num_joints;
    lcm_extended_status_.actuator_temperature.resize(
        lcm_extended_status_.num_joints, 0);
    lcm_extended_status_.num_fingers = lcm_status_.num_fingers;
    if (lcm_extended_status_.num_fingers) {
      lcm_extended_status_.finger_temperature.resize(
        lcm_extended_status_.num_fingers, 0);
    }

    lcm::Subscription* sub = lcm_.subscribe(FLAGS_lcm_command_channel,
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
    GetJointOffsets(joint_offset_);
  }

  void Run() {
    InitializeStatus();

    int64_t time_for_next_step = GetTime();
    while (true) {
      Pet();

      time_for_next_step +=
          static_cast<int64_t>(kMinSendReceiveIntervalSeconds * 1e6);

      SendAndReceive();
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

  bool ParseActualPosition(const RS485_Message &msg, int joint_index) {
    constexpr int kDataIndexMotorCurrent = 0;  // Index of sensed motor current.
    constexpr int kDataIndexHallPosition = 1;  // Index of sensed motor position.
    constexpr int kDataIndexSpeed = 2;         // Index of sensed motor speed.
    constexpr int kDataIndexTorque = 3;        // Index of sensed torque.

    lcm_status_.joint_current[joint_index] =
        msg.DataFloat[kDataIndexMotorCurrent];
    lcm_status_.joint_position[joint_index] =
        to_radians(msg.DataFloat[kDataIndexHallPosition])
        - joint_offset_[joint_index];
    lcm_status_.joint_velocity[joint_index] =
        to_radians(msg.DataFloat[kDataIndexSpeed]);
    lcm_status_.joint_torque[joint_index] =
        -msg.DataFloat[kDataIndexTorque];
    return true;
  }

  int SendAndWaitForResponse() {
    constexpr int kResponseWaitTimeInMicroseconds = 50000;
    int received = 0;
    int sent = 0;
    int result = SdkOpenRS485_Write(pkg_out_, 1, sent);
    if (result != NO_ERROR_KINOVA) {
      throw std::runtime_error(
          "Error sending command during initialization.");
    }

    usleep(kResponseWaitTimeInMicroseconds);
    result = SdkOpenRS485_Read(pkg_in_, 1, received);
    return received;
}

  void InitializeStatus() {
    std::cerr << "Starting initialization\n";

    int result = SdkOpenRS485_Activate();
    if (result != 1) {
      throw std::runtime_error("Failed activating RS-485 mode");
    }

    // Wait a bit, and clear out spurious bytes on the RS-485 bus.
    constexpr int kInitializationWaitTimeInMicroseconds = 200000;
    usleep(kInitializationWaitTimeInMicroseconds);

    constexpr int kClearOutWaitTimeInMicroseconds = 1000;
    int n_rec = -1;
    while (n_rec != 0) {
      result = SdkOpenRS485_Read(pkg_in_, 1, n_rec);
      usleep(kClearOutWaitTimeInMicroseconds);
    }

    // Get initial positions first.
    // We need to give this multiple attempts to allow for RS-485 errors.
    constexpr int kNumTrials = 50;

    for (int i = 0; i < kMaxNumJoints; ++i) {
      int received = 0;
      const int joint_address = joint_to_address(i);
      for (int j = 0; j < kNumTrials; ++j) {
        pkg_out_[0].Command = RS485_MSG_GET_ACTUALPOSITION;
        pkg_out_[0].SourceAddress = 0;
        pkg_out_[0].DestinationAddress = joint_address;

        received = SendAndWaitForResponse();
        if (received > 0) {
          break;
        }
      }

      if (received != 1) {
        std::cerr << "Unable to get position of joint " << i
                  << ": " << received << " messages received (expected 1)"
                  << std::endl;
        throw std::runtime_error("Error reading initial position");
      }

      if (pkg_in_[0].Command != RS485_MSG_SEND_ACTUALPOSITION) {
        std::cerr << "Unexpected response from joint " << i
                  << ": " << pkg_in_[0].Command
                  << std::endl;
        throw std::runtime_error("Error reading initial position");
      }
      ParseActualPosition(pkg_in_[0], i);
      commanded_position_[i] = lcm_status_.joint_position[i];

      if (FLAGS_limit_actuator_range) {
        if ((commanded_position_[i] < 0) ||
            (commanded_position_[i] > 2 * M_PI)) {
          std::cerr << "Joint " << i << " out of range during initialization."
                    << std::endl;
          throw std::runtime_error("Joint out of range");
        }
      }

      const PidConfiguration* pid_config = GetConfiguredPid(joint_address);
      if (pid_config != nullptr) {
        for (int j = 0; j < kNumTrials; ++j) {
          pkg_out_[0].Command = RS485_MSG_KP_GAIN;
          pkg_out_[0].SourceAddress = 0;
          pkg_out_[0].DestinationAddress = joint_address;
          pkg_out_[0].DataFloat[0] = pid_config->p;
          pkg_out_[0].DataFloat[1] = pid_config->p;

          received = SendAndWaitForResponse();
          if (received > 0) {
            break;
          }
        }

        if (received != 1) {
          std::cerr << "Unable to set P configuration of joint " << i
                    << ": " << received << " messages received (expected 1)"
                    << std::endl;
          throw std::runtime_error("Error setting P configuration");
        }

        for (int j = 0; j < kNumTrials; ++j) {
          pkg_out_[0].Command = RS485_MSG_KI_KD_GAIN;
          pkg_out_[0].SourceAddress = 0;
          pkg_out_[0].DestinationAddress = joint_address;
          pkg_out_[0].DataFloat[0] = pid_config->d;
          pkg_out_[0].DataFloat[1] = pid_config->i;

          received = SendAndWaitForResponse();
          if (received > 0) {
            break;
          }
        }

        if (received != 1) {
          std::cerr << "Unable to set I,D configuration of joint " << i
                    << ": " << received << " messages received (expected 1)"
                    << std::endl;
          throw std::runtime_error("Error setting I,D configuration");
        }
      }
    }

    std::cerr << "Initialization complete.\n";
  }

  bool ParseSendAll1(const RS485_Message &msg, int joint_index) {
    //constexpr int kDataIndexCmdPosition = 0;   // Index of commanded position.
    constexpr int kDataIndexCurrent = 0;       // Index of sensed current.
    constexpr int kDataIndexHallPosition = 1;  // Index of hall sensor position.
    constexpr int kDataIndexSpeed = 2;         // Index of sensed speed.
    constexpr int kDataIndexTorque = 3;        // Index of sensed torque.

    lcm_status_.joint_current[joint_index] = msg.DataFloat[kDataIndexCurrent];
    lcm_status_.joint_position[joint_index] =
        to_radians(msg.DataFloat[kDataIndexHallPosition]) -
        joint_offset_[joint_index];
    lcm_status_.joint_velocity[joint_index] =
        to_radians(msg.DataFloat[kDataIndexSpeed]);
    lcm_status_.joint_torque[joint_index] = -msg.DataFloat[kDataIndexTorque];
    return true;
  }

  bool ParseSendAll2(const RS485_Message &msg, int joint_index) {
    //constexpr int kDataIndexCmdPosition = 0;   // Index of commanded position.
    lcm_extended_status_.actuator_temperature[joint_index] =
        static_cast<float>((msg.DataLong[3] & 0xffff0000) >> 16) / 100.;

    return true;
  }

  bool ParseSendAll3(const RS485_Message &msg, int joint_index) {
    if (joint_index == 1) {
      std::cerr << "SendAll3 "
                << msg.DataFloat[0] << " "
                << msg.DataFloat[1] << " "
                << msg.DataFloat[2] << " "
                << msg.DataFloat[3] << "\n";
    }

    lcm_status_.joint_current[joint_index] = msg.DataFloat[0];
    return true;
  }

  bool ParseAcknowledge(const RS485_Message &msg, int joint_index) {
    constexpr int kDataIndexCommand = 0;  // Index of command code in ACK message.
    std::cerr << "Got ACK from joint index "
              << joint_index << "Command: "
              << msg.DataLong[kDataIndexCommand];
    return true;
  }

  bool ParseMessage(const RS485_Message &msg) {
    int joint_index = address_to_joint(msg.SourceAddress);
    if (joint_index < 0) {
      std::cerr << "Invalid SourceAddress: "
		<< static_cast<int>(msg.SourceAddress) << std::endl;
      return false;
    }
    if (msg.DestinationAddress != 0) {
      std::cerr << "Invalid DestinationAddress: "
                << static_cast<int>(msg.DestinationAddress) << std::endl;
      return false;
    }

    switch (msg.Command) {
      // case RS485_MSG_SEND_ACTUALPOSITION:
      //   return ParseActualPosition(msg, joint_index);
      // case RS485_MSG_SEND_POSITION_CURRENT:
      //   return ParsePositionCurrent(msg, joint_index);
      case RS485_MSG_SEND_ALL_VALUES_1:
        return ParseSendAll1(msg, joint_index);
      case RS485_MSG_SEND_ALL_VALUES_2:
        return ParseSendAll2(msg, joint_index);
      case RS485_MSG_SEND_ALL_VALUES_3:
        return ParseSendAll3(msg, joint_index);
      case RS485_MSG_ACK:
        return ParseAcknowledge(msg, joint_index);
      // case RS485_MSG_REPORT_ERROR:
      //   return ParseReportError(msg, joint_index);
      default:
        std::cerr << "Invalid/Unsupported command code:"
                  << std::hex << msg.Command << std::dec
                  << std::endl;
        return false;
    }
    return true;
  }

  void SendAndReceive() {
    for (int i = 0; i < kMaxNumJoints; ++i) {
      pkg_out_[0].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
      pkg_out_[0].SourceAddress = 0;
      pkg_out_[0].DestinationAddress = joint_to_address(i);
      pkg_out_[0].DataFloat[0] =
          to_degrees(commanded_position_[i] + joint_offset_[i]);
      pkg_out_[0].DataFloat[1] = pkg_out_[0].DataFloat[0];
      pkg_out_[0].DataLong[2] = 0;  // Docs imply something should go here!?

      int sent = 0;
      int result = SdkOpenRS485_Write(pkg_out_, 1, sent);
      if (result != NO_ERROR_KINOVA) {
        std::cerr << "Failed sending command to joint " << i << std::endl;
        throw std::runtime_error("Command send failed.");
      }
    }

    int num_received_messages = 1;
    while (num_received_messages > 0) {
      int result = SdkOpenRS485_Read(pkg_in_, 1, num_received_messages);
      if (result != NO_ERROR_KINOVA) {
        throw std::runtime_error("Error reading RS485");
      }

      for (int i = 0; i < num_received_messages; i++) {
        if (!ParseMessage(pkg_in_[i])) {
          //throw std::runtime_error("Error parsing message");
        }
      }
    }
    constexpr int kYieldTimeInMicroseconds = 300;
    usleep(kYieldTimeInMicroseconds);
  }

  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const drake::lcmt_jaco_command* command) {
    for (int i = 0; i < command->num_joints; ++i) {
      if (FLAGS_limit_actuator_range) {
        const double joint_tol = to_radians(FLAGS_actuator_buffer_range);
        commanded_position_[i] =
            std::max(std::min(0 + joint_tol, command->joint_position[i]),
                     2 * M_PI - joint_tol);
      } else {
        commanded_position_[i] = command->joint_position[i];
      }
    }
  }

  void PublishStatus() {
    lcm_extended_status_.utime = GetTime();
    lcm_.publish(FLAGS_lcm_extended_status_channel, &lcm_extended_status_);

    lcm_status_.utime = GetTime();
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status_);
    ++msgs_sent_;
  }

  lcm::LCM lcm_;
  drake::lcmt_jaco_status lcm_status_{};
  drake_jaco_driver::lcmt_jaco_extended_status lcm_extended_status_{};

  float commanded_position_[kMaxNumJoints];
  RS485_Message pkg_in_[kKinovaSDKPackageLength];
  RS485_Message pkg_out_[kKinovaSDKPackageLength];
  int64_t msgs_sent_{0};
  timer_t timer_id_{};
  struct itimerspec timer_value_{};
  double joint_offset_[kMaxNumJoints];
  bool command_time_valid_{false};
  std::vector<double> hold_joint_position_;
};

}  // namespace

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (InitializeApi() != NO_ERROR_KINOVA) {
    return 1;
  }

  if (SetGravity(FLAGS_optimal_z) != 0) {
    return 1;
  }

  KinovaDriver().Run();

  return 0;
}
