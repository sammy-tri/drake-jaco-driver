#include "kinova_driver_common.h"

#include <arpa/inet.h>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

#include <gflags/gflags.h>

DEFINE_string(serial, "",
              "Serial number of robot to control.");
DEFINE_string(actuator_pid_0, "",
              "If specified, set the PID parameters for the first actuator to "
              "the 3 comma delimited values specified here.  See other "
              "flags for other actuators.");
DEFINE_string(actuator_pid_1, "", "");
DEFINE_string(actuator_pid_2, "", "");
DEFINE_string(actuator_pid_3, "", "");
DEFINE_string(actuator_pid_4, "", "");
DEFINE_string(actuator_pid_5, "", "");
DEFINE_string(actuator_pid_6, "", "");

#ifdef USE_ETHERNET
DEFINE_string(local_ip_addr, "",
              "Local IP address for ethernet communication.");
DEFINE_string(netmask, "255.255.255.0", "Netmask.");
DEFINE_string(robot_ip_addr, "", "Robot IP address.");
DEFINE_int32(local_cmd_port, 25015, "Local command port");
DEFINE_int32(local_broadcast_port, 25025, "Local broadcast port");
DEFINE_int32(robot_port, 55000, "Robot port");

const int kRxTimeoutMs = 1000;
#endif

namespace {
void sighandler(int) {
  std::cerr << "Closing API.\n";
  SdkCloseAPI();
  exit(0);
}

bool IsValidDevice(const KinovaDevice& device) {
  // Apply some heuristics to attempt to figure out if Kinova indicated that a
  // device was found but failed to populate it.  This list will potentially
  // grow as we find more ways for junk to sneak out of the API.
  if (static_cast<unsigned char>(device.DeviceType) == eRobotType_Error) {
    return false;
  }

  return true;
}

void HandleWatchdogSignal(int signal) noexcept {
  std::cerr << "Lost communication with robot.  Exiting.";
  abort();
}

}  // namespace

int InitializeApi() {
  if (signal(SIGINT, sighandler) == SIG_ERR) {
    perror("Unable to set signal handler");
  }

  if (signal(SIGTERM, sighandler) == SIG_ERR) {
    perror("Unable to set signal handler");
  }

  std::cerr << "Initializing API" << std::endl;
#ifdef USE_ETHERNET
  EthernetCommConfig comm_config;
  comm_config.localIpAddress = inet_addr(FLAGS_local_ip_addr.c_str());
  comm_config.subnetMask = inet_addr(FLAGS_netmask.c_str());
  comm_config.robotIpAddress = inet_addr(FLAGS_robot_ip_addr.c_str());
  comm_config.localCmdport = FLAGS_local_cmd_port;
  comm_config.localBcastPort = FLAGS_local_broadcast_port;
  comm_config.robotPort = FLAGS_robot_port;
  comm_config.rxTimeOutInMs = kRxTimeoutMs;

  int result = Ethernet_InitEthernetAPI(comm_config);
#else
  int result = InitAPI();
#endif

  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Initialization failed: " << result << std::endl;
    return result;
  }

  KinovaDevice list[MAX_KINOVA_DEVICE];
#ifdef USE_ETHERNET
  std::cerr << "Refreshing device list" << std::endl;
  result = Ethernet_RefresDevicesList();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Refreshing device list failed: " << result << std::endl;
    return result;
  }
#endif

  std::cerr << "Getting devices" << std::endl;
  int device_count = SdkGetDevices(list, result);

  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Unable to list devices: " << result << std::endl;
    return result;
  }

  if (device_count <= 0) {
    std::cerr << "No devices found." << std::endl;
    return UNKNOWN_ERROR;
  }

  const std::string serial = FLAGS_serial;

  int selected_device_id = -1;
  for (int device_id = 0; device_id < device_count; ++device_id) {
    if (IsValidDevice(list[device_id])) {
      std::cerr << "Found robot " << device_id << " : "
                << list[device_id].Model << " ("
                << list[device_id].SerialNumber << ")"
                << " Firmware: " << list[device_id].VersionMajor << "."
                << list[device_id].VersionMinor << "."
                << list[device_id].VersionRelease
                << " Type: " << list[device_id].DeviceType
                << " ID: " << list[device_id].DeviceID << std::endl;
      if (selected_device_id == -1 &&
          (serial.empty() || serial == list[device_id].SerialNumber)) {
        selected_device_id = device_id;
      }
    } else {
      std::cerr << "Skipping invalid device " << device_id << std::endl;
    }
  }

  if (selected_device_id == -1) {
    std::cerr << "Robot " << serial << " not found." << std::endl;
    return UNKNOWN_ERROR;
  } else {
    std::cerr << "Selecting robot " << selected_device_id << "\n";
  }

  if (list[selected_device_id].DeviceType !=
      eRobotType_Spherical_7DOF_Service) {
    std::cerr << "Untested robot type: "
              << list[selected_device_id].DeviceType
              << std::endl;
  }

  result = SdkSetActiveDevice(list[selected_device_id]);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Setting active device failed: " << result << std::endl;
    return result;
  }

  result = SdkStartControlAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Starting control API failed: " << result << std::endl;
    return result;
  }

  result = SdkInitFingers();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Finger initialization failed: " << result << std::endl;
    return result;
  }

  SdkSetAngularControl();

  if (!FLAGS_actuator_pid_0.empty()) {
    result = SetActuatorPidFromString(16, FLAGS_actuator_pid_0);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  if (!FLAGS_actuator_pid_1.empty()) {
    result = SetActuatorPidFromString(17, FLAGS_actuator_pid_1);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  if (!FLAGS_actuator_pid_2.empty()) {
    result = SetActuatorPidFromString(18, FLAGS_actuator_pid_2);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  if (!FLAGS_actuator_pid_3.empty()) {
    result = SetActuatorPidFromString(19, FLAGS_actuator_pid_3);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  if (!FLAGS_actuator_pid_4.empty()) {
    result = SetActuatorPidFromString(20, FLAGS_actuator_pid_4);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  if (!FLAGS_actuator_pid_5.empty()) {
    result = SetActuatorPidFromString(21, FLAGS_actuator_pid_5);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  if (!FLAGS_actuator_pid_6.empty()) {
    result = SetActuatorPidFromString(25, FLAGS_actuator_pid_6);
    if (result != NO_ERROR_KINOVA) {
      return result;
    }
  }

  return NO_ERROR_KINOVA;
}

int SetGravity(const std::string& optimal_z) {
  if (!optimal_z.empty()) {
    std::ifstream z_file(optimal_z);
    if (!z_file.is_open()) {
      std::cerr << "Unable to open: " << optimal_z << "\n";
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
  return 0;
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

int SetActuatorPidFromString(int address, const std::string& pid_string) {
  size_t start = 0;
  size_t next_comma = pid_string.find(",");
  if (next_comma == std::string::npos) {
    throw std::runtime_error(
        std::string("Expected 3 values, got: ") + pid_string);
  }

  const float p = stof(pid_string.substr(start, next_comma - start));
  start = next_comma + 1;

  next_comma = pid_string.find(",", start);
  if (next_comma == std::string::npos) {
    throw std::runtime_error(
        std::string("Expected 3 values, got: ") + pid_string);
  }
  const float i = stof(pid_string.substr(start, next_comma - start));
  start = next_comma + 1;

  const float d = stof(pid_string.substr(start, std::string::npos));

  const int result = SdkSetActuatorPID(address, p, i, d);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Setting PID faied on address " << address << "\n";
    return result;
  }
  return NO_ERROR_KINOVA;
}
