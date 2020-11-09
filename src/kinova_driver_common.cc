#include "kinova_driver_common.h"

#include <arpa/inet.h>
#include <signal.h>
#include <stdio.h>

#include <cstring>
#include <iostream>

#include <gflags/gflags.h>

DEFINE_string(serial, "",
              "Serial number of robot to control.");

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

  return NO_ERROR_KINOVA;
}
