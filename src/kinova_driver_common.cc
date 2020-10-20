#include "kinova_driver_common.h"

#include <signal.h>
#include <stdio.h>

#include <cstring>
#include <iostream>

// Even though we don't use the ethernet layer, some versions of the
// SDK refer to it from the USB command layer header but don't
// actually include the definition...  So we depend on the include
// order here.
#include "Kinova.API.EthCommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"

namespace {
void sighandler(int) {
  std::cerr << "Closing API.\n";
  CloseAPI();
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

int InitializeApi(const std::string& serial) {
  if (signal(SIGINT, sighandler) == SIG_ERR) {
    perror("Unable to set signal handler");
  }

  if (signal(SIGTERM, sighandler) == SIG_ERR) {
    perror("Unable to set signal handler");
  }

  std::cerr << "Initializing API" << std::endl;
  int result = InitAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Initialization failed: " << result << std::endl;
    return result;
  }

  KinovaDevice list[MAX_KINOVA_DEVICE];
  int device_count = GetDevices(list, result);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Unable to list devices: " << result << std::endl;
    return result;
  }

  if (device_count <= 0) {
    std::cerr << "No devices found." << std::endl;
    return UNKNOWN_ERROR;
  }

  int selected_device_id = -1;
  for (int device_id = 0; device_id < device_count; ++device_id) {
    if (IsValidDevice(list[device_id])) {
      std::cout << "Found USB robot " << device_id << " : "
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
      std::cout << "Skipping invalid device " << device_id << std::endl;
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

  result = SetActiveDevice(list[selected_device_id]);
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Setting active device failed: " << result << std::endl;
    return result;
  }

  result = StartControlAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Starting control API failed: " << result << std::endl;
    return result;
  }

  result = InitFingers();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Finger initialization failed: " << result << std::endl;
    return result;
  }

  SetAngularControl();

  return NO_ERROR_KINOVA;
}
