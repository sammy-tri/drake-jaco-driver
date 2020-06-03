#include "kinova_driver_common.h"

#include <signal.h>

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
}

int InitializeApi() {
  if (signal(SIGINT, sighandler) == SIG_ERR) {
    perror("Unable to set signal handler");
  }

  if (signal(SIGTERM, sighandler) == SIG_ERR) {
    perror("Unable to set signal handler");
  }

  int result = InitAPI();
  if (result != NO_ERROR_KINOVA) {
    std::cerr << "Initialization failed: " << result << std::endl;
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

  if (device_count > 1) {
    std::cerr << "Only one arm supported at this time." << std::endl;
    return UNKNOWN_ERROR;
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

  return NO_ERROR_KINOVA;
}
