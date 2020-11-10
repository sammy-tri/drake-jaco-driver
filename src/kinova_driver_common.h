#pragma once

#include <string>

// Pull in the Kinova headers in this file, because the selection of which
// headers we include and in what order depends on compile time defs.

#ifdef USE_ETHERNET
#include "Kinova.API.EthCommandLayerUbuntu.h"

// I don't know why the Kinova API defines this twice, and differently.  It
// doesn't appear to be used.
#undef COMM_LAYER_PATH
#endif  // USE_ETHERNET

// Even if we're not using the ethernet layer, some versions of the SDK refer
// to it from the USB command layer header but don't actually include the
// definition...  So we depend on the include order here.
#include "Kinova.API.EthCommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"

// The Kinova API prefixes the Ethernet versions of API functions with the
// string "Ethernet_".  Define a macro we can use in other places to pick the
// correct version at compile time.
#ifdef USE_ETHERNET

#define SdkSendBasicTrajectory(x) Ethernet_SendBasicTrajectory(x);
#define SdkGetAngularPosition(x) Ethernet_GetAngularPosition(x);
#define SdkGetAngularVelocity(x) Ethernet_GetAngularVelocity(x);
#define SdkGetAngularForceGravityFree(x) Ethernet_GetAngularForceGravityFree(x);
#define SdkGetAngularForce(x) Ethernet_GetAngularForce(x);
#define SdkGetAngularCurrent(x) Ethernet_GetAngularCurrent(x);
#define SdkSetGravityOptimalZParam(x) Ethernet_SetGravityOptimalZParam(x);
#define SdkSetGravityType(x) Ethernet_SetGravityType(x);
#define SdkCloseAPI(x) Ethernet_CloseAPI(x);
#define SdkGetDevices(x, y) Ethernet_GetDevices(x, y);
#define SdkSetActiveDevice(x) Ethernet_SetActiveDevice(x);
#define SdkStartControlAPI(x) Ethernet_StartControlAPI(x);
#define SdkInitFingers(x) Ethernet_InitFingers(x);
#define SdkSetAngularControl(x) Ethernet_SetAngularControl(x);

#else

#define SdkSendBasicTrajectory(x) SendBasicTrajectory(x);
#define SdkGetAngularPosition(x) GetAngularPosition(x);
#define SdkGetAngularVelocity(x) GetAngularVelocity(x);
#define SdkGetAngularForceGravityFree(x) GetAngularForceGravityFree(x);
#define SdkGetAngularForce(x) GetAngularForce(x);
#define SdkGetAngularCurrent(x) GetAngularCurrent(x);
#define SdkSetGravityOptimalZParam(x) SetGravityOptimalZParam(x);
#define SdkSetGravityType(x) SetGravityType(x);
#define SdkCloseAPI(x) CloseAPI(x);
#define SdkGetDevices(x, y) GetDevices(x, y);
#define SdkSetActiveDevice(x) SetActiveDevice(x);
#define SdkStartControlAPI(x) StartControlAPI(x);
#define SdkInitFingers(x) InitFingers(x);
#define SdkSetAngularControl(x) SetAngularControl(x);

#endif  // USE_ETHERNET


/// Initializes the Kinova API.  Returns NO_ERROR_KINOVA on success, or the
/// Kinova error code on failure (if the failure was not directly due to the
/// Kinova API, returns UNKNOWN_ERROR).  Installs a signal handler to close
/// the API for some signals.
///
/// Please ensure that gflags::ParseCommandLineFlags has been called before
/// infoking this function!
int InitializeApi();
