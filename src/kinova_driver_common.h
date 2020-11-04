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
#define MAYBE_ETHERNET(x) Ethernet_##x
#else
#define MAYBE_ETHERNET(x) x
#endif  // USE_ETHERNET


/// Initializes the Kinova API.  Returns NO_ERROR_KINOVA on success, or the
/// Kinova error code on failure (if the failure was not directly due to the
/// Kinova API, returns UNKNOWN_ERROR).  Installs a signal handler to close
/// the API for some signals.
///
/// Please ensure that gflags::ParseCommandLineFlags has been called before
/// infoking this function!
int InitializeApi();
