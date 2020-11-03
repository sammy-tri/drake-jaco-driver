#pragma once

#include <string>

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
