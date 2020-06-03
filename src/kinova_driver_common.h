#pragma once

/// Initializes the Kinova API.  Returns NO_ERROR_KINOVA on success, or the
/// Kinova error code on failure (if the failure was not directly due to the
/// Kinova API, returns UNKNOWN_ERROR).  Installs a signal handler to close
/// the API for some signals.
int InitializeApi();
