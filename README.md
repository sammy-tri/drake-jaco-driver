# drake-jaco-driver

This repository contains the application code used to communicate with
the Kinova Jaco arm from Drake.  It currently supports communicating
with a single arm via USB.

## Building the driver

The `WORKSPACE` file assumes that the Kinova SDK is installed in
`/opt/JACO2SDK/`.

To build, run `bazel build //src:kinova_driver`.  This will output an
executable in `bazel-bin/src/kinova_driver`.

## Driver operation

When controlling the arm, the driver operates in joint velocity mode.

The driver communicates using two LCM messages (defined in drake): `lcmt_jaco_status` and `lcmt_jaco_command`.

The main polling loop runs at 100Hz, and does the following:

* Read status from arm and publish on LCM channel `KINOVA_JACO_STATUS`
* Check for incoming `KINOVA_JACO_COMMAND` messages.
 * If a message has been received, send a joint velocity command to the arm with the specified values.
 * If no message is received before the next status should be published, no command is sent to the arm.  This will cause the arm to stop moving after a short period of time.

## Issues

The arm periodically seems to stop communicating over USB.  The reason
for this is not knon, but unplugging and plugging back in the USB port
(and restarting the driver) usually seems to fix it.