January 21 2021
  Initial change notification to support arms that are supported by kinova robot arms

# drake-jaco-driver

This repository contains the application code used to communicate with
the Kinova Jaco arm from Drake.  It currently supports communicating
with an arm via USB or Ethernet.

## Building the driver

The `WORKSPACE` file assumes that the Kinova SDK is installed in
`/opt/JACO-SDK/`.

To build, run `bazel build //...`.  This will output two versions of
the driver: `bazel-bin/src/kinova_driver` and
`bazel-bin/src/kinova_driver_ethernet`.

## Driver operation

 To use the ethernet version, the arm needs to be configured
appropriately in the "Ethernet" pane of `DevelopmentCenter` (from the
Kinova SDK), and the appropriate IP and port numbers passed to
`bazel-bin/src/kinova_driver_ethernet` on the command line.  See the
usage message for ``bazel-bin/src/kinova_driver_ethernet` for specific
flags.

When controlling the arm, the driver operates in joint velocity mode.

The driver communicates using two LCM messages (defined in drake): `lcmt_jaco_status` and `lcmt_jaco_command`.

The main polling loop runs at 100Hz, and does the following:

* Read status from arm and publish on LCM channel `KINOVA_JACO_STATUS`
* Check for incoming `KINOVA_JACO_COMMAND` messages.
 * If a message has been received, send a joint velocity command to the arm with the specified values.
 * If no message is received before the next status should be published, no command is sent to the arm.  This will cause the arm to stop moving after a short period of time.

## Issues

The arm periodically seems to stop communicating (or refuse to
initialize) over USB.  The reason for this is not knon, but unplugging
and plugging back in the USB port (and restarting the driver) usually
seems to fix it.

Enumerating the connected arms occasionaly returns garbage data in
some of the device entries.  Generally starting the driver again will
find the target device successfully, though this may require multiple
attempts.

A single driver instance cannot control multiple arms.  Multiple
driver instances are required, either using different LCM URLs or
different channel names.

## Calibration

The build process also generates a calibration program in
`bazel-bin/src/kinova_calibration`.  When run, this program will move
the arm to the vertical position, and reset the actuator torque
sensors to zero.

If run with the `--gravity` argument, it will run the robot's gravity
Z estimation sequence.  This takes quite some time, and requires an
obstacle free area around the robot base of approximately the length
of the fully extended arm.  When the process completes, the Kinova API
will write a file named `ParametersOptimal_Z.txt`.  When running the
driver, this file can be passed as a command line argument to enable
optimal gravity estimation mode: `./bazel-bin/src/kinova_driver --optimal_z ParametersOptimal_Z.txt`

Example:
 build code:
bazel build //...
start the system up:
 bazel-bin/src/kinova_driver

Clean the build  
 bazel clean --expunge
https://github.com/sammy-tri/drake-jaco-driver
 
