# -*- python -*-

package(default_visibility = ["//visibility:public"])

# We need the ethernet headers too (see comment in kinova_driver.cc).
cc_library(
    name = "usb_command_layer",
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    linkopts = ["-ldl"],
    srcs = [
        "Kinova.API.USBCommandLayerUbuntu.so",
        "Kinova.API.CommLayerUbuntu.so",
    ],
)
