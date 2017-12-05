# -*- python -*-

package(default_visibility = ["//visibility:public"])

# Copy the .so provided by Kinova to start with the prefix "lib"
# (bazel is very insistent on this point).
genrule(
    name = "usb_command_layer_lib",
    srcs = ["Kinova.API.USBCommandLayerUbuntu.so"],
    outs = ["libKinova.API.USBCommandLayerUbuntu.so"],
    cmd = "cp \"$<\" \"$@\"",
)

# We need the ethernet headers too (see comment in kinova_driver.cc).
cc_library(
    name = "usb_command_layer",
    hdrs = [
        "Kinova.API.EthCommLayerUbuntu.h",
        "Kinova.API.UsbCommandLayerUbuntu.h",
        "KinovaTypes.h",
        "Kinova.API.CommLayerUbuntu.h",
        ],
    linkstatic = 1,
    srcs = [":usb_command_layer_lib"],
)
