# -*- python -*-

package(default_visibility = ["//visibility:public"])

# If both the USB and the Ethernet libraries are available, USB mode
# no longer works.  Split the libraries in two.

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

cc_library(
    name = "ethernet_command_layer",
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    linkopts = ["-ldl"],
    srcs = [
        "Kinova.API.EthCommandLayerUbuntu.so",
        "Kinova.API.EthCommLayerUbuntu.so",
    ],
)
