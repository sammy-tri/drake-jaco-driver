# -*- python -*-

workspace(name = "drake_jaco_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "dc635f1e5d3ea5f7586c47adbc1004e6f7a2099f",
    "2d587175630183d01581bf644db752ebdac13ed80e61f4e68324f79e05d9249e",
)
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Download a specific commit of Drake, from github.
http_archive(
    name = "drake",
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

load("@drake//tools/workspace:default.bzl", "add_default_repositories")

# WARNING: We allow `vtk`, but should take care to avoid linking in multiple
# versions of VTK!
add_default_repositories()

new_local_repository(
    name = "jaco_sdk",
    path = "/opt/JACO-SDK/API",
    build_file = "tools/jaco_sdk.BUILD"
    )
