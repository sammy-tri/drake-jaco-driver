# -*- python -*-

workspace(name = "drake_jaco_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "cc70b6b970b140e96f9687896379ab46cf1e4b12",
    "36a7a26cbbefb1c7afdeb76a0c06fa423dcaca1d788e994c3a9887dc32ddb934",
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
