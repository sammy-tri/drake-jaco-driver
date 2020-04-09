# -*- python -*-

workspace(name = "drake_jaco_driver")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

(DRAKE_COMMIT, DRAKE_CHECKSUM) = (
    "92d8f2dc690e969a5d2ac83c2774d5d994d5c9d0",
    "5198b317e5ab918be805153a752c216f98ea4a45dab5f51078990942ebda6f67",
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
    path = "/opt/JACO2SDK/API",
    build_file = "tools/jaco_sdk.BUILD"
    )
