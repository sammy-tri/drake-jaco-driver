# -*- python -*-

workspace(name = "drake_iiwa_driver")

new_local_repository(
    name = "jaco_sdk",
    path = "/opt/JACO2SDK/API",
    build_file = "tools/jaco_sdk.BUILD"
    )

local_repository(
    name = "kythe",
    path = "tools/third_party/com_github_google_kythe",
)

load("@kythe//tools/build_rules/config:pkg_config.bzl", "pkg_config_package")

pkg_config_package(
    name = "glib",
    modname = "glib-2.0",
)

load("//tools:github.bzl", "github_archive")

github_archive(
    name = "drake",
    repository = "RobotLocomotion/drake",
    commit = "650cf04d17c9861077deed9d1b4b36855c1dfd94",
    sha256 = "3893ff266a18a718c8a05736adab4dae5edbdccf3899ef0c8a81864c4de312fa",  # noqa
)

github_archive(
    name = "gflags",
    repository = "gflags/gflags",
    commit = "a69b2544d613b4bee404988710503720c487119a",
    sha256 = "8b3836d5ca34a2da4d6375cf5f2030c719b508ca16014fcc9d5e9b295b56a6c1",
)

github_archive(
    name = "lcm",
    repository = "lcm-proj/lcm",
    commit = "c0a0093a950fc83e12e8d5918a0319b590356e7e",
    sha256 = "f967e74e639ea56318242e93c77a15a504345c8200791cab70d9dad86aa969b2",  # noqa
    build_file = "tools/lcm.BUILD",
)
