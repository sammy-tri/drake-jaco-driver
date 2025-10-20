load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")

def _extensions_impl(module_ctx):
    new_local_repository(
        name = "jaco_sdk",
        build_file = "//tools:jaco_sdk.BUILD.bazel",
        path = "/opt/JACO-SDK/API",
    )
    return module_ctx.extension_metadata(reproducible = True)

extensions = module_extension(
    implementation = _extensions_impl,
)
