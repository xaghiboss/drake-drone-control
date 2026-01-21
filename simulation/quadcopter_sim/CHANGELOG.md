# Changelog

All notable changes to this project are documented here.

## v1.1 — 2026-01-21
Changes in this release:
- Added: `build.sh` and `run.sh` helper scripts that force Bazel to use a writable output root (`$HOME/.bazel_output_quadcopter`).
- Added: `.gitignore` to exclude Bazel outputs and editor temp files.
- Added: `bazel_dep(name = "rules_cc", version = "0.2.14")` to `MODULE.bazel` so C++ rules are available under Bzlmod.
- Changed: `BUILD.bazel` to explicitly `load("@rules_cc//cc:defs.bzl", "cc_binary")`.
- Fixed: build failures caused by read-only system caches and missing rule loads.

Notes
- `MODULE.bazel.lock` is committed to pin module versions for reproducible builds.
- Use semantic versioning (MAJOR.MINOR.PATCH) for future releases.
