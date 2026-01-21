#!/usr/bin/env bash
set -euo pipefail

# Absolute output root outside the workspace (expanded now).
OUT_ROOT="/home/zirgham/.bazel_output_quadcopter"
mkdir -p "$OUT_ROOT"
chmod 700 "$OUT_ROOT"

# Make sure Bazel uses that output root for this build.
bazel --output_user_root="$OUT_ROOT" shutdown || true
bazel --output_user_root="$OUT_ROOT" build //:quadcopter --cxxopt='-std=c++20' --verbose_failures
