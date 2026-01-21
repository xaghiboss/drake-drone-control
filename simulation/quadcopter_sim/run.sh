#!/usr/bin/env bash
set -euo pipefail

OUT_ROOT="/home/zirgham/.bazel_output_quadcopter"
if [ ! -d "$OUT_ROOT" ]; then
  echo "No build found. Run ./build.sh first."
  exit 1
fi

# Run the binary directly (keeps terminal behavior correct)
./bazel-bin/quadcopter
