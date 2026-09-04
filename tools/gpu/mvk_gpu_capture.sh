#!/bin/bash
# Capture a Metal GPU trace of the divergent-gather compute pre-pass + the
# skinned vertex shaders, for inspection in Xcode's Metal debugger.
#
# Usage:  tools/gpu/mvk_gpu_capture.sh [trace.xtr]
# Output: /tmp/dg/skin.gputrace   (open with:  open /tmp/dg/skin.gputrace )
#
# In Xcode: the frame nav on the left lists every encoder. Find the compute
# dispatch ("main0" kernel, the divergent-gather pre-pass) - select it, open
# the bound buffers, and look at xe_divergent_gather's contents. Then find the
# following vertex encoder for the same skinned draw and compare. The diag
# build (XE_DGATHER_DIAG=1, see dg_diag.sh) writes a0 into reserved gather
# slots so you can read the pre-pass a0 and the vertex-shader a0 directly.

set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN="$ROOT/build/bin/macOS/Checked/xenia-gpu-vulkan-trace-dump"
TRACE="${1:-$ROOT/build/scratch/gpu/4D5307E6_149373.xtr}"
OUT="/tmp/dg/skin.gputrace"

mkdir -p /tmp/dg
rm -rf "$OUT"

echo "trace : $TRACE"
echo "out   : $OUT"
echo

DYLD_LIBRARY_PATH=/opt/homebrew/lib \
MTL_CAPTURE_ENABLED=1 \
MVK_CONFIG_AUTO_GPU_CAPTURE_SCOPE=2 \
MVK_CONFIG_AUTO_GPU_CAPTURE_OUTPUT_FILE="$OUT" \
"$BIN" "$TRACE" /tmp/dg/cap_out --divergent_float_constant_gather=true 2>&1 \
  | grep -iE "capture|gputrace|GPU Trace|IssueSwap|error" | grep -v FEATURE_NOT_PRESENT

echo
if [ -e "$OUT" ]; then
  echo "OK -> $OUT  ($(du -sh "$OUT" | cut -f1))"
  echo "open it with:  open '$OUT'"
else
  echo "NO CAPTURE PRODUCED - check MoltenVK / MTL_CAPTURE_ENABLED"
fi
