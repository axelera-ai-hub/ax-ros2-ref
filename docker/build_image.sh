#!/bin/bash
# Builds the AxROS Docker image.
# Stages /opt/axelera/ and the ONNX Runtime into docker/staging/,
# runs docker build, then cleans up staging.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STAGING="$SCRIPT_DIR/staging"
IMAGE_NAME="axros:jazzy-latest"

# ── Validate prerequisites ───────────────────────────────────────────────────
if [ ! -d "/opt/axelera" ]; then
  echo "ERROR: /opt/axelera not found."
  echo "Install the Voyager SDK first: https://github.com/axelera-ai-hub/voyager-sdk"
  exit 1
fi

ONNX_SRC="$HOME/voyager-sdk/operators/onnxruntime/onnxruntime-linux-x64-1.17.1"
if [ ! -d "$ONNX_SRC" ]; then
  echo "ERROR: ONNX Runtime not found at $ONNX_SRC"
  echo "Ensure voyager-sdk is cloned at ~/voyager-sdk"
  exit 1
fi

if ! ls /dev/metis-* &>/dev/null; then
  echo "WARNING: No Metis device detected at /dev/metis-*"
  echo "The image will build but inference will fail at runtime without hardware."
fi

# ── Stage SDK files ──────────────────────────────────────────────────────────
echo "Staging Axelera SDK from /opt/axelera/ (~1.1GB, please wait)..."
mkdir -p "$STAGING/axelera"
rsync -a --info=progress2 /opt/axelera/ "$STAGING/axelera/"

echo "Staging ONNX Runtime (~22MB)..."
mkdir -p "$STAGING/onnxruntime"
rsync -a "$ONNX_SRC/" "$STAGING/onnxruntime/"

# ── Build image ──────────────────────────────────────────────────────────────
echo ""
echo "Building Docker image: $IMAGE_NAME"
echo "Build context: $REPO_ROOT"
echo ""

docker build \
  --tag "$IMAGE_NAME" \
  --file "$SCRIPT_DIR/Dockerfile" \
  "$REPO_ROOT"

# ── Cleanup ──────────────────────────────────────────────────────────────────
echo "Cleaning up staging directory..."
rm -rf "$STAGING"

echo ""
echo "Build complete: $IMAGE_NAME"
echo ""
echo "Run the full pipeline:"
echo "  docker compose -f $SCRIPT_DIR/docker-compose.yml up"
echo ""
echo "Or run inference only (bring your own image source):"
echo "  docker run --rm \\"
echo "    --device=\$(ls /dev/metis-* | head -1) \\"
echo "    -v ~/voyager-sdk/build:/models:ro \\"
echo "    -v ~/ax-ros2-ref/results:/results \\"
echo "    $IMAGE_NAME"
