#!/bin/bash
# AxROS container entrypoint.
# Loads the Axelera environment, sources ROS2 and the workspace,
# validates hardware and model, then launches the inference pipeline.
set -e

# ── Load Axelera environment ─────────────────────────────────────────────────
if [ -f /etc/axros.env ]; then
  set -a
  # shellcheck source=/dev/null
  source /etc/axros.env
  set +a
fi

# ── Source ROS2 ──────────────────────────────────────────────────────────────
# shellcheck source=/opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.bash

# ── Source workspace overlay ─────────────────────────────────────────────────
if [ -f /ros2_ws/install/setup.bash ]; then
  # shellcheck source=/ros2_ws/install/setup.bash
  source /ros2_ws/install/setup.bash
fi

# ── Validate Metis device ────────────────────────────────────────────────────
METIS_DEVICES=$(ls /dev/metis-* 2>/dev/null || true)
if [ -z "$METIS_DEVICES" ]; then
  echo "WARNING: No Metis device found at /dev/metis-*"
  echo "   Ensure you passed --device=/dev/metis-0:1:0 to docker run"
  echo "   or set the devices: section in docker-compose.yml"
fi

# ── Validate model directory ─────────────────────────────────────────────────
MODEL_NAME="${MODEL_NAME:-yolov5s-v7-coco}"
MODEL_DIR="/models/${MODEL_NAME}/${MODEL_NAME}/4"

if [ ! -f "$MODEL_DIR/model.json" ]; then
  echo "ERROR: model.json not found at $MODEL_DIR"
  echo ""
  echo "Mount your voyager-sdk/build/ directory:"
  echo "  -v ~/voyager-sdk/build:/models:ro"
  echo ""
  echo "Available models in /models/:"
  ls /models/ 2>/dev/null || echo "  (none — /models is empty or not mounted)"
  exit 1
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " AxROS — Axelera AI x ROS2 Inference"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " Model:   $MODEL_NAME"
echo " Device:  ${METIS_DEVICES:-none detected}"
echo " Results: /results/"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ── Launch ───────────────────────────────────────────────────────────────────
# If arguments are passed, exec them directly (allows: docker run axros bash)
if [ "$#" -gt 0 ]; then
  exec "$@"
fi

# Default: launch the full inference pipeline
exec ros2 launch metis_ros2_inference metis_inference.launch.py \
  model_name:="$MODEL_NAME" \
  model_dir:="$MODEL_DIR"
