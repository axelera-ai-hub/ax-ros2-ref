#!/bin/bash
# (C) Copyright Axelera AI 2026
# All Rights Reserved
# ─────────────────────────────────────────────────────────────────────────────
# test_safe_motion.sh: Send a single safe pose to the arm controller.
#
# Usage (with MoveIt already running — either sim or real):
#   ./test_safe_motion.sh              # Move to safe test position
#   ./test_safe_motion.sh home         # Return to home/upright position
#   ./test_safe_motion.sh forward      # Reach forward (x=0.25, z=0.2)
#   ./test_safe_motion.sh left         # Reach left   (y=0.2, z=0.2)
#
# These are conservative positions well within the NED2 workspace (0.38m).
# All poses use tool-pointing-down orientation.
# ─────────────────────────────────────────────────────────────────────────────

set -e

TOPIC="/target_hover_pose"

# Tool-pointing-down quaternion (90° rotation about Y)
OX="0.0"
OY="0.707"
OZ="0.0"
OW="0.707"

case "${1:-forward}" in
  home)
    X="0.15"; Y="0.0"; Z="0.3"
    echo "Sending HOME position: (0.15, 0, 0.3) — arm upright, slightly forward"
    ;;
  forward)
    X="0.2"; Y="0.0"; Z="0.2"
    echo "Sending FORWARD position: (0.2, 0, 0.2)"
    ;;
  left)
    X="0.0"; Y="0.2"; Z="0.2"
    echo "Sending LEFT position: (0, 0.2, 0.2)"
    ;;
  right)
    X="0.0"; Y="-0.2"; Z="0.2"
    echo "Sending RIGHT position: (0, -0.2, 0.2)"
    ;;
  *)
    echo "Unknown position: $1"
    echo "Usage: $0 [home|forward|left|right]"
    exit 1
    ;;
esac

echo "Publishing to ${TOPIC}..."

ros2 topic pub --once "${TOPIC}" geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, \
    pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, \
           orientation: {x: ${OX}, y: ${OY}, z: ${OZ}, w: ${OW}}}}"

echo "Done. Check RViz / robot for motion."
