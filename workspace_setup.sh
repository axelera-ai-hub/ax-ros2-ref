#!/bin/bash

# Add at the very top of workspace_setup.sh, before anything else:
# Correct HOME if it points to a non-existent directory
if [ ! -d "$HOME" ]; then
  export HOME="$(getent passwd $USER | cut -d: -f6)"
  echo "Corrected HOME to: $HOME"
fi

# Resolve the repo root relative to this script's actual location
# This works regardless of what $HOME is set to on the machine
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"
ROS2_WS="${REPO_ROOT}/ros2_ws"

source /opt/ros/jazzy/setup.bash

# Install missing ROS2 apt dependencies if not present
ROS_PKGS=(
  ros-jazzy-vision-msgs
  ros-jazzy-cv-bridge
  ros-jazzy-image-transport
  ros-jazzy-message-filters
  ros-jazzy-rclcpp-components
  ros-jazzy-foxglove-bridge
  ros-jazzy-moveit
  ros-jazzy-moveit-ros-planning-interface
  ros-jazzy-moveit-core
  python3-colcon-common-extensions
)
MISSING=()
for pkg in "${ROS_PKGS[@]}"; do
  if ! dpkg -s "$pkg" &>/dev/null; then
    MISSING+=("$pkg")
  fi
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo "Installing missing ROS2 packages: ${MISSING[*]}"
  sudo apt-get update -qq
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    "${MISSING[@]}"
fi

# Install roslibpy (required by ned-ros2-driver)
if ! pip3 show roslibpy &>/dev/null; then
  echo "Installing roslibpy..."
  pip3 install --break-system-packages "roslibpy<2.0.0"
fi

# ─── 3. Axelera environment ────────────────────────────────────────────────
# Pinned to a specific Voyager SDK version for ABI stability across
# host machines and architectures. Override by exporting AXELERA_SDK_VERSION
# before sourcing this script (e.g. AXELERA_SDK_VERSION=1.6.0-1).
AXELERA_SDK_VERSION="${AXELERA_SDK_VERSION:-1.5.3-1}"

if [ -d "/opt/axelera/runtime-${AXELERA_SDK_VERSION}" ]; then
  AXELERA_RUNTIME_DIR="/opt/axelera/runtime-${AXELERA_SDK_VERSION}"
  AXELERA_DEVICE_DIR="/opt/axelera/device-${AXELERA_SDK_VERSION}/omega"
else
  echo "  WARNING: pinned Voyager SDK ${AXELERA_SDK_VERSION} not found at /opt/axelera/runtime-${AXELERA_SDK_VERSION}"
  echo "   Falling back to latest installed runtime."
  AXELERA_RUNTIME_DIR="$(ls -d /opt/axelera/runtime-* 2>/dev/null | sort -V | tail -1)"
  AXELERA_DEVICE_DIR="$(ls -d /opt/axelera/device-*/omega 2>/dev/null | sort -V | tail -1)"
fi
# RISC-V toolchain is independent of the runtime ABI; keep it dynamic.
AXELERA_RISCV_TOOLCHAIN_DIR="$(ls -d /opt/axelera/riscv-gnu-newlib-* 2>/dev/null | sort -V | tail -1)"

if [ -z "$AXELERA_RUNTIME_DIR" ]; then
  echo "  WARNING: /opt/axelera/runtime-* not found."
  echo "   Install the Voyager SDK: https://github.com/axelera-ai-hub/voyager-sdk"
else
  export AXELERA_RUNTIME_DIR
  export AXELERA_DEVICE_DIR
  export AXELERA_RISCV_TOOLCHAIN_DIR
  export AIPU_FIRMWARE_OMEGA="${AXELERA_DEVICE_DIR}/bin/start_axelera_runtime.elf"
  export AIPU_RUNTIME_STAGE0_OMEGA="${AXELERA_DEVICE_DIR}/bin/start_axelera_runtime_stage0.bin"
  export PATH="${AXELERA_RUNTIME_DIR}/bin:${AXELERA_RISCV_TOOLCHAIN_DIR}/bin:${PATH}"
  export PKG_CONFIG_PATH="${AXELERA_RUNTIME_DIR}/lib/pkgconfig:${PKG_CONFIG_PATH}"
fi

# ─── 3b. Voyager SDK root (consumed by ROS2 launch files) ──────────────────
# Launch files default to $VOYAGER_SDK_DIR (and fall back to ~/voyager-sdk)
# so the same launches work on any machine once this env is sourced.
VOYAGER_SDK_DIR="$(cd "${SCRIPT_DIR}/../voyager-sdk" 2>/dev/null && pwd)"
if [ -n "${VOYAGER_SDK_DIR}" ]; then
  export VOYAGER_SDK_DIR
fi

# ─── 4. ONNX Runtime — search voyager-sdk relative to repo root ────────────
# Voyager ships ONNX Runtime per architecture:
#   x86_64  → onnxruntime-linux-x64-<ver>
#   aarch64 → onnxruntime-linux-aarch64-<ver>
# Override by exporting ONNX_ARCH_TAG before sourcing (e.g. for cross-builds).
case "$(uname -m)" in
  x86_64)  _detected_arch_tag="x64" ;;
  aarch64|arm64) _detected_arch_tag="aarch64" ;;
  *)
    echo "  WARNING: unrecognized host architecture '$(uname -m)'; defaulting ONNX_ARCH_TAG=x64"
    _detected_arch_tag="x64" ;;
esac
ONNX_ARCH_TAG="${ONNX_ARCH_TAG:-${_detected_arch_tag}}"
unset _detected_arch_tag

ONNX_DIR="$(ls -d "${SCRIPT_DIR}/../voyager-sdk/operators/onnxruntime/onnxruntime-linux-${ONNX_ARCH_TAG}-"* 2>/dev/null | head -1)"
if [ -n "$ONNX_DIR" ] && [ -d "$ONNX_DIR/lib" ]; then
  export ONNX_DIR
  export LD_LIBRARY_PATH="${AXELERA_RUNTIME_DIR}/lib:${ONNX_DIR}/lib:${LD_LIBRARY_PATH}"
else
  # fallback: search common locations
  ONNX_DIR="$(find /home /localhome -name "libonnxruntime.so" 2>/dev/null | head -1 | xargs dirname 2>/dev/null)"
  if [ -n "$ONNX_DIR" ]; then
    export ONNX_DIR
    export LD_LIBRARY_PATH="${AXELERA_RUNTIME_DIR}/lib:${ONNX_DIR}:${LD_LIBRARY_PATH}"
  else
    echo "  WARNING: ONNX Runtime not found for arch '${ONNX_ARCH_TAG}'."
    echo "   Expected: ${SCRIPT_DIR}/../voyager-sdk/operators/onnxruntime/onnxruntime-linux-${ONNX_ARCH_TAG}-*"
  fi
fi

# ─── 5. Workspace install overlay ──────────────────────────────────────────
cd "${ROS2_WS}"
source install/setup.bash 2>/dev/null || true

# ─── 6. Voyager SDK venv (Python nodes only — skip for C++ builds) ─────────
VOYAGER_VENV="$(ls -d "${SCRIPT_DIR}/../voyager-sdk/venv" 2>/dev/null | head -1)"
if [[ "${AXROS_ENABLE_VENV}" == "1" ]]; then
  if [ -f "${VOYAGER_VENV}/bin/activate" ]; then
    cd "${SCRIPT_DIR}/../voyager-sdk" && source venv/bin/activate
    export PYTHONPATH="${VIRTUAL_ENV}/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    echo "AxROS workspace ready (Jazzy + Voyager SDK venv)"
  else
    echo "Voyager SDK venv not found at ${VOYAGER_VENV}"
  fi
else
  echo "AxROS workspace ready (Jazzy, no venv — for venv: export AXROS_ENABLE_VENV=1 && source workspace_setup.sh)"
fi
