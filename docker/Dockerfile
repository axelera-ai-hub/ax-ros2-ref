# ── Stage 1: builder ─────────────────────────────────────────────────────────
# Builds the ros2_ws. Colcon build artifacts stay in this stage only.
FROM ros:jazzy-ros-base-noble AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    libopencv-dev \
    pkg-config \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-message-filters \
    ros-jazzy-rclcpp-components \
  && rm -rf /var/lib/apt/lists/*

# Copy Axelera SDK from staging (created by build_image.sh)
COPY docker/staging/axelera/ /opt/axelera/

# Copy ONNX Runtime from staging
COPY docker/staging/onnxruntime/ /opt/onnxruntime/

# Set pkg-config path so CMake finds axruntime.pc at build time
RUN RUNTIME_DIR=$(ls -d /opt/axelera/runtime-* 2>/dev/null | head -1) && \
    echo "Found AxRuntime at: $RUNTIME_DIR"

ENV PKG_CONFIG_PATH=/opt/axelera/runtime-1.5.3-1/lib/pkgconfig

# Copy ros2_ws source
COPY ros2_ws/src/ /ros2_ws/src/

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && \
    PKG_CONFIG_PATH=/opt/axelera/runtime-1.5.3-1/lib/pkgconfig:$PKG_CONFIG_PATH \
    colcon build \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DONNXRUNTIME_ROOT=/opt/onnxruntime \
      --install-base /ros2_ws/install

# ── Stage 2: runtime ──────────────────────────────────────────────────────────
# Lean runtime image — only install/, not build/ or log/
FROM ros:jazzy-ros-base-noble AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# Runtime apt dependencies (no build tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-message-filters \
    ros-jazzy-rclcpp-components \
  && rm -rf /var/lib/apt/lists/*

# Copy Axelera SDK (needed at runtime for firmware loading)
COPY docker/staging/axelera/ /opt/axelera/

# Copy ONNX Runtime
COPY docker/staging/onnxruntime/ /opt/onnxruntime/

# Copy only the install directory from builder
COPY --from=builder /ros2_ws/install/ /ros2_ws/install/

# Write environment variables to /etc/axros.env — sourced by entrypoint.sh.
# Uses glob expansion to handle version strings without hardcoding them.
RUN RUNTIME_DIR=$(ls -d /opt/axelera/runtime-* 2>/dev/null | head -1) && \
    DEVICE_DIR=$(ls -d /opt/axelera/device-* 2>/dev/null | head -1) && \
    RISCV_DIR=$(ls -d /opt/axelera/riscv-gnu-newlib-* 2>/dev/null | head -1) && \
    printf '%s\n' \
      "AXELERA_RUNTIME_DIR=${RUNTIME_DIR}" \
      "AXELERA_DEVICE_DIR=${DEVICE_DIR}/omega" \
      "AXELERA_RISCV_TOOLCHAIN_DIR=${RISCV_DIR}" \
      "AIPU_FIRMWARE_OMEGA=${DEVICE_DIR}/omega/bin/start_axelera_runtime.elf" \
      "AIPU_RUNTIME_STAGE0_OMEGA=${DEVICE_DIR}/omega/bin/start_axelera_runtime_stage0.bin" \
      "LD_LIBRARY_PATH=${RUNTIME_DIR}/lib:/opt/onnxruntime/lib" \
      "PATH=${RUNTIME_DIR}/bin:${RISCV_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    > /etc/axros.env

# Create volume mount points
RUN mkdir -p /models /results

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
