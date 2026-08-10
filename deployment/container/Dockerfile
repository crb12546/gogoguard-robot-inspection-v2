# syntax=docker/dockerfile:1.7
FROM gogoguard-robot-inspection:v2-dev AS mediamtx_seed
FROM gogoguard-locked-deps:20260805 AS locked_robot_deps

FROM --platform=linux/arm64 ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ninja-build patch pkg-config \
    python3-dev python3-matplotlib python3-pip python3-setuptools python3-yaml \
    python3-colcon-common-extensions python3-rosdep \
    libapr1-dev libboost-all-dev libeigen3-dev libomp-dev libpcl-dev \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-sensor-msgs-py \
    ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-rmw-cyclonedds-cpp ros-humble-navigation2 ros-humble-nav2-bringup \
    openssh-client rsync ca-certificates tini alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# Unitree's frozen ROS 2 interface package emits the vendor DDS IDL alongside
# the standard rosidl messages.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

# Keep native C++ builds within the memory envelope of both the local builder
# and the 16 GB Orin NX target. Unbounded CMake/colcon parallelism can launch
# multiple compiler trees and exhaust the host before producing an image.
ARG BUILD_JOBS=2
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} MAKEFLAGS=-j${BUILD_JOBS}

WORKDIR /opt/gogoguard/app
COPY third_party/locked_stack /opt/gogoguard/app/third_party/locked_stack

# The navigation slice retains the exact dependency revisions already built in
# the commissioned Humble environment. The prefix contains commit receipts for
# small_gicp and Unitree SDK2; the image itself is content-addressed locally.
COPY --from=locked_robot_deps /deps /opt/gogoguard/deps

# Build the exact robot capability snapshot retained from the established stack.
# These are not fresh upstream clones: the locked snapshot includes the timing,
# shutdown and base-frame changes previously made for this Go2/MID-360 runtime.
RUN cmake -S third_party/locked_stack/src/Livox-SDK2 -B /tmp/livox-sdk-build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/livox-sdk-build --parallel "${BUILD_JOBS}" \
    && cmake --install /tmp/livox-sdk-build \
    && test -f /usr/local/lib/liblivox_lidar_sdk_shared.so \
    && rm -rf /tmp/livox-sdk-build

COPY --from=locked_robot_deps /unitree_go /opt/gogoguard/ros_ws/src/unitree_go
COPY modules/navigation/ros/go2_nav2_runtime /opt/gogoguard/ros_ws/src/go2_nav2_runtime
COPY modules/device_io/ros/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp \
    /tmp/gogoguard-owned/go2_sdk2_udp_receiver.cpp
COPY modules/localization/ros/patches/go2-vgicp-orin-v2.patch \
    /tmp/gogoguard-owned/go2-vgicp-orin-v2.patch
COPY modules/device_io/ros/go2_cmd_vel_bridge/src/go2_vui_control.cpp \
    /tmp/gogoguard-owned/go2_vui_control.cpp
COPY modules/device_io/ros/go2_cmd_vel_bridge/CMakeLists.txt \
    /tmp/gogoguard-owned/go2_cmd_vel_bridge.CMakeLists.txt
RUN mkdir -p /opt/gogoguard/ros_ws/src \
    && cp -a third_party/locked_stack/src/livox_ros_driver2 /opt/gogoguard/ros_ws/src/ \
    && cp -a third_party/locked_stack/src/FAST_LIO /opt/gogoguard/ros_ws/src/fast_lio \
    && cp -a third_party/locked_stack/src/go2_map_manager /opt/gogoguard/ros_ws/src/ \
    && cp -a third_party/locked_stack/src/go2_cmd_vel_bridge /opt/gogoguard/ros_ws/src/ \
    && install -m 0644 /tmp/gogoguard-owned/go2_sdk2_udp_receiver.cpp \
      /opt/gogoguard/ros_ws/src/go2_cmd_vel_bridge/src/go2_sdk2_udp_receiver.cpp \
    && install -m 0644 /tmp/gogoguard-owned/go2_vui_control.cpp \
      /opt/gogoguard/ros_ws/src/go2_cmd_vel_bridge/src/go2_vui_control.cpp \
    && install -m 0644 /tmp/gogoguard-owned/go2_cmd_vel_bridge.CMakeLists.txt \
      /opt/gogoguard/ros_ws/src/go2_cmd_vel_bridge/CMakeLists.txt \
    && cp -a third_party/locked_stack/src/go2_nav2_interfaces /opt/gogoguard/ros_ws/src/ \
    && cp -a third_party/locked_stack/src/go2_fastlio_patrol /opt/gogoguard/ros_ws/src/ \
    && cp -a third_party/locked_stack/src/go2_site_ops /opt/gogoguard/ros_ws/src/ \
    && cp -a third_party/locked_stack/src/unitree_api /opt/gogoguard/ros_ws/src/ \
    && cd /opt/gogoguard/ros_ws/src \
    && patch --strip=1 --forward \
      --input=/tmp/gogoguard-owned/go2-vgicp-orin-v2.patch \
    && grep -q 'go2-vgicp-orin-v2' \
      go2_map_manager/src/continuous_map_localizer.cpp \
    && grep -q 'GO2_VGICP_ORIN_V2' \
      go2_site_ops/go2_site_ops/localization_quality.py \
    && cp /opt/gogoguard/ros_ws/src/livox_ros_driver2/package_ROS2.xml \
      /opt/gogoguard/ros_ws/src/livox_ros_driver2/package.xml \
    && cp -a /opt/gogoguard/ros_ws/src/livox_ros_driver2/launch_ROS2 \
      /opt/gogoguard/ros_ws/src/livox_ros_driver2/launch \
    && source /opt/ros/humble/setup.bash \
    && export CMAKE_PREFIX_PATH=/opt/gogoguard/deps:${CMAKE_PREFIX_PATH:-} \
    && cd /opt/gogoguard/ros_ws \
    && colcon build --executor sequential --merge-install \
      --packages-select \
        livox_ros_driver2 fast_lio unitree_go unitree_api go2_site_ops \
        go2_map_manager go2_cmd_vel_bridge go2_nav2_interfaces \
        go2_fastlio_patrol go2_nav2_runtime \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
    && test -x install/lib/livox_ros_driver2/livox_ros_driver2_node \
    && test -x install/lib/fast_lio/fastlio_mapping \
    && test -x install/lib/go2_map_manager/continuous_map_localizer \
    && test -x install/lib/go2_cmd_vel_bridge/go2_sdk2_udp_receiver \
    && test -x install/lib/go2_cmd_vel_bridge/go2_vui_control \
    && test -x install/lib/go2_nav2_runtime/patrol_runtime_manager \
    && rm -rf build log /tmp/gogoguard-owned

# H.264 incident video is remuxed without decode or encode. Keep this runtime
# tool after the expensive ROS/C++ build so diagnostics changes do not
# invalidate the Livox, FAST-LIO, Nav2 and Unitree compilation cache.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Pinned ARM64 MediaMTX release. The checksum is the upstream v1.20.0 release
# checksum; the gateway only remuxes the commissioned Z1Pro H.264 RTSP stream.
COPY --from=mediamtx_seed /usr/local/bin/mediamtx /tmp/mediamtx
COPY --from=mediamtx_seed /usr/share/licenses/mediamtx/LICENSE /tmp/mediamtx.LICENSE
RUN test "$(sha256sum /tmp/mediamtx | cut -d' ' -f1)" = \
      "2da379972ba86627632aa7e3f779c680ba04a5ee26ef2a20dc61cefcc24f73b8" \
    && install -d -m 0755 /usr/share/licenses/mediamtx \
    && install -m 0755 /tmp/mediamtx /usr/local/bin/mediamtx \
    && install -m 0644 /tmp/mediamtx.LICENSE /usr/share/licenses/mediamtx/LICENSE \
    && rm /tmp/mediamtx /tmp/mediamtx.LICENSE

# Livox SDK2 installs its shared library under /usr/local/lib. The target's
# runtime linker cache is host-specific, so make that application-owned path
# explicit inside the container rather than relying on a build-host cache.
ENV CMAKE_PREFIX_PATH=/opt/gogoguard/deps \
    LD_LIBRARY_PATH=/opt/gogoguard/deps/lib:/usr/local/lib

COPY pyproject.toml setup.py /opt/gogoguard/app/
COPY modules /opt/gogoguard/app/modules
COPY apps /opt/gogoguard/app/apps
COPY services /opt/gogoguard/app/services
COPY config /opt/gogoguard/app/config
COPY deployment/container/edge-entrypoint /tmp/gogoguard-edge-entrypoint
COPY deployment/container/interaction-requirements.txt /tmp/interaction-requirements.txt

RUN python3 -m pip install --no-cache-dir -r /tmp/interaction-requirements.txt \
    && python3 -m pip install --no-cache-dir --no-deps --no-build-isolation . \
    && install -m 0755 /tmp/gogoguard-edge-entrypoint /usr/local/bin/gogoguard-edge-entrypoint \
    && install -D -m 0755 \
      third_party/locked_stack/scripts/patrol_performance_monitor.py \
      /home/unitree/go2_fastlio_ws/scripts/patrol_performance_monitor.py \
    && rm /tmp/gogoguard-edge-entrypoint /tmp/interaction-requirements.txt

VOLUME ["/var/lib/gogoguard"]
EXPOSE 8080 8889/tcp 8189/tcp 8189/udp
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/gogoguard-edge-entrypoint"]
