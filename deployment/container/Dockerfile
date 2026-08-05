# syntax=docker/dockerfile:1.7
FROM --platform=linux/arm64 ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ninja-build pkg-config \
    python3-dev python3-matplotlib python3-pip python3-setuptools python3-yaml \
    python3-colcon-common-extensions python3-rosdep \
    libapr1-dev libboost-all-dev libeigen3-dev libomp-dev libpcl-dev \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-sensor-msgs-py \
    ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-rmw-cyclonedds-cpp ros-humble-navigation2 ros-humble-nav2-bringup \
    openssh-client rsync ca-certificates tini \
    && rm -rf /var/lib/apt/lists/*

# Keep native C++ builds within the memory envelope of both the local builder
# and the 16 GB Orin NX target. Unbounded CMake/colcon parallelism can launch
# multiple compiler trees and exhaust the host before producing an image.
ARG BUILD_JOBS=2
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} MAKEFLAGS=-j${BUILD_JOBS}

WORKDIR /opt/gogoguard/app
COPY third_party/locked_stack /opt/gogoguard/app/third_party/locked_stack

# Build the exact robot capability snapshot retained from the established stack.
# These are not fresh upstream clones: the locked snapshot includes the timing,
# shutdown and base-frame changes previously made for this Go2/MID-360 runtime.
RUN cmake -S third_party/locked_stack/src/Livox-SDK2 -B /tmp/livox-sdk-build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/livox-sdk-build --parallel "${BUILD_JOBS}" \
    && cmake --install /tmp/livox-sdk-build \
    && test -f /usr/local/lib/liblivox_lidar_sdk_shared.so \
    && rm -rf /tmp/livox-sdk-build

RUN mkdir -p /opt/gogoguard/ros_ws/src \
    && cp -a third_party/locked_stack/src/livox_ros_driver2 /opt/gogoguard/ros_ws/src/ \
    && cp -a third_party/locked_stack/src/FAST_LIO /opt/gogoguard/ros_ws/src/fast_lio \
    && cp /opt/gogoguard/ros_ws/src/livox_ros_driver2/package_ROS2.xml \
      /opt/gogoguard/ros_ws/src/livox_ros_driver2/package.xml \
    && cp -a /opt/gogoguard/ros_ws/src/livox_ros_driver2/launch_ROS2 \
      /opt/gogoguard/ros_ws/src/livox_ros_driver2/launch \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/gogoguard/ros_ws \
    && colcon build --executor sequential --merge-install \
      --packages-select livox_ros_driver2 fast_lio \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
    && test -x install/lib/livox_ros_driver2/livox_ros_driver2_node \
    && test -x install/lib/fast_lio/fastlio_mapping \
    && rm -rf build log

# Livox SDK2 installs its shared library under /usr/local/lib. The target's
# runtime linker cache is host-specific, so make that application-owned path
# explicit inside the container rather than relying on a build-host cache.
ENV LD_LIBRARY_PATH=/usr/local/lib

COPY pyproject.toml setup.py /opt/gogoguard/app/
COPY modules /opt/gogoguard/app/modules
COPY apps /opt/gogoguard/app/apps
COPY services /opt/gogoguard/app/services
COPY config /opt/gogoguard/app/config
COPY deployment/container/edge-entrypoint /tmp/gogoguard-edge-entrypoint

RUN python3 -m pip install --no-cache-dir --no-deps --no-build-isolation . \
    && install -m 0755 /tmp/gogoguard-edge-entrypoint /usr/local/bin/gogoguard-edge-entrypoint \
    && rm /tmp/gogoguard-edge-entrypoint

VOLUME ["/var/lib/gogoguard"]
EXPOSE 8080
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/gogoguard-edge-entrypoint"]
