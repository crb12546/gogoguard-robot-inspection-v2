# syntax=docker/dockerfile:1.7
FROM --platform=linux/arm64 ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-setuptools python3-colcon-common-extensions \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-sensor-msgs-py \
    ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-pcl-ros \
    openssh-client rsync ca-certificates tini \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/gogoguard/app
COPY . /opt/gogoguard/app
RUN python3 -m pip install --no-cache-dir --no-deps .

VOLUME ["/var/lib/gogoguard"]
EXPOSE 8080
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["gogoguard-site-console", "--mode", "robot", "--map-worker", "ssh", "--host", "0.0.0.0", "--port", "8080", "--data-root", "/var/lib/gogoguard", "--config", "/etc/gogoguard/runtime.json", "--static-root", "/opt/gogoguard/app/apps/site_console/frontend"]
