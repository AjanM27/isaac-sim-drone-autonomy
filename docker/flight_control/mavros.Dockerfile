FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-mavros ros-jazzy-mavros-extras geographiclib-tools \
    && /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*
