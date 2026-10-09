FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rtabmap-odom \
        ros-jazzy-rtabmap-slam \
        ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /slam_ws
COPY docker/slam/isaac_slam /slam_ws/src/isaac_slam

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["bash"]
