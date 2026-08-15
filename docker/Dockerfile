ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG DEBIAN_FRONTEND=noninteractive

ENV COLCON_WS=/root/ros2_ws

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}

COPY android_bridge src/android_bridge

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-select android_bridge"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
