ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

# 필요한 시스템 패키지 설치 (cv_bridge 포함, X11 앱도 설치)
RUN apt-get update && \
    apt-get -y --quiet --no-install-recommends install \
        gcc \
        git \
        python3 \
        python3-pip \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-cv-bridge \
        x11-apps && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install -r src/requirements.txt

RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
