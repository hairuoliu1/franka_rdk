FROM ros:humble-ros-base 

ENV DEBIAN_FRONTEND=noninteractive \
  ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml

# 换源配置（清华大学镜像）
# 1) Ubuntu APT 源
RUN sed -i 's|archive.ubuntu.com|mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
  sed -i 's|security.ubuntu.com|mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list && \
  sed -i 's|ports.ubuntu.com|mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# 2) ROS 2 APT 源（仅替换域名，保留路径）
RUN grep -rl "packages.ros.org" /etc/apt/sources.list.d/ 2>/dev/null | \
  xargs -r sed -i 's|packages.ros.org|mirrors.tuna.tsinghua.edu.cn|g'

# 3) rosdep 索引源
RUN if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    sed -i 's|raw.githubusercontent.com/ros/rosdistro/master|mirrors.tuna.tsinghua.edu.cn/rosdistro|g' \
    /etc/ros/rosdep/sources.list.d/20-default.list; \
  fi

# 4) pip 源
RUN mkdir -p /root/.pip && \
  printf '[global]\nindex-url = https://pypi.tuna.tsinghua.edu.cn/simple\n' > /root/.pip/pip.conf

RUN apt-get update && apt-get install -y \
  # Install libfranka build depencencies
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libfmt-dev \
  libpoco-dev \
  ros-humble-pinocchio \
  # Install ros packages
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-pip \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Install robotiq gripper
ARG ROS2_ROBOTIQ_GRIPPER_COMMIT_HASH="2ff85455d4b9f973c4b0bab1ce95fb09367f0d26"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  mkdir tmp_build && \
  cd tmp_build && \
  git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
  cd ros2_robotiq_gripper && git checkout ${ROS2_ROBOTIQ_GRIPPER_COMMIT_HASH} && cd .. && \
  sed -i 's/kGripperMaxSpeed = 0.150;/kGripperMaxSpeed = 1.0;/g' ros2_robotiq_gripper/robotiq_driver/src/hardware_interface.cpp && \
  vcs import . --input ros2_robotiq_gripper/ros2_robotiq_gripper.humble.repos && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  colcon build --install-base /opt/ros/humble/franka --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  cd .. && \
  rm -rf tmp_build && \
  rm -rf /var/lib/apt/lists/*"

# Install rqt reconfigure from source (for array parameters)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  mkdir -p /tmp/rqt_reconfigure && cd /tmp && \
  git clone https://github.com/ros-visualization/rqt_reconfigure.git --branch humble && \
  cd rqt_reconfigure && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  colcon build --install-base /opt/ros/humble/franka --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  cd .. && \
  rm -rf /tmp/rqt_reconfigure && \
  rm -rf /var/lib/apt/lists/*"

# Install libfranka, franka_ros2 and franka_description
# Note: The versions need to be set according to your robot's system version
# https://frankarobotics.github.io/docs/compatibility.html
ARG LIBFRANKA_VERSION=0.18.2
ARG FRANKA_ROS2_VERSION=v2.1.0
ARG FRANKA_DESCRIPTION_VERSION=1.3.0

# 再构建 ROS2 包
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
  mkdir -p /tmp/franka_ros2 && cd /tmp/franka_ros2 && \
  git clone --recursive https://github.com/frankarobotics/franka_ros2.git --branch ${FRANKA_ROS2_VERSION} && \
  git clone --recursive https://github.com/frankarobotics/franka_description.git --branch ${FRANKA_DESCRIPTION_VERSION} && \
  apt-get update && rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/* && \
  colcon build --install-base /opt/ros/humble/franka \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off \
    --parallel-workers 1 && \
  cd .. && rm -rf /tmp/franka_ros2'

# 先单独构建 libfranka（最耗资源）
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
  mkdir -p /tmp/libfranka && cd /tmp/libfranka && \
  git clone --recursive https://github.com/frankarobotics/libfranka --branch ${LIBFRANKA_VERSION} && \
  cd libfranka && mkdir build && cd build && \
  cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
  make -j2 && make install && \
  cd / && rm -rf /tmp/libfranka'