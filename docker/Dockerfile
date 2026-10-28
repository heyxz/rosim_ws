# rosim 开发容器（Ubuntu 22.04 + ROS 2 Humble + MoveIt 2.5.9 源码）
#
#   docker build -f docker/Dockerfile -t rosim:humble .
#   docker run --rm -it --net=host -v "$PWD":/workspace rosim:humble
#
# 说明：
#  * 开发机没有 docker，本文件按 docs/install.md 手写，**未实际构建验证**；
#    首次构建失败请把报错贴回来。
#  * 镜像较大（ROS desktop + MoveIt 源码编译，约 8–12 GB）。
#  * Python 依赖（.pylibs 约 5.3 GB）默认不装，需要时见文件末尾的注释。

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=humble \
    MOVEIT_WS=/opt/ws_moveit

# ---------------------------------------------------------------- ROS 2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release software-properties-common ca-certificates git \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-vcstool \
        build-essential cmake \
        libeigen3-dev libboost-all-dev libtinyxml2-dev libconsole-bridge-dev \
        libocct-foundation-dev libocct-data-exchange-dev libvtk9-dev qtbase5-dev \
        nodejs npm \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- MoveIt 2.5.9
RUN mkdir -p ${MOVEIT_WS}/src && cd ${MOVEIT_WS}/src \
    && git clone -b humble https://github.com/moveit/moveit2.git \
    && cd moveit2 && git checkout 2.5.9 && cd ${MOVEIT_WS}/src \
    && vcs import < moveit2/moveit2.repos \
    && . /opt/ros/humble/setup.sh \
    && rosdep init || true \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro humble \
    && cd ${MOVEIT_WS} \
    && colcon build --symlink-install --parallel-workers "$(nproc)" \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------- 环境
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'source ${MOVEIT_WS}/install/setup.bash' \
    >> /root/.bashrc

WORKDIR /workspace

# 可选：安装 RL 依赖（不含在默认镜像里，因为体积大且非必需）
# RUN cd /workspace && python3 -m pip install --target .pylibs -r rl_a/requirements.txt \
#     && python3 -m pip install --target .pylibs \
#        --index-url https://download.pytorch.org/whl/cu124 torch==2.6.0

CMD ["/bin/bash"]
