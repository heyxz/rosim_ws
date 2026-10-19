#!/usr/bin/env bash
# ROS 2 仿真服务端启动脚本（systemd 用户服务使用）
set -e
source /opt/ros/humble/setup.bash
source /home/xz/ws_moveit/install/setup.bash
source /home/xz/rosim_ws/install/setup.bash
export HOME=/home/xz/rosim_ws/.scratch/home
export ROS_HOME="$HOME/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch rosim_server server.launch.py
