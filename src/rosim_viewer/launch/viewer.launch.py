"""Standalone viewer launch (expects rosim_planning + controllers running)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("rosim_description")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", os.path.join(desc_share, "urdf", "robot.urdf.xacro"),
        " ", "no_ros2_control:=true",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    viewer = Node(
        package="rosim_viewer",
        executable="rosim_viewer",
        parameters=[robot_description],
        output="screen",
    )

    return LaunchDescription([viewer])
