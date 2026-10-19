"""Debug-mode bringup: robot + planning + the WebSocket server.

Replaces the Qt viewer (rosim_viewer) with rosim_server. The server node is a
ROS client of rosim_planning and renders the scene headlessly; the browser only
receives JPEG frames and JSON control messages.

  1. robot_state_publisher      (TF from /joint_states)
  2. ros2_control_node          (controller_manager, mock hardware)
  3. spawner joint_state_broadcaster + arm_controller (JTC)
  4. rosim_planning             (embedded OMPL, FollowEdge action server)
  5. rosim_server               (headless render + WebSocket on :8765)

Open http://<host>:8765/ in a browser (debug mode).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_file(path):
    with open(path, "r") as f:
        return f.read()


def generate_launch_description():
    desc_share = get_package_share_directory("rosim_description")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", os.path.join(desc_share, "urdf", "robot.urdf.xacro"),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(os.path.join(desc_share, "config", "robot.srdf"))
    }
    kinematics = os.path.join(desc_share, "config", "kinematics.yaml")
    joint_limits = os.path.join(desc_share, "config", "joint_limits.yaml")
    ompl_planning = os.path.join(desc_share, "config", "ompl_planning.yaml")
    controllers_yaml = os.path.join(desc_share, "config", "ros2_controllers.yaml")

    use_rviz = LaunchConfiguration("use_rviz")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    planning = Node(
        package="rosim_planning",
        executable="rosim_planning_node",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            ompl_planning,
            {"planner_libraries": ["librosim_rl.so"]},
        ],
        output="screen",
    )

    # Server: display-only robot model (no kinematics solvers), so it only needs
    # the URDF + SRDF. web_root resolves to the installed share dir by default.
    server = Node(
        package="rosim_server",
        executable="rosim_server_node",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"ws_address": "0.0.0.0"},
            {"ws_port": LaunchConfiguration("ws_port")},
            {"fps": 12},
            {"width": 1280},
            {"height": 720},
            {"quality": 70},
            # Where the in-page model picker starts browsing.
            {"model_dir": os.path.join(desc_share, "meshes")},
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(desc_share, "config", "display.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        # Configurable so a second instance (tests, side-by-side runs) does not
        # collide with the service on 8765.
        DeclareLaunchArgument("ws_port", default_value="8765"),
        rsp,
        control_node,
        jsb_spawner,
        arm_spawner,
        planning,
        server,
        rviz,
    ])
