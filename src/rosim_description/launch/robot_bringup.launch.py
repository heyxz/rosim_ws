"""Full system bringup (no move_group needed — rosim_planning embeds OMPL):

  1. robot_state_publisher        (TF from /joint_states)
  2. ros2_control_node            (controller_manager, mock hardware)
  3. spawner joint_state_broadcaster + arm_controller (JTC)
  4. rosim_planning              (embedded OMPL, FollowEdge action server)
  5. rosim_viewer                (Qt+VTK GUI)
  6. optional rviz2              (use_rviz:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
        "robot_description_semantic": load_file(
            os.path.join(desc_share, "config", "robot.srdf"))
    }
    kinematics = os.path.join(desc_share, "config", "kinematics.yaml")
    joint_limits = os.path.join(desc_share, "config", "joint_limits.yaml")
    ompl_planning = os.path.join(desc_share, "config", "ompl_planning.yaml")
    controllers_yaml = os.path.join(desc_share, "config", "ros2_controllers.yaml")

    use_rviz = LaunchConfiguration("use_rviz")
    use_viewer = LaunchConfiguration("use_viewer")

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

    # rosim_planning: ALL FIVE parameter sources must land on this node —
    # missing kinematics.yaml => KDL plugin not found; missing SRDF => RobotModelLoader fails.
    planning = Node(
        package="rosim_planning",
        executable="rosim_planning_node",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            ompl_planning,
            # planner extensions dlopen'ed at init; their static factory
            # registrations then run in-process (rosim_rl registers "rl")
            {"planner_libraries": ["librosim_rl.so"]},
        ],
        output="screen",
    )

    viewer = Node(
        package="rosim_viewer",
        executable="rosim_viewer",
        # SRDF too: without it RDFLoader blocks 10 s waiting for a
        # robot_description_semantic topic and logs an SRDF parse error
        parameters=[robot_description, robot_description_semantic],
        output="screen",
        condition=IfCondition(use_viewer),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(desc_share, "config", "display.rviz")],
    )

    # GUI owns the session: when the viewer window closes (or the viewer
    # crashes), tear down every other node in this launch.
    viewer_exit_shuts_down = RegisterEventHandler(
        OnProcessExit(
            target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason="rosim_viewer exited"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_viewer", default_value="true"),
        rsp,
        control_node,
        jsb_spawner,
        arm_spawner,
        planning,
        viewer,
        rviz,
        viewer_exit_shuts_down,
    ])
