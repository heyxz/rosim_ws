cmake_minimum_required(VERSION 3.8)
project(rosim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/EdgeInfo.msg"
  "msg/ModelInfo.msg"
  "msg/SeamPart.msg"
  "msg/SeamSpec.msg"
  "srv/ImportModel.srv"
  "srv/MoveWorkpiece.srv"
  "srv/SetSpeedScale.srv"
  "srv/SetPlanner.srv"
  "srv/RlPlan.srv"
  "srv/PlanJoint.srv"
  "srv/MoveToPose.srv"
  "srv/GoHome.srv"
  "srv/JogJoint.srv"
  "srv/CreateSeam.srv"
  "srv/JogCartesian.srv"
  "action/FollowEdge.action"
  "action/FollowSeams.action"
  DEPENDENCIES geometry_msgs trajectory_msgs std_msgs action_msgs
)

ament_package()
