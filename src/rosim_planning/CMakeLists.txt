cmake_minimum_required(VERSION 3.8)
project(rosim_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosim_interfaces REQUIRED)
find_package(rosim_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
# source-overlay MoveIt2: CollisionEnvFCL lives inside the moveit_core prefix
find_library(MOVEIT_COLDET_FCL_LIB moveit_collision_detection_fcl REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

# planning core library (planner implementations + edge following)
add_library(rosim_planning_core SHARED
  src/motion_planner.cpp
  src/fallback_planner.cpp
  src/line_planner.cpp
  src/ompl_planner.cpp
  src/edge_follower.cpp
  src/trajectory_timing.cpp
)
set_target_properties(rosim_planning_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rosim_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rosim_planning_core
  rclcpp rclcpp_action rosim_interfaces rosim_core
  moveit_core moveit_ros_planning moveit_msgs geometric_shapes pluginlib
  tf2_eigen trajectory_msgs sensor_msgs Eigen3 Boost)
target_link_libraries(rosim_planning_core ${CMAKE_DL_LIBS} Threads::Threads)

# node executable
add_executable(rosim_planning_node src/planning_node.cpp src/main.cpp)
target_include_directories(rosim_planning_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rosim_planning_node rosim_planning_core ${CMAKE_DL_LIBS})
ament_target_dependencies(rosim_planning_node
  rclcpp rclcpp_action rosim_interfaces rosim_core
  moveit_core moveit_ros_planning moveit_msgs geometric_shapes pluginlib
  tf2_eigen trajectory_msgs sensor_msgs Eigen3 Boost)

install(TARGETS rosim_planning_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
# dev tool: measure candidate home poses (self-collision + TCP placement)
add_executable(home_probe tools/home_probe.cpp)
target_link_libraries(home_probe ${CMAKE_DL_LIBS})
ament_target_dependencies(home_probe
  rclcpp moveit_core moveit_ros_planning moveit_msgs Eigen3 Boost)

# dev tool: probe one TCP pose with many random-seed IK, list all branches
add_executable(ik_probe tools/ik_probe.cpp)
target_link_libraries(ik_probe ${CMAKE_DL_LIBS})
ament_target_dependencies(ik_probe
  rclcpp rosim_core moveit_core moveit_ros_planning moveit_msgs geometric_shapes
  Eigen3 Boost)

# dev tool: IK reachability scan of every workpiece edge
add_executable(edge_scan tools/edge_scan.cpp)
target_link_libraries(edge_scan ${CMAKE_DL_LIBS} rosim_planning_core)
ament_target_dependencies(edge_scan
  rclcpp rclcpp_action rosim_interfaces rosim_core
  moveit_core moveit_ros_planning moveit_msgs geometric_shapes
  tf2_eigen Eigen3 Boost)

# dev tool: per-waypoint OK/COLLIDE/NO-IK diagnosis of one edge
add_executable(traverse_probe tools/traverse_probe.cpp)
target_link_libraries(traverse_probe ${CMAKE_DL_LIBS} rosim_planning_core)
ament_target_dependencies(traverse_probe
  rclcpp rclcpp_action rosim_interfaces rosim_core
  moveit_core moveit_ros_planning moveit_msgs geometric_shapes
  tf2_eigen Eigen3 Boost)

# dev tool: measure the self-collision pair table for SRDF generation
add_executable(acm_probe tools/acm_probe.cpp)
target_link_libraries(acm_probe ${CMAKE_DL_LIBS} ${MOVEIT_COLDET_FCL_LIB})
ament_target_dependencies(acm_probe
  rclcpp moveit_core moveit_ros_planning moveit_msgs Eigen3 Boost)

install(TARGETS rosim_planning_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS home_probe ik_probe edge_scan traverse_probe acm_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
# regression harness (python, no build step needed)
install(PROGRAMS tools/edge_regress DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(rosim_planning_core)
ament_export_dependencies(
  rclcpp rclcpp_action rosim_interfaces rosim_core
  moveit_core moveit_ros_planning moveit_msgs geometric_shapes pluginlib
  tf2_eigen trajectory_msgs sensor_msgs Eigen3 Boost)

ament_package()
