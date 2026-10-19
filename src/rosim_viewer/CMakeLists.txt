cmake_minimum_required(VERSION 3.8)
project(rosim_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosim_interfaces REQUIRED)
find_package(rosim_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(VTK REQUIRED COMPONENTS
  CommonCore CommonDataModel CommonTransforms
  FiltersCore FiltersGeneral FiltersSources FiltersGeometry
  RenderingCore RenderingAnnotation RenderingOpenGL2 RenderingLOD
  InteractionStyle InteractionWidgets
  GUISupportQt RenderingQt
  IOGeometry IOPLY CommonComputationalGeometry)
message(STATUS "rosim_viewer: VTK ${VTK_VERSION} (${VTK_RENDERING_BACKEND}) with Qt support")

set(VIEWER_HEADERS
  include/rosim_viewer/ros_bridge.h
  include/rosim_viewer/scene_view.h
  include/rosim_viewer/robot_assembly.h
  include/rosim_viewer/main_window.h
)

add_executable(rosim_viewer
  src/main.cpp
  src/ros_bridge.cpp
  src/scene_view.cpp
  src/robot_assembly.cpp
  src/main_window.cpp
  ${VIEWER_HEADERS}
)
target_include_directories(rosim_viewer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rosim_viewer
  Qt5::Widgets ${VTK_LIBRARIES})
target_include_directories(rosim_viewer SYSTEM PRIVATE ${VTK_INCLUDE_DIRS})
ament_target_dependencies(rosim_viewer
  ament_index_cpp rclcpp rclcpp_action rosim_interfaces rosim_core
  moveit_core moveit_ros_planning moveit_msgs sensor_msgs trajectory_msgs
  tf2_eigen Eigen3 Boost)

# dev tool: headless repro of the GUI import display sequence (crash probe)
add_executable(crash_probe tools/crash_probe.cpp)
target_link_libraries(crash_probe ${VTK_LIBRARIES})
target_include_directories(crash_probe SYSTEM PRIVATE ${VTK_INCLUDE_DIRS})
ament_target_dependencies(crash_probe rosim_core moveit_core Eigen3 Boost)

# dev tool: robot display transform path check (FK vs vtk UserMatrix)
add_executable(robot_pose_probe tools/robot_pose_probe.cpp)
target_link_libraries(robot_pose_probe ${VTK_LIBRARIES})
target_include_directories(robot_pose_probe SYSTEM PRIVATE ${VTK_INCLUDE_DIRS})
ament_target_dependencies(robot_pose_probe rosim_core moveit_core moveit_ros_planning
  moveit_msgs Eigen3 Boost)

install(TARGETS rosim_viewer crash_probe robot_pose_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
