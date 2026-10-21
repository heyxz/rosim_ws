cmake_minimum_required(VERSION 3.8)
project(rosim_rl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ROSIM_WITH_TORCH "Link the RL planner against /opt/libtorch (TorchScript policies)" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosim_planning REQUIRED)
find_package(rosim_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rosim_rl SHARED src/rl_remote_planner.cpp)
set_target_properties(rosim_rl PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rosim_rl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rosim_rl
  rclcpp rosim_planning rosim_interfaces moveit_core moveit_msgs Eigen3)
# rl_remote_planner.cpp 用到 rosim::planning::regularizeTiming（在
# rosim_planning_core 里）。这个库是 dlopen 进规划节点的插件，宿主进程已经
# 加载了 rosim_planning_core，符号在运行期解析，不必静态链接。

if(ROSIM_WITH_TORCH)
  set(Torch_DIR "/opt/libtorch/lib/cmake/Torch")
  find_package(Torch REQUIRED)
  target_compile_definitions(rosim_rl PUBLIC ROSIM_WITH_TORCH=1)
  target_link_libraries(rosim_rl ${TORCH_LIBRARIES})
  message(STATUS "rosim_rl: libtorch enabled")
else()
  message(STATUS "rosim_rl: libtorch disabled (stub policy); set -DROSIM_WITH_TORCH=ON to enable")
endif()

install(TARGETS rosim_rl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY python/ DESTINATION share/${PROJECT_NAME}/python)
# expose rosim_gym on PYTHONPATH so `python3 -m rosim_gym.random_rollout` works
# right after sourcing the workspace
ament_environment_hooks(env-hooks/${PROJECT_NAME}.dsv.in)

ament_export_include_directories(include)
ament_export_libraries(rosim_rl)
ament_export_dependencies(rclcpp rosim_planning rosim_interfaces moveit_core moveit_msgs Eigen3)

ament_package()
