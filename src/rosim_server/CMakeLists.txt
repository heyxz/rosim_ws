cmake_minimum_required(VERSION 3.8)
project(rosim_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

find_package(VTK REQUIRED COMPONENTS
  CommonCore CommonDataModel CommonTransforms
  FiltersCore FiltersSources FiltersGeometry
  RenderingCore RenderingOpenGL2 RenderingFreeType RenderingAnnotation
  InteractionStyle
  IOImage IOGeometry CommonComputationalGeometry)
message(STATUS "rosim_server: VTK ${VTK_VERSION} (${VTK_RENDERING_BACKEND})")

# Headless offscreen renderer (VTK) — the only VTK-using library. Kept separate
# so the X/GLX concrete window type is confined to its .cpp (PIMPL).
add_library(rosim_server_core SHARED
  src/frame_renderer.cpp
)
target_include_directories(rosim_server_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rosim_server_core ${VTK_LIBRARIES})
target_include_directories(rosim_server_core SYSTEM PUBLIC ${VTK_INCLUDE_DIRS})
ament_target_dependencies(rosim_server_core
  rclcpp rosim_core moveit_core moveit_msgs sensor_msgs Eigen3 Boost)

# WebSocket/HTTP server + ROS bridge node (Boost.Beast + nlohmann/json, both
# header-only; nlohmann is vendored under include/rosim_server/third_party).
add_executable(rosim_server_node
  src/server_main.cpp
  src/server_node.cpp
  src/ws_server.cpp
)
target_include_directories(rosim_server_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rosim_server/third_party>)
target_link_libraries(rosim_server_node rosim_server_core ${VTK_LIBRARIES} Threads::Threads)
target_include_directories(rosim_server_node SYSTEM PRIVATE ${VTK_INCLUDE_DIRS})
ament_target_dependencies(rosim_server_node
  rclcpp rclcpp_action rosim_interfaces rosim_core moveit_core moveit_ros_planning
  moveit_msgs sensor_msgs geometry_msgs trajectory_msgs tf2_eigen Eigen3 Boost)

add_executable(render_probe tools/render_probe.cpp)
target_link_libraries(render_probe rosim_server_core ${VTK_LIBRARIES})
target_include_directories(render_probe SYSTEM PRIVATE ${VTK_INCLUDE_DIRS})
ament_target_dependencies(render_probe rclcpp rosim_core moveit_core Eigen3 Boost)

# Register the OpenGL2 backend object factories (vtkRenderer -> vtkOpenGLRenderer,
# vtkPolyDataMapper -> vtkOpenGLPolyDataMapper, ...). Without this the factories
# return the abstract base classes and offscreen rendering silently produces black.
vtk_module_autoinit(
  TARGETS rosim_server_core render_probe rosim_server_node
  MODULES ${VTK_LIBRARIES})

install(TARGETS rosim_server_core render_probe rosim_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Web UI: served over HTTP by the WS server. web_root is resolved at runtime via
# ament_index to share/${PROJECT_NAME}/web.
install(DIRECTORY web DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
