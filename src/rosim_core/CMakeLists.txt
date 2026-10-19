cmake_minimum_required(VERSION 3.8)
project(rosim_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_CXX_FLAGS MATCHES "std=c\\+\\+")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# ---------------------------------------------------------------------------
# OpenCASCADE. Main path: distro cmake config. Fallback: raw TK libraries.
# (apt: libocct-foundation/modeling-data/modeling-algorithms/data-exchange-dev)
# ---------------------------------------------------------------------------
find_package(OCCT QUIET)
if(OCCT_FOUND)
  message(STATUS "rosim_core: OCCT ${OCCT_VERSION} found via cmake config")
  set(ROSIM_OCCT_LIBS ${OCCT_LIBRARIES})
  set(ROSIM_OCCT_INCLUDES ${OCCT_INCLUDE_DIRS})
else()
  message(WARNING
    "rosim_core: OCCT cmake config not found, falling back to manual OCCT paths")
  set(ROSIM_OCCT_INCLUDES /usr/include/opencascade)
  set(ROSIM_OCCT_LIBS
      TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep
      TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool TKMesh TKShHealing
      TKXSBase TKSTEPBase TKSTEPAttr TKSTEP209 TKSTEP)
endif()

# ---------------------------------------------------------------------------
# librosim_occ — STEP/BREP import, edge extraction, discretization,
# tessellation, collision mesh. No VTK/Qt dependency (planning node uses this).
# ---------------------------------------------------------------------------
add_library(rosim_occ SHARED
  src/occ_geometry_service.cpp
  src/edge_frame_generator.cpp
  src/seam_builder.cpp
  src/corner_planner.cpp
)
set_target_properties(rosim_occ PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rosim_occ PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ROSIM_OCCT_INCLUDES})
target_link_libraries(rosim_occ ${ROSIM_OCCT_LIBS})
ament_target_dependencies(rosim_occ
  rclcpp moveit_core moveit_msgs geometric_shapes Eigen3 Boost)

# ---------------------------------------------------------------------------
# librosim_viz — OCC mesh -> VTK polydata, workpiece/edge actors, robot FK
# actors. Depends on rosim_occ and VTK.
# ---------------------------------------------------------------------------
find_package(VTK QUIET COMPONENTS
  CommonCore CommonDataModel CommonTransforms
  FiltersCore FiltersGeneral FiltersSources FiltersGeometry
  RenderingCore RenderingAnnotation RenderingOpenGL2 RenderingLOD
  InteractionStyle IOGeometry IOPLY CommonComputationalGeometry)

if(VTK_FOUND)
  add_library(rosim_viz SHARED
    src/occ_to_vtk.cpp
    src/robot_link_actors.cpp
  )
  set_target_properties(rosim_viz PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(rosim_viz PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(rosim_viz rosim_occ ${VTK_LIBRARIES})
  target_include_directories(rosim_viz SYSTEM PUBLIC ${VTK_INCLUDE_DIRS})
  ament_target_dependencies(rosim_viz
    rclcpp moveit_core moveit_msgs geometric_shapes Eigen3 Boost)
  set(ROSIM_VIZ_TARGETS rosim_viz)
else()
  message(WARNING "rosim_core: VTK not found, librosim_viz skipped")
endif()

# ---------------------------------------------------------------------------
# tools
# ---------------------------------------------------------------------------
add_executable(rosim_step_generator tools/step_generator.cpp)
target_link_libraries(rosim_step_generator rosim_occ)
ament_target_dependencies(rosim_step_generator rclcpp Eigen3)

add_executable(test_occ_step test/test_occ_step.cpp)
target_link_libraries(test_occ_step rosim_occ)
ament_target_dependencies(test_occ_step rclcpp Eigen3)

add_executable(corner_probe tools/corner_probe.cpp)
target_link_libraries(corner_probe rosim_occ)
ament_target_dependencies(corner_probe rclcpp Eigen3)

add_executable(seam_probe tools/seam_probe.cpp)
target_link_libraries(seam_probe rosim_occ)
ament_target_dependencies(seam_probe rclcpp Eigen3)

add_executable(gp12_prepare tools/gp12_prepare.cpp)
target_link_libraries(gp12_prepare rosim_occ)
ament_target_dependencies(gp12_prepare rclcpp Eigen3)

install(TARGETS rosim_occ rosim_step_generator test_occ_step seam_probe gp12_prepare
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
if(VTK_FOUND)
  install(TARGETS ${ROSIM_VIZ_TARGETS}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
endif()
# probe tools usable via `ros2 run rosim_core <tool>`
install(TARGETS seam_probe corner_probe test_occ_step RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include ${ROSIM_OCCT_INCLUDES})
set(ROSIM_EXPORT_LIBS rosim_occ)
if(VTK_FOUND)
  list(APPEND ROSIM_EXPORT_LIBS rosim_viz)
endif()
ament_export_libraries(${ROSIM_EXPORT_LIBS} ${ROSIM_OCCT_LIBS})
ament_export_dependencies(rclcpp moveit_core moveit_msgs geometric_shapes Eigen3 Boost)

ament_package()
