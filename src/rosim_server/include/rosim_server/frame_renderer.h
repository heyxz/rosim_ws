#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <vtkJPEGWriter.h>
#include <vtkNew.h>
#include <vtkOpenGLRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

// X/GLX window type is forward-declared here: its header pulls in <X11/Xlib.h>
// whose macros (None, Status, ...) clash with rclcpp. The concrete type is only
// included in frame_renderer.cpp, after the rclcpp/moveit headers.
class vtkXOpenGLRenderWindow;

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_core/robot_link_actors.h"

namespace rosim::server
{

/// \brief Headless VTK renderer: owns an offscreen render window, the robot
/// link actors and the workpiece/edge actors, and grabs JPEG frames for
/// streaming to the browser. Mirrors the responsibility of the old Qt
/// SceneView, minus the widget embedding.
class FrameRenderer
{
public:
  FrameRenderer(int width, int height);
  ~FrameRenderer();

  /// \param model robot model for FK display (may be null: robot skipped).
  bool init(const moveit::core::RobotModelPtr& model);

  vtkRenderer* renderer() { return renderer_.Get(); }

  /// Push link transforms from a state whose updateLinkTransforms() has
  /// already been called.
  void updateRobot(const moveit::core::RobotState& state);

  /// \param reset_camera false keeps the current stream camera (used when the
  ///        part is only moved, not newly imported).
  void addWorkpiece(const std::string& model_id, const rosim::core::MeshData& solid,
                    const std::vector<rosim::core::EdgeData>& edges,
                    const Eigen::Isometry3d& world_pose, bool reset_camera = true);
  void removeWorkpiece(const std::string& model_id);
  void clearWorkpieces();

  /// -1 clears; otherwise highlights edge `edge_index` of every workpiece
  /// (single-workpiece usage: index == the edge's enumeration index).
  void setHighlightedEdge(int edge_index);

  /// Render the current scene and encode it as a JPEG image (in memory).
  std::vector<uint8_t> captureJpeg(int quality = 70);

  int width() const { return width_; }
  int height() const { return height_; }

private:
  void buildGround();
  void resetCamera();

  int width_;
  int height_;

  vtkNew<vtkOpenGLRenderer> renderer_;
  vtkSmartPointer<vtkXOpenGLRenderWindow> window_;
  vtkNew<vtkWindowToImageFilter> window_to_image_;
  vtkNew<vtkJPEGWriter> jpeg_writer_;

  std::unique_ptr<rosim::core::RobotLinkActors> robot_actors_;

  struct WorkpieceActors
  {
    vtkSmartPointer<vtkActor> solid;
    std::vector<vtkSmartPointer<vtkActor>> edge_actors;
  };
  std::map<std::string, WorkpieceActors> workpieces_;
};

}  // namespace rosim::server
