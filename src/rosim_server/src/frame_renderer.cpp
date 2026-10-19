// Headless VTK renderer: offscreen X/GLX window, robot link actors, workpiece
// solid + edge actors, camera framing, and in-memory JPEG capture for the
// optional binary stream. The X11-bearing window type is confined to this TU.

#include "rosim_server/frame_renderer.h"

// Concrete X/GLX window, included only here (after the rclcpp/moveit headers
// pulled in by frame_renderer.h) so its X11 macros do not collide with rclcpp.
#include <vtkXOpenGLRenderWindow.h>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkLight.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>

#include "rosim_core/occ_to_vtk.h"

namespace rosim::server
{

FrameRenderer::FrameRenderer(int width, int height) : width_(width), height_(height)
{
}

FrameRenderer::~FrameRenderer() = default;

bool FrameRenderer::init(const moveit::core::RobotModelPtr& model)
{
  window_ = vtkSmartPointer<vtkXOpenGLRenderWindow>::New();
  window_->SetOffScreenRendering(true);
  window_->SetSize(width_, height_);
  window_->AddRenderer(renderer_.Get());

  renderer_->SetBackground(0.10, 0.11, 0.14);
  renderer_->SetBackground2(0.16, 0.18, 0.23);
  renderer_->GradientBackgroundOn();

  // key + fill lights (same placement as the old Qt SceneView)
  auto key = vtkSmartPointer<vtkLight>::New();
  key->SetLightTypeToSceneLight();
  key->SetPosition(2.0, -2.5, 3.0);
  key->SetFocalPoint(0, 0, 0);
  key->SetIntensity(0.9);
  renderer_->AddLight(key);

  auto fill = vtkSmartPointer<vtkLight>::New();
  fill->SetLightTypeToSceneLight();
  fill->SetPosition(-2.5, 2.0, 1.5);
  fill->SetFocalPoint(0, 0, 0);
  fill->SetIntensity(0.45);
  renderer_->AddLight(fill);

  buildGround();

  if (model)
  {
    robot_actors_ = std::make_unique<rosim::core::RobotLinkActors>();
    if (!robot_actors_->init(model))
      robot_actors_.reset();
    else
      robot_actors_->addToRenderer(renderer_.Get());
  }

  resetCamera();

  // capture pipeline: window -> image -> JPEG
  window_to_image_->SetInput(window_.Get());
  // Read the default (back) buffer: offscreen rendering draws into an internal
  // FBO whose color attachment is read via the back-buffer path, so leave the
  // filter default. (The unrelated ~1 s/frame vsync stall that showed up here
  // was fixed by __GL_SYNC_TO_VBLANK=0 in server_main.cpp, not by this.)
  jpeg_writer_->SetInputConnection(window_to_image_->GetOutputPort());
  jpeg_writer_->WriteToMemoryOn();

  // Deliberately do NOT Render() here: the GL context must be created on the
  // thread that will drive the frame loop (render thread). Rendering here would
  // bind the context to the caller's thread and break the later captures. The
  // first captureJpeg() makes the context current on its own thread.
  return true;
}

void FrameRenderer::updateRobot(const moveit::core::RobotState& state)
{
  if (robot_actors_)
    robot_actors_->updateFromState(state);
}

void FrameRenderer::addWorkpiece(const std::string& model_id,
                                 const rosim::core::MeshData& solid,
                                 const std::vector<rosim::core::EdgeData>& edges,
                                 const Eigen::Isometry3d& world_pose, bool reset_camera)
{
  removeWorkpiece(model_id);

  WorkpieceActors wp;
  wp.solid = rosim::core::buildSolidActor(solid, world_pose);
  renderer_->AddActor(wp.solid);
  for (const auto& edge : edges)
  {
    auto actor = rosim::core::buildEdgeActor(edge);
    renderer_->AddActor(actor);
    wp.edge_actors.push_back(actor);
  }
  workpieces_[model_id] = wp;

  if (reset_camera)
    resetCamera();
}

void FrameRenderer::removeWorkpiece(const std::string& model_id)
{
  const auto it = workpieces_.find(model_id);
  if (it == workpieces_.end())
    return;
  renderer_->RemoveActor(it->second.solid);
  for (const auto& actor : it->second.edge_actors)
    renderer_->RemoveActor(actor);
  workpieces_.erase(it);
}

void FrameRenderer::clearWorkpieces()
{
  std::vector<std::string> ids;
  ids.reserve(workpieces_.size());
  for (const auto& [id, _] : workpieces_)
    ids.push_back(id);
  for (const auto& id : ids)
    removeWorkpiece(id);
}

void FrameRenderer::setHighlightedEdge(int edge_index)
{
  for (auto& [id, wp] : workpieces_)
    for (const auto& actor : wp.edge_actors)
      rosim::core::setEdgeHighlight(actor, false);

  if (edge_index < 0)
    return;

  for (auto& [id, wp] : workpieces_)
  {
    if (edge_index < static_cast<int>(wp.edge_actors.size()))
      rosim::core::setEdgeHighlight(wp.edge_actors[edge_index], true);
  }
}

std::vector<uint8_t> FrameRenderer::captureJpeg(int quality)
{
  // The GL context is owned by whichever thread calls captureJpeg (the render
  // thread). MakeCurrent() on the first call creates it here, so rendering is
  // never attempted from a different thread than the context belongs to.
  window_->MakeCurrent();
  window_->Render();
  window_to_image_->Modified();
  window_to_image_->Update();
  jpeg_writer_->SetQuality(quality);
  jpeg_writer_->Write();

  vtkUnsignedCharArray* result = jpeg_writer_->GetResult();
  if (!result || result->GetNumberOfValues() == 0)
    return {};
  return std::vector<uint8_t>(result->GetPointer(0),
                              result->GetPointer(0) + result->GetNumberOfValues());
}

void FrameRenderer::buildGround()
{
  // 10 cm grid lines (same bounds as the old Qt SceneView)
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  // 10 cm grid lines; wide enough for the 2.8 m linear rail and the part.
  double bounds[4] = { -1.6, 2.0, -1.4, 1.4 };
  vtkIdType pid = 0;
  for (double x = bounds[0]; x <= bounds[1] + 1e-9; x += 0.1)
  {
    points->InsertNextPoint(x, bounds[2], 0.0);
    points->InsertNextPoint(x, bounds[3], 0.0);
    vtkIdType ids[2] = { pid, pid + 1 };
    lines->InsertNextCell(2, ids);
    pid += 2;
  }
  for (double y = bounds[2]; y <= bounds[3] + 1e-9; y += 0.1)
  {
    points->InsertNextPoint(bounds[0], y, 0.0);
    points->InsertNextPoint(bounds[1], y, 0.0);
    vtkIdType ids[2] = { pid, pid + 1 };
    lines->InsertNextCell(2, ids);
    pid += 2;
  }
  auto polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetLines(lines);
  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(polydata);
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0.32, 0.34, 0.40);
  actor->GetProperty()->SetOpacity(0.5);
  renderer_->AddActor(actor);

  // small world axes at the origin
  auto axes = vtkSmartPointer<vtkAxesActor>::New();
  axes->SetTotalLength(0.25, 0.25, 0.25);
  axes->SetConeRadius(0.02);
  axes->AxisLabelsOff();
  renderer_->AddActor(axes);
}

void FrameRenderer::resetCamera()
{
  renderer_->ResetCamera();
}

}  // namespace rosim::server
