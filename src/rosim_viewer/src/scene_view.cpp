// Qt+VTK viewport: renderer setup, ground/axes, workpiece and edge actors, and
// the edge hover/click interactor style (a click is a press/release within
// 4 px; camera drags never select an edge).

#include "rosim_viewer/scene_view.h"

#include <cmath>
#include <cstdlib>

#include <QVBoxLayout>

#include <vtkAxesActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkLight.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include "rosim_core/occ_to_vtk.h"

namespace rosim::viewer
{

// ---------------------------------------------------------------------------
// EdgeInteractorStyle
// ---------------------------------------------------------------------------
vtkStandardNewMacro(EdgeInteractorStyle);

void EdgeInteractorStyle::OnMouseMove()
{
  if (pick_at && hover_callback)
  {
    const int x = this->Interactor->GetEventPosition()[0];
    const int y = this->Interactor->GetEventPosition()[1];
    const int idx = pick_at(x, y);
    if (idx != last_hover_)
    {
      last_hover_ = idx;
      hover_callback(idx);
    }
  }
  vtkInteractorStyleTrackballCamera::OnMouseMove();
}

void EdgeInteractorStyle::OnLeftButtonDown()
{
  this->Interactor->GetEventPosition(press_pos_);
  vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void EdgeInteractorStyle::OnLeftButtonUp()
{
  if (select_callback)
  {
    int pos[2];
    this->Interactor->GetEventPosition(pos);
    const int dx = pos[0] - press_pos_[0];
    const int dy = pos[1] - press_pos_[1];
    if (std::abs(dx) <= 4 && std::abs(dy) <= 4 && pick_at)
    {
      const int idx = pick_at(pos[0], pos[1]);
      select_callback(idx);
    }
  }
  vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

// ---------------------------------------------------------------------------
// SceneView
// ---------------------------------------------------------------------------
SceneView::SceneView(QWidget* parent) : QWidget(parent)
{
  vtk_widget_ = new QVTKOpenGLNativeWidget(this);

  auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  vtk_widget_->SetRenderWindow(render_window);

  renderer_->SetBackground(0.10, 0.11, 0.14);
  renderer_->SetBackground2(0.16, 0.18, 0.23);
  renderer_->GradientBackgroundOn();
  render_window->AddRenderer(renderer_.Get());

  // lights: key + fill
  {
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
  }

  buildGround();

  // camera (framed for the GP12: ~1.4 m reach, flange at ~1.26 m at zeros)
  vtkCamera* camera = renderer_->GetActiveCamera();
  camera->SetPosition(2.6, -2.3, 1.9);
  camera->SetFocalPoint(0.55, 0.0, 0.6);
  camera->SetViewUp(0, 0, 1);
  camera->SetClippingRange(0.02, 30.0);
  renderer_->ResetCamera();

  // picking setup: only edge tubes are pickable
  picker_->SetPickFromList(1);
  picker_->SetTolerance(0.025);

  style_ = vtkSmartPointer<EdgeInteractorStyle>::New();
  vtk_widget_->GetInteractor()->SetInteractorStyle(style_);

  style_->pick_at = [this](int x, int y) {
    int idx = -1;
    pickTargetAt(x, y, idx);
    return idx;
  };
  style_->hover_callback = [this](int idx) {
    setHighlightedEdge(idx);                 // local look
    emit edgeHovered(idx);                   // GUI bookkeeping
  };
  style_->select_callback = [this](int idx) { emit edgeSelected(idx); };

  // layout
  auto layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(vtk_widget_);
}

vtkRenderWindow* SceneView::renderWindow() const
{
  return vtk_widget_->renderWindow();
}

void SceneView::render()
{
  vtk_widget_->renderWindow()->Render();
}

void SceneView::buildGround()
{
  // 10 cm grid lines
  {
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    double bounds[4] = { -0.75, 1.85, -1.3, 1.3 };
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
  }
  // world axes (small, at origin)
  {
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(0.25, 0.25, 0.25);
    axes->SetConeRadius(0.02);
    axes->AxisLabelsOff();
    renderer_->AddActor(axes);
  }
}

void SceneView::addWorkpiece(const std::string& model_id,
                             const rosim::core::MeshData& solid_mesh,
                             const std::vector<rosim::core::EdgeData>& edges,
                             const Eigen::Isometry3d& world_pose)
{
  removeWorkpiece(model_id);

  WorkpieceActors wp;
  wp.edges = edges;
  // solid is in the shape's LOCAL frame; place it where the edges are
  wp.solid = rosim::core::buildSolidActor(solid_mesh, world_pose);
  renderer_->AddActor(wp.solid);

  for (const auto& edge : edges)
  {
    auto actor = rosim::core::buildEdgeActor(edge);
    renderer_->AddActor(actor);
    picker_->AddPickList(actor);
    wp.edge_actors.push_back(actor);
  }
  workpieces_[model_id] = wp;
  render();
}

void SceneView::removeWorkpiece(const std::string& model_id)
{
  const auto it = workpieces_.find(model_id);
  if (it == workpieces_.end())
    return;
  renderer_->RemoveActor(it->second.solid);
  for (const auto& actor : it->second.edge_actors)
  {
    renderer_->RemoveActor(actor);
    picker_->DeletePickList(actor);
  }
  workpieces_.erase(it);
}

void SceneView::clearWorkpieces()
{
  std::vector<std::string> ids;
  ids.reserve(workpieces_.size());
  for (const auto& [id, _] : workpieces_)
    ids.push_back(id);
  for (const auto& id : ids)
    removeWorkpiece(id);
}

void SceneView::setHighlightedEdge(int edge_index)
{
  // clear previous
  for (auto& [id, wp] : workpieces_)
    for (const auto& actor : wp.edge_actors)
      rosim::core::setEdgeHighlight(actor, false);

  highlighted_edge_ = edge_index;
  if (edge_index < 0)
    return;

  for (auto& [id, wp] : workpieces_)
  {
    if (edge_index < static_cast<int>(wp.edge_actors.size()))
    {
      rosim::core::setEdgeHighlight(wp.edge_actors[edge_index], true);
      render();
      return;
    }
  }
}

std::vector<rosim::core::EdgeData> SceneView::edgesOf(const std::string& model_id) const
{
  const auto it = workpieces_.find(model_id);
  return (it == workpieces_.end()) ? std::vector<rosim::core::EdgeData>() : it->second.edges;
}

std::string SceneView::modelOfEdge(int edge_index) const
{
  for (const auto& [id, wp] : workpieces_)
    if (edge_index >= 0 && edge_index < static_cast<int>(wp.edge_actors.size()))
      return id;
  return "";
}

vtkSmartPointer<vtkActor> SceneView::pickTargetAt(int x, int y, int& edge_index_out)
{
  edge_index_out = -1;
  if (!picker_->Pick(x, y, 0, renderer_.Get()))
    return nullptr;
  vtkActor* picked = vtkActor::SafeDownCast(picker_->GetProp3D());
  if (!picked)
    return nullptr;

  for (const auto& [id, wp] : workpieces_)
    for (size_t i = 0; i < wp.edge_actors.size(); ++i)
      if (wp.edge_actors[i] == picked)
      {
        edge_index_out = static_cast<int>(i);
        return wp.edge_actors[i];
      }
  return nullptr;
}

}  // namespace rosim::viewer
