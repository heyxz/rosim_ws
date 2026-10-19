#pragma once

#include <functional>
#include <map>
#include <memory>

#include <QVTKOpenGLNativeWidget.h>
#include <QWidget>

#include <vtkActor.h>
#include <vtkCellPicker.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include "rosim_core/occ_geometry_service.h"

namespace rosim::viewer
{

/// \brief Custom interaction: trackball camera + edge hover/click picking.
/// A left click that did not drag (<= 4 px) is a pick attempt.
class EdgeInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static EdgeInteractorStyle* New();
  vtkTypeMacro(EdgeInteractorStyle, vtkInteractorStyleTrackballCamera);

  void OnMouseMove() override;
  void OnLeftButtonDown() override;
  void OnLeftButtonUp() override;

  /// Registered by SceneView; return the edge index under the cursor or -1.
  std::function<int(int x, int y)> pick_at;
  std::function<void(int)> hover_callback;   // edge index or -1
  std::function<void(int)> select_callback;  // edge index or -1

private:
  int press_pos_[2] = { 0, 0 };
  int last_hover_ = -1;
};

/// \brief The 3D viewport: renderer, lights, ground grid, world axes,
/// workpiece + edge actors, and edge picking.
class SceneView : public QWidget
{
  Q_OBJECT

public:
  explicit SceneView(QWidget* parent = nullptr);

  vtkRenderer* renderer() const { return renderer_.Get(); }
  QVTKOpenGLNativeWidget* widget() const { return vtk_widget_; }
  vtkRenderWindow* renderWindow() const;

  void addWorkpiece(const std::string& model_id, const rosim::core::MeshData& solid_mesh,
                    const std::vector<rosim::core::EdgeData>& edges,
                    const Eigen::Isometry3d& world_pose = Eigen::Isometry3d::Identity());
  void removeWorkpiece(const std::string& model_id);
  void clearWorkpieces();

  /// Highlight control; -1 clears. Emits edgeHovered/edgeSelected via the style.
  void setHighlightedEdge(int edge_index);
  int highlightedEdge() const { return highlighted_edge_; }

  std::vector<rosim::core::EdgeData> edgesOf(const std::string& model_id) const;
  std::string modelOfEdge(int edge_index) const;

public slots:
  void render();

signals:
  void edgeHovered(int edge_index);
  void edgeSelected(int edge_index);

private:
  void buildGround();
  vtkSmartPointer<vtkActor> pickTargetAt(int x, int y, int& edge_index_out);

  QVTKOpenGLNativeWidget* vtk_widget_ = nullptr;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkCellPicker> picker_;
  vtkSmartPointer<EdgeInteractorStyle> style_;

  struct WorkpieceActors
  {
    vtkSmartPointer<vtkActor> solid;
    std::vector<vtkSmartPointer<vtkActor>> edge_actors;  // order == EdgeData index
    std::vector<rosim::core::EdgeData> edges;
  };
  std::map<std::string, WorkpieceActors> workpieces_;

  int highlighted_edge_ = -1;
};

}  // namespace rosim::viewer
