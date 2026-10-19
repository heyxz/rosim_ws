// VTK actor builders shared by the Qt viewer and the headless server renderer:
// workpiece solid, pickable edge tubes, highlight styling, and the Eigen->VTK
// matrix conversion that keeps translations in the correct column.

#include "rosim_core/occ_to_vtk.h"

#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkLine.h>
#include <vtkMatrix4x4.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkTubeFilter.h>

namespace rosim::core
{

vtkSmartPointer<vtkMatrix4x4> eigenToVtkMatrix(const Eigen::Isometry3d& pose)
{
  auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      matrix->SetElement(r, c, pose.linear()(r, c));
  matrix->SetElement(0, 3, pose.translation().x());
  matrix->SetElement(1, 3, pose.translation().y());
  matrix->SetElement(2, 3, pose.translation().z());
  matrix->SetElement(3, 0, 0.0);
  matrix->SetElement(3, 1, 0.0);
  matrix->SetElement(3, 2, 0.0);
  matrix->SetElement(3, 3, 1.0);
  return matrix;
}

vtkSmartPointer<vtkPolyData> buildSolidPolyData(const MeshData& mesh)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();
  const vtkIdType n_points = static_cast<vtkIdType>(mesh.vertices.size() / 3);
  points->SetNumberOfPoints(n_points);
  for (vtkIdType i = 0; i < n_points; ++i)
    points->SetPoint(i, mesh.vertices[3 * i], mesh.vertices[3 * i + 1], mesh.vertices[3 * i + 2]);

  auto polys = vtkSmartPointer<vtkCellArray>::New();
  const vtkIdType n_tris = static_cast<vtkIdType>(mesh.triangles.size() / 3);
  polys->EstimateSize(n_tris, 3);
  for (vtkIdType i = 0; i < n_tris; ++i)
  {
    // triangles are uint32; vtkIdType is 64-bit — widen per-index, never
    // reinterpret-cast the storage (reading 3 int64 from 3 uint32 mashes
    // adjacent indices into garbage point ids and segfaults BuildLinks)
    const vtkIdType cell[3] = { static_cast<vtkIdType>(mesh.triangles[3 * i]),
                                static_cast<vtkIdType>(mesh.triangles[3 * i + 1]),
                                static_cast<vtkIdType>(mesh.triangles[3 * i + 2]) };
    polys->InsertNextCell(3, cell);
  }

  auto polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetPolys(polys);
  return polydata;
}

vtkSmartPointer<vtkActor> buildSolidActor(const MeshData& mesh)
{
  auto polydata = buildSolidPolyData(mesh);

  auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputData(polydata);
  normals->ComputeCellNormalsOn();
  normals->ComputePointNormalsOn();
  normals->SplittingOn();
  normals->ConsistencyOn();
  normals->Update();

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(normals->GetOutputPort());

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0.72, 0.72, 0.78);
  actor->GetProperty()->SetInterpolationToPhong();
  actor->GetProperty()->SetSpecular(0.25);
  actor->GetProperty()->SetSpecularPower(20.0);
  return actor;
}

vtkSmartPointer<vtkActor> buildSolidActor(const MeshData& mesh,
                                          const Eigen::Isometry3d& world_pose)
{
  auto actor = buildSolidActor(mesh);
  actor->SetUserMatrix(eigenToVtkMatrix(world_pose));
  return actor;
}

vtkSmartPointer<vtkActor> buildEdgeActor(const EdgeData& edge)
{
  auto points = vtkSmartPointer<vtkPoints>::New();
  auto lines = vtkSmartPointer<vtkCellArray>::New();

  points->SetDataTypeToDouble();
  const vtkIdType n_points = static_cast<vtkIdType>(edge.polyline.size());
  points->SetNumberOfPoints(n_points);
  for (vtkIdType i = 0; i < n_points; ++i)
  {
    const auto& p = edge.polyline[i];
    points->SetPoint(i, p.x(), p.y(), p.z());
  }

  auto line = vtkSmartPointer<vtkLine>::New();
  for (vtkIdType i = 0; i + 1 < n_points; ++i)
  {
    line->GetPointIds()->SetId(0, i);
    line->GetPointIds()->SetId(1, i + 1);
    lines->InsertNextCell(line);
  }
  // Closed edges additionally wrap around.
  if (edge.closed && n_points > 2)
  {
    line->GetPointIds()->SetId(0, n_points - 1);
    line->GetPointIds()->SetId(1, 0);
    lines->InsertNextCell(line);
  }

  auto polyline = vtkSmartPointer<vtkPolyData>::New();
  polyline->SetPoints(points);
  polyline->SetLines(lines);

  // Tube instead of bare polyline: comfortably pickable and visible at distance.
  auto tube = vtkSmartPointer<vtkTubeFilter>::New();
  tube->SetInputData(polyline);
  tube->SetRadius(0.0015);
  tube->SetNumberOfSides(8);
  tube->SetCapping(true);

  auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(tube->GetOutputPort());

  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  setEdgeHighlight(actor, false);
  return actor;
}

void setEdgeHighlight(vtkActor* actor, bool highlighted)
{
  if (!actor)
    return;
  if (highlighted)
  {
    actor->GetProperty()->SetColor(0.0, 0.95, 1.0);
    actor->GetProperty()->SetOpacity(1.0);
    actor->GetProperty()->SetSpecular(0.8);
  }
  else
  {
    actor->GetProperty()->SetColor(0.75, 0.45, 0.05);
    actor->GetProperty()->SetOpacity(0.95);
    actor->GetProperty()->SetSpecular(0.2);
  }
}

}  // namespace rosim::core
