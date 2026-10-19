#pragma once

#include <Eigen/Geometry>
#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "rosim_core/occ_geometry_service.h"

namespace rosim::core
{

/// Eigen::Isometry3d -> vtkMatrix4x4, filled element-wise via SetElement (row,
/// col). Do NOT build a flat[16] and DeepCopy it instead: that copies straight
/// row-major, so a column-major layout (translation at [12..14]) silently moves
/// the translation into the bottom row — every actor renders at the origin,
/// stacked. Shared by workpiece solid placement and the robot link renderer.
vtkSmartPointer<vtkMatrix4x4> eigenToVtkMatrix(const Eigen::Isometry3d& pose);

/// Triangle-soup MeshData -> vtkPolyData (points + polys).
vtkSmartPointer<vtkPolyData> buildSolidPolyData(const MeshData& mesh);

/// Workpiece solid actor: shaded + normals + gentle specular.
/// MeshData carries the shape's LOCAL frame — the returned actor is at the
/// identity pose; use the pose overload to place it in the world.
vtkSmartPointer<vtkActor> buildSolidActor(const MeshData& mesh);

/// Same, with the world pose applied as the actor's user matrix so the solid
/// coincides with the (world-frame) edge actors.
vtkSmartPointer<vtkActor> buildSolidActor(const MeshData& mesh,
                                          const Eigen::Isometry3d& world_pose);

/// One edge as a thin tube (pickable, visually crisp). Tube radius ~1.5 mm.
/// Default (unhighlighted) color: dark amber.
vtkSmartPointer<vtkActor> buildEdgeActor(const EdgeData& edge);

/// Toggle the highlight look of an edge actor (cyan + emissive boost).
void setEdgeHighlight(vtkActor* actor, bool highlighted);

}  // namespace rosim::core
