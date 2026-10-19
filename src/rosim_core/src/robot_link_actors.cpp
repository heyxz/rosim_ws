// VTK robot rendering: one actor per URDF collision shape, with per-shape
// origin offsets, updated from a moveit RobotState (FK) on every joint update.

#include "rosim_core/robot_link_actors.h"

#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <geometric_shapes/shapes.h>
#include <moveit/robot_model/link_model.h>

#include "rosim_core/occ_to_vtk.h"

namespace rosim::core
{

namespace
{

/// Brand-ish color per link (matches the URDF material intent).
void linkColor(const std::string& name, double rgb[3])
{
  if (name == "base_link")
  {
    rgb[0] = 0.25; rgb[1] = 0.25; rgb[2] = 0.25;
  }
  else if (name == "rail_base_link")
  {
    rgb[0] = 0.34; rgb[1] = 0.36; rgb[2] = 0.39;  // track: steel
  }
  else if (name == "rail_carriage")
  {
    rgb[0] = 0.16; rgb[1] = 0.30; rgb[2] = 0.46;  // carriage: blue
  }
  else if (name.rfind("link", 0) == 0)
  {
    rgb[0] = 0.95; rgb[1] = 0.45; rgb[2] = 0.05;  // arm links: orange
  }
  else if (name == "tool_tip")
  {
    rgb[0] = 0.16; rgb[1] = 0.17; rgb[2] = 0.19;  // torch: graphite
  }
  else
  {
    rgb[0] = 0.30; rgb[1] = 0.30; rgb[2] = 0.32;  // wrist: graphite
  }
}

/// Build shape geometry in the shape's local frame, centered at origin.
vtkSmartPointer<vtkPolyDataAlgorithm> makeShapeSource(const shapes::Shape* shape)
{
  switch (shape->type)
  {
    case shapes::CYLINDER:
    {
      const auto* cyl = static_cast<const shapes::Cylinder*>(shape);
      auto source = vtkSmartPointer<vtkCylinderSource>::New();
      source->SetRadius(cyl->radius);
      source->SetHeight(cyl->length);
      source->SetResolution(32);
      source->SetCenter(0.0, 0.0, 0.0);
      // vtkCylinderSource axis is +Y; URDF/moveit cylinder axis is +Z.
      auto rotate = vtkSmartPointer<vtkTransform>::New();
      rotate->RotateX(-90.0);
      auto filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
      filter->SetInputConnection(source->GetOutputPort());
      filter->SetTransform(rotate);
      return filter;
    }
    case shapes::BOX:
    {
      const auto* box = static_cast<const shapes::Box*>(shape);
      auto source = vtkSmartPointer<vtkCubeSource>::New();
      source->SetXLength(box->size[0]);
      source->SetYLength(box->size[1]);
      source->SetZLength(box->size[2]);
      source->SetCenter(0.0, 0.0, 0.0);
      return source;
    }
    case shapes::SPHERE:
    {
      const auto* sph = static_cast<const shapes::Sphere*>(shape);
      auto source = vtkSmartPointer<vtkSphereSource>::New();
      source->SetRadius(sph->radius);
      source->SetThetaResolution(24);
      source->SetPhiResolution(24);
      return source;
    }
    case shapes::MESH:
    {
      // CAD link geometry (e.g. the GP12 STLs): copy the triangle soup into
      // MeshData, wrap as polydata, and let vtkPolyDataNormals compute
      // consistent per-vertex normals for smooth shading.
      const auto* mesh = static_cast<const shapes::Mesh*>(shape);
      MeshData data;
      data.vertices.assign(mesh->vertices, mesh->vertices + 3 * mesh->vertex_count);
      data.triangles.assign(mesh->triangles, mesh->triangles + 3 * mesh->triangle_count);
      auto normals = vtkSmartPointer<vtkPolyDataNormals>::New();
      normals->SetInputData(buildSolidPolyData(data));
      normals->ConsistencyOn();
      normals->SplittingOff();
      return normals;
    }
    default:
      return nullptr;
  }
}

}  // namespace

bool RobotLinkActors::init(const moveit::core::RobotModelPtr& model)
{
  model_ = model;
  links_.clear();
  shape_offsets_.clear();
  if (!model_)
    return false;

  for (const moveit::core::LinkModel* link : model_->getLinkModels())
  {
    if (!link)
      continue;
    const auto& shapes = link->getShapes();
    const auto& origins = link->getCollisionOriginTransforms();
    if (shapes.empty())
      continue;  // pure frames such as tool0 are not drawn

    for (size_t s = 0; s < shapes.size(); ++s)
    {
      auto source = makeShapeSource(shapes[s].get());
      if (!source)
      {
        RCLCPP_WARN(rclcpp::get_logger("rosim.viz"),
                    "RobotLinkActors: shape %zu on link '%s' not renderable (type %d), skipped",
                    s, link->getName().c_str(), static_cast<int>(shapes[s]->type));
        continue;
      }

      auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(source->GetOutputPort());

      auto actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);

      double rgb[3];
      linkColor(link->getName(), rgb);
      actor->GetProperty()->SetColor(rgb);
      actor->GetProperty()->SetInterpolationToPhong();
      actor->GetProperty()->SetSpecular(0.15);

      if (s < origins.size())
        shape_offsets_[actor.Get()] = origins[s];

      LinkVis vis;
      vis.link_name = link->getName();
      vis.actor = actor;
      links_.push_back(std::move(vis));
    }
  }
  return !links_.empty();
}

void RobotLinkActors::addToRenderer(vtkRenderer* renderer)
{
  for (auto& vis : links_)
    renderer->AddActor(vis.actor);
}

void RobotLinkActors::updateFromState(const moveit::core::RobotState& state)
{
  for (auto& vis : links_)
  {
    Eigen::Isometry3d pose = state.getGlobalLinkTransform(vis.link_name);
    auto it = shape_offsets_.find(vis.actor.Get());
    if (it != shape_offsets_.end())
      pose = pose * it->second;

    // Element-wise SetElement (not a flat[16] DeepCopy) — see eigenToVtkMatrix
    // for why the column-major pitfall matters. Position/Orientation stay at
    // identity defaults, so the actor transform equals the user matrix exactly.
    vis.actor->SetUserMatrix(eigenToVtkMatrix(pose));
  }
}

void RobotLinkActors::setAllVisible(bool visible)
{
  for (auto& vis : links_)
    vis.actor->SetVisibility(visible ? 1 : 0);
}

}  // namespace rosim::core
