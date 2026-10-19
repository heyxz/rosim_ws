#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace rosim::core
{

/// \brief Renders the robot in VTK from a moveit RobotModel and updates link
/// poses from a RobotState (FK). Geometry comes from the URDF collision
/// shapes (primitives; mesh shapes are skipped with a warning).
class RobotLinkActors
{
public:
  /// Build actors for every link with collision geometry.
  bool init(const moveit::core::RobotModelPtr& model);

  void addToRenderer(vtkRenderer* renderer);

  /// Push link transforms from the state (call state.updateLinkTransforms()
  /// beforehand). Pose = global link transform * shape origin, applied via
  /// SetUserMatrix (exact, no Euler reconstruction).
  void updateFromState(const moveit::core::RobotState& state);

  void setAllVisible(bool visible);

  const moveit::core::RobotModelPtr& model() const { return model_; }

private:
  struct LinkVis
  {
    std::string link_name;
    vtkSmartPointer<vtkActor> actor;
  };

  moveit::core::RobotModelPtr model_;
  std::vector<LinkVis> links_;
  /// Per-actor URDF collision origin offset inside the link frame.
  std::map<vtkActor*, Eigen::Isometry3d> shape_offsets_;
};

}  // namespace rosim::core
