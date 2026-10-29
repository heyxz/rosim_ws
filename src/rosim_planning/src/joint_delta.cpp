#include "rosim_planning/joint_delta.h"

#include <algorithm>
#include <cmath>

namespace rosim::planning
{

JointDelta computeJointDelta(const PlanRequest& req)
{
  JointDelta delta;
  if (req.goal_joint_positions.empty())
    return delta;

  const auto& model = req.start.getRobotModel();
  const auto* jmg = model->getJointModelGroup(req.group);
  if (!jmg)
    return delta;

  const auto& names = jmg->getActiveJointModelNames();
  const std::size_t n = std::min(req.goal_joint_positions.size(), names.size());

  double arm_sum = 0.0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto* joint = model->getJointModel(names[i]);
    const double d = req.goal_joint_positions[i] - req.start.getVariablePosition(names[i]);
    if (joint != nullptr && joint->getType() == moveit::core::JointModel::PRISMATIC)
      delta.rail = std::max(delta.rail, std::fabs(d));
    else
      arm_sum += d * d;
  }
  delta.arm = std::sqrt(arm_sum);
  return delta;
}

}  // namespace rosim::planning
