#include "rosim_planning/line_planner.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "rosim_planning/trajectory_timing.h"

namespace rosim::planning
{

LinePlanner::LinePlanner(const rclcpp::Node::SharedPtr& node, moveit::core::RobotModelPtr model)
  : node_(node), model_(std::move(model))
{
}

bool LinePlanner::plan(const PlanRequest& req, PlanResult& result)
{
  const auto* jmg = model_->getJointModelGroup(req.group);
  if (!jmg)
  {
    result.error = "unknown group " + req.group;
    return false;
  }

  // kinematics.yaml 只给 6 轴 "arm" 配了 KDL 求解器；"manipulator"（arm + 导轨）
  // 是 OMPL 用的复合组，没有 IK。直线插补时把导轨固定在起点值，用 arm 组求 IK。
  const moveit::core::JointModelGroup* ik_group = jmg;
  if (!jmg->getSolverInstance())
  {
    ik_group = model_->getJointModelGroup("arm");
    if (!ik_group || !ik_group->getSolverInstance())
    {
      result.error = "line: no IK solver for group " + req.group;
      return false;
    }
  }

  moveit::core::RobotState start(req.start);
  start.update();

  // Goal state: 只覆盖请求里给出的关节，其余（导轨）保持起点值。
  moveit::core::RobotState goal(start);
  const auto& names = jmg->getActiveJointModelNames();
  if (req.goal_joint_positions.empty())
  {
    result.error = "line planner needs a joint goal";
    return false;
  }
  // 约定：goal_joint_positions[0..5] 是 j1..j6（组里还有第 7 个变量 rail_joint，
  // 不能按组内索引对齐，否则整条直线会偏到另一个位姿上）。
  if (req.goal_joint_positions.size() == 6 && names.size() != 6)
  {
    for (std::size_t i = 0; i < 6; ++i)
      goal.setVariablePosition("j" + std::to_string(i + 1), req.goal_joint_positions[i]);
  }
  else
  {
    const std::size_t m = std::min(req.goal_joint_positions.size(), names.size());
    for (std::size_t i = 0; i < m; ++i)
      goal.setVariablePosition(names[i], req.goal_joint_positions[i]);
  }
  goal.update();

  const Eigen::Isometry3d t0 = start.getGlobalLinkTransform(tip_link_);
  const Eigen::Isometry3d t1 = goal.getGlobalLinkTransform(tip_link_);

  const Eigen::Vector3d p0 = t0.translation(), p1 = t1.translation();
  const double dist = (p1 - p0).norm();
  const Eigen::Quaterniond q0(t0.linear()), q1(t1.linear());
  const double angle = q0.angularDistance(q1);

  const int steps = std::max(2, static_cast<int>(std::ceil(
                                    std::max(dist / step_m_, angle / step_rad_))) + 1);

  auto trajectory = std::make_unique<robot_trajectory::RobotTrajectory>(model_, req.group);

  moveit::core::RobotState seed(start);
  std::vector<double> prev_q;
  start.copyJointGroupPositions(jmg, prev_q);

  // 起点先入轨迹（时间戳统一为 0，后面 TOTG 再算）
  trajectory->addSuffixWayPoint(start, 0.0);

  for (int k = 1; k <= steps; ++k)
  {
    const double s = static_cast<double>(k) / steps;
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = p0 + s * (p1 - p0);
    target.linear() = q0.slerp(s, q1).toRotationMatrix();

    if (!seed.setFromIK(ik_group, target, tip_link_, std::vector<double>{}, ik_timeout_))
    {
      result.error = "line: IK failed at " + std::to_string(k) + "/" + std::to_string(steps) +
                     " (tcp " + std::to_string(target.translation().x()) + ", " +
                     std::to_string(target.translation().y()) + ", " +
                     std::to_string(target.translation().z()) + ")";
      return false;
    }
    seed.update();

    std::vector<double> q;
    seed.copyJointGroupPositions(jmg, q);
    double jump = 0.0;
    for (std::size_t i = 0; i < q.size() && i < prev_q.size(); ++i)
      jump = std::max(jump, std::abs(q[i] - prev_q[i]));
    if (jump > max_jump_)
    {
      result.error = "line: joint jump " + std::to_string(jump).substr(0, 5) + " rad at step " +
                     std::to_string(k) + " (IK branch flip)";
      return false;
    }
    prev_q = q;

    if (req.scene && req.scene->isStateColliding(seed, req.group))
    {
      result.error = "line: collision at step " + std::to_string(k) + "/" +
                     std::to_string(steps);
      return false;
    }

    trajectory->addSuffixWayPoint(seed, 0.0);
  }

  // 目标点用请求里的关节值收尾，避免 IK 在末端留下毫米级残差
  trajectory->addSuffixWayPoint(goal, 0.0);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.1, 0.05);
  if (!totg.computeTimeStamps(*trajectory, req.velocity_scaling, req.acceleration_scaling))
  {
    result.error = "line: time parameterisation failed";
    return false;
  }
  regularizeTiming(*trajectory, model_, node_->get_logger());

  RCLCPP_INFO(node_->get_logger(), "line plan: %zu points, %.1f mm, %.2f rad (no fallback)",
              trajectory->size(), dist * 1000.0, angle);
  result.success = true;
  result.trajectory = std::move(trajectory);
  return true;
}

namespace
{
const bool line_registered = [] {
  MotionPlannerFactory::registerCreator(
      "line", [](const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model) {
        return std::make_shared<LinePlanner>(node, model);
      });
  return true;
}();
}  // namespace

}  // namespace rosim::planning
