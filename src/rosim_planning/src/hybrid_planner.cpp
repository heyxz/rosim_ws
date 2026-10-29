#include "rosim_planning/hybrid_planner.h"

#include "rosim_planning/joint_delta.h"

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rosim::planning
{

namespace
{
std::string shortNumber(double v)
{
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(2);
  ss << v;
  return ss.str();
}
}  // namespace

HybridPlanner::HybridPlanner(MotionPlannerPtr rl, MotionPlannerPtr ompl, double max_delta,
                             rclcpp::Logger logger)
  : rl_(std::move(rl)), ompl_(std::move(ompl)), max_delta_(max_delta), logger_(logger)
{
}

bool HybridPlanner::plan(const PlanRequest& req, PlanResult& result)
{
  last_note_.clear();

  const JointDelta move = computeJointDelta(req);
  const double delta = move.norm();
  RCLCPP_DEBUG(logger_, "gate: arm=%.3f rail=%.3f goal_n=%zu group=%s",
              move.arm, move.rail, req.goal_joint_positions.size(), req.group.c_str());
  if (delta > max_delta_)
  {
    last_note_ = "手臂位移 " + shortNumber(delta) + " rad 超出 RL 范围，整段 OMPL";
    return ompl_->plan(req, result);
  }

  PlanResult rl_result;
  if (!rl_ || !rl_->plan(req, rl_result) || !rl_result.success || !rl_result.trajectory)
  {
    last_note_ = "RL 未出解，整段 OMPL";
    return ompl_->plan(req, result);
  }

  // 逐点校验 RL 的路径：找到第一个碰撞点。
  const auto model = req.scene->getRobotModel();
  auto* group = model->getJointModelGroup(req.group);
  if (group == nullptr)
  {
    last_note_ = "缺少关节组，整段 OMPL";
    return ompl_->plan(req, result);
  }

  auto& rl_traj = *rl_result.trajectory;   // getWayPointPtr 在本版 MoveIt 里是非 const
  std::size_t n = rl_traj.getWayPointCount();
  std::size_t first_bad = n;          // n 表示"全部无碰撞"
  moveit::core::RobotState probe(req.start);
  for (std::size_t i = 0; i < n; ++i)
  {
    probe = *rl_traj.getWayPointPtr(i);
    if (req.scene->isStateColliding(probe, req.group, false))
    {
      first_bad = i;
      break;
    }
  }

  if (first_bad == n)
  {
    result = std::move(rl_result);
    last_note_ = "RL 全程无碰撞（" + std::to_string(n) + " 点）";
    return true;
  }

  // H3：保留 [0, first_bad) 这一段，从 last-good 状态让 OMPL 接到目标。
  const std::size_t keep = (first_bad == 0) ? 0 : first_bad - 1;
  if (keep == 0)
  {
    last_note_ = "RL 第一步就碰撞，整段 OMPL";
    return ompl_->plan(req, result);
  }

  PlanRequest sub(req.start);
  sub.start = *rl_traj.getWayPointPtr(keep);
  sub.goal_joint_positions = req.goal_joint_positions;
  sub.scene = req.scene;
  sub.group = req.group;
  sub.planner_id = req.planner_id;
  sub.allowed_planning_time = req.allowed_planning_time;
  sub.velocity_scaling = req.velocity_scaling;
  sub.acceleration_scaling = req.acceleration_scaling;

  PlanResult repaired;
  if (!ompl_->plan(sub, repaired) || !repaired.success || !repaired.trajectory)
  {
    last_note_ = "RL 前缀 " + std::to_string(keep + 1) + "/" + std::to_string(n) +
                 " 点，OMPL 修复失败，整段 OMPL";
    return ompl_->plan(req, result);
  }

  // 拼接：RL 前缀 + OMPL 修复段。用 append 保持前缀的时间参数化。
  auto merged = std::make_unique<robot_trajectory::RobotTrajectory>(model, req.group);
  for (std::size_t i = 0; i <= keep; ++i)
  {
    const auto& st = *rl_traj.getWayPointPtr(i);
    merged->addSuffixWayPoint(st, i == 0 ? 0.0 : rl_traj.getWayPointDurationFromPrevious(i));
  }
  merged->append(*repaired.trajectory, 0.0);

  result.success = true;
  result.trajectory = std::move(merged);
  last_note_ = "RL 前缀 " + std::to_string(keep + 1) + "/" + std::to_string(n) + " 点 + OMPL 修复";
  RCLCPP_INFO(logger_, "hybrid: %s", last_note_.c_str());
  return true;
}

}  // namespace rosim::planning
