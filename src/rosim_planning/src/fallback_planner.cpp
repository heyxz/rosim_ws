#include "rosim_planning/fallback_planner.h"

#include <cmath>

namespace rosim::planning
{

namespace
{
/// 六关节位移的欧氏范数；目标维度不足时返回 0（交给 primary 自己判断）。
double jointDelta(const PlanRequest& req)
{
  if (req.goal_joint_positions.empty())
    return 0.0;

  const auto& model = req.start.getRobotModel();
  const auto* jmg = model->getJointModelGroup(req.group);
  if (!jmg)
    return 0.0;

  // 约定：goal[0..5] 对应 j1..j6（组里第 7 个变量是 rail_joint，顺序不保证）
  const auto& names = jmg->getActiveJointModelNames();
  double sum = 0.0;
  if (req.goal_joint_positions.size() == 6 && names.size() != 6)
  {
    for (std::size_t i = 0; i < 6; ++i)
    {
      const double d =
          req.goal_joint_positions[i] - req.start.getVariablePosition("j" + std::to_string(i + 1));
      sum += d * d;
    }
  }
  else
  {
    const std::size_t n =
        std::min<std::size_t>(req.goal_joint_positions.size(), names.size());
    for (std::size_t i = 0; i < n; ++i)
    {
      const double d = req.goal_joint_positions[i] - req.start.getVariablePosition(names[i]);
      sum += d * d;
    }
  }
  return std::sqrt(sum);
}
}  // namespace

FallbackPlanner::FallbackPlanner(MotionPlannerPtr primary, MotionPlannerPtr fallback,
                                 double max_delta, rclcpp::Logger logger)
  : primary_(std::move(primary)), fallback_(std::move(fallback)), max_delta_(max_delta),
    logger_(logger)
{
}

std::string FallbackPlanner::name() const
{
  return primary_ ? primary_->name() + "+fallback" : "fallback";
}

bool FallbackPlanner::plan(const PlanRequest& req, PlanResult& result)
{
  last_fallback_ = false;
  last_note_.clear();

  if (!primary_)
    return fallback_ && fallback_->plan(req, result);

  const double delta = jointDelta(req);
  if (fallback_ && delta > max_delta_)
  {
    last_fallback_ = true;
    last_note_ = "goal offset " + std::to_string(delta).substr(0, 4) + " rad > planner range";
    RCLCPP_INFO(logger_, "fallback: %s -> %s", last_note_.c_str(), fallback_->name().c_str());
    return fallback_->plan(req, result);
  }

  PlanResult primary_result;
  if (primary_->plan(req, primary_result) && primary_result.success)
  {
    result = std::move(primary_result);
    return true;
  }

  if (!fallback_)
  {
    result = std::move(primary_result);
    return false;
  }

  last_fallback_ = true;
  last_note_ = "primary failed (" + primary_result.error + "), fell back";
  RCLCPP_INFO(logger_, "fallback: %s -> %s", primary_result.error.c_str(), fallback_->name().c_str());
  PlanResult fb;
  if (fallback_->plan(req, fb) && fb.success)
  {
    result = std::move(fb);
    return true;
  }

  result = std::move(fb);
  result.error = primary_result.error + "; fallback failed: " + result.error;
  return false;
}

}  // namespace rosim::planning
