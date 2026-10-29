#include "rosim_planning/fallback_planner.h"

#include "rosim_planning/joint_delta.h"

#include <cmath>

namespace rosim::planning
{

namespace
{
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
  last_success_.clear();

  if (!primary_)
    return fallback_ && fallback_->plan(req, result);

  const JointDelta move = computeJointDelta(req);
  const double delta = move.norm();
  if (fallback_ && delta > max_delta_)
  {
    last_fallback_ = true;
    last_note_ = "手臂位移 " + std::to_string(delta).substr(0, 4) + " rad 超出 RL 范围";
    RCLCPP_INFO(logger_, "fallback: %s -> %s", last_note_.c_str(), fallback_->name().c_str());
    const bool ok = fallback_->plan(req, result);
    if (ok)
      last_success_ = fallback_->name();
    return ok;
  }

  PlanResult primary_result;
  if (primary_->plan(req, primary_result) && primary_result.success)
  {
    result = std::move(primary_result);
    last_success_ = primary_->name();
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
    last_success_ = fallback_->name();
    // 回退链可能是嵌套的：直接问更深一级
    if (auto* nested = dynamic_cast<const FallbackPlanner*>(fallback_.get()))
      if (!nested->lastSuccessName().empty())
        last_success_ = nested->lastSuccessName();
    return true;
  }

  result = std::move(fb);
  result.error = primary_result.error + "; fallback failed: " + result.error;
  return false;
}

}  // namespace rosim::planning
