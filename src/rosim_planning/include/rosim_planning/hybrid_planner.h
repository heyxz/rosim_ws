#pragma once

#include <string>

#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

/// \brief H3 混合：RL 出初解 → 逐点校验 → 保留无碰撞前缀，用 OMPL 修掉后半段。
///
/// 与 FallbackPlanner 的区别：回退是把 RL 的整条路径**扔掉**重新规划；这里
/// 只在"RL 已经走通的那一段"之后接手，所以 RL 的部分成果会被保留。
///
/// 四种结果（都写进 lastNote()，便于在界面上说明用的是谁）：
///   1. 位移超过 RL 范围      → 整段 OMPL
///   2. RL 失败               → 整段 OMPL
///   3. RL 成功且全程无碰撞    → 直接用 RL（最快）
///   4. RL 成功但有碰撞        → RL 前缀 + OMPL 修复；修复失败再整段 OMPL
class HybridPlanner : public MotionPlanner
{
public:
  HybridPlanner(MotionPlannerPtr rl, MotionPlannerPtr ompl, double max_delta,
                rclcpp::Logger logger = rclcpp::get_logger("hybrid_planner"));

  std::string name() const override
  {
    return "hybrid";
  }

  bool plan(const PlanRequest& req, PlanResult& result) override;

  /// 最近一次 plan() 的说明（用哪个规划器的哪一段）。
  const std::string& lastNote() const
  {
    return last_note_;
  }

private:
  MotionPlannerPtr rl_;
  MotionPlannerPtr ompl_;
  double max_delta_;
  mutable std::string last_note_;
  rclcpp::Logger logger_;
};

}  // namespace rosim::planning
