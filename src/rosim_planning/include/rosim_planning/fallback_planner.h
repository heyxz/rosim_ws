#pragma once

#include <memory>
#include <string>

#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

/// \brief 用一个"快但不可靠"的规划器打头阵，失败或目标太远时回退到 OMPL。
///
/// 动机（实测见 docs/rl_bench.md）：RL 策略在小位移上比 OMPL 快 2–3 倍，
/// 但目标偏移 1.8 rad 时只有 40% 成功率。单独用 RL 会把失败直接抛给调用方。
///
/// 两条规则：
///  1. 目标偏移 > max_delta → 不试 RL，直接 OMPL（避免"先失败再回退"把
///     13 ms 拖到 92 ms）；
///  2. 试了 RL 失败 → 回退 OMPL，失败原因保留在 result.error 里。
///
/// 这是装饰器：焊缝跟随的接近/退刀段也走同一条 plan() 接口，因此规则对
/// 所有调用方一致，而不是只在 plan_joint 服务里特判。
class FallbackPlanner : public MotionPlanner
{
public:
  FallbackPlanner(MotionPlannerPtr primary, MotionPlannerPtr fallback, double max_delta,
                  rclcpp::Logger logger = rclcpp::get_logger("fallback_planner"));

  std::string name() const override;

  bool plan(const PlanRequest& req, PlanResult& result) override;

  /// 最近一次 plan() 是否发生了回退（供上层把原因写进消息/日志）。
  bool lastFallback() const
  {
    return last_fallback_;
  }

  /// 最近一次成功出解的规划器名（可能是 fallback 链里更深的一级）。
  const std::string& lastSuccessName() const
  {
    return last_success_;
  }

  std::string lastNote() const override
  {
    return last_note_;
  }

private:
  MotionPlannerPtr primary_;
  MotionPlannerPtr fallback_;
  double max_delta_ = 1.5;  ///< 六关节位移的欧氏范数上限 [rad]
  rclcpp::Logger logger_;
  bool last_fallback_ = false;
  std::string last_success_;
  std::string last_note_;
};

}  // namespace rosim::planning
