#pragma once

#include <memory>
#include <string>

#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

/// \brief 笛卡尔直线规划器：TCP 沿直线从起点走到目标，逐步 IK + 碰撞检查。
///
/// 为什么需要它：焊接的接近/退刀在工艺上就是"沿工具轴直线下降/抬起"。
/// 用直线插补比随机采样式规划器更符合工艺，而且**完全确定**——同样的起止
/// 位姿每次给出同一条路径，便于复现和调试。
///
/// 失败条件（任一即返回 false，交给 FallbackPlanner 回退 OMPL）：
///   - 某个采样点 IK 无解
///   - 某个采样点碰撞
///   - 相邻 IK 解跳变过大（肘部翻转，直线走不过去）
///
/// 时间参数化仍由 TOTG 按 URDF 限位完成，下游执行/可视化与其它规划器一致。
class LinePlanner : public MotionPlanner
{
public:
  LinePlanner(const rclcpp::Node::SharedPtr& node, moveit::core::RobotModelPtr model);

  std::string name() const override
  {
    return "line";
  }

  bool plan(const PlanRequest& req, PlanResult& result) override;

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  std::string tip_link_ = "tool_tip";
  double step_m_ = 0.003;      ///< 平移采样步长 [m]
  double step_rad_ = 0.02;     ///< 姿态采样步长 [rad]
  double max_jump_ = 0.35;     ///< 相邻 IK 解允许的最大关节跳变 [rad]
  double ik_timeout_ = 0.02;   ///< 单点 IK 超时 [s]
};

}  // namespace rosim::planning
