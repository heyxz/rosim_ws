#pragma once

#include <string>
#include <utility>

#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

/// 起始状态到目标关节位置之间的位移度量。
struct JointDelta
{
  double arm{0.0};   ///< 旋转关节的欧氏范数（rad），不含导轨
  double rail{0.0};  ///< 导轨位移的绝对值（m）
  double norm() const { return arm; }
};

/// 计算 `req.start` → `req.goal_joint_positions` 的位移。
///
/// 两个坑（2026-10-29 修正）：
///   1) `goal_joint_positions` 来自 `copyJointGroupPositions`，**组顺序**；
///      而 `RobotState::getVariablePositions()` 是**模型顺序**，两者不一致
///      （模型里 rail_joint 排在 j1 之前）。按下标硬减会全部错位，
///      算出来的位移毫无意义，导致 H3 的 RL 分支几乎永不触发。
///   2) 导轨是移动关节（单位 m），不能和旋转关节（rad）混在同一个欧氏范数里。
///
/// 现在按 **活动关节名** 逐个匹配，并把移动关节单列到 `rail`。
JointDelta computeJointDelta(const PlanRequest& req);

}  // namespace rosim::planning
