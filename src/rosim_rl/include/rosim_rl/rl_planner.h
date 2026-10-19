#pragma once

#include <functional>
#include <string>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "rosim_planning/motion_planner.h"

namespace rosim::rl
{

/// \brief Reinforcement-learning MotionPlanner — the extension seam requested
/// for future RL-based collision-free path decisions.
///
/// The contract mirrors a standard RL loop:
///   observation = joint state (6) + goal joints (6) + TCP pose (6)
///                 + edge tangent/dir (3) + nearest collision distance (1)  -> 22
///   action      = joint increment (6), clamped to +/-0.1 rad
///
/// plan() steps the policy from req.start toward req.goal_joint_positions,
/// collision-checking every state against the scene. Surviving states are
/// assembled into a RobotTrajectory and time-parameterized, so downstream
/// execution/visualization is identical to the OMPL path.
///
/// Before a policy is loaded, plan() fails fast with a clear message and
/// SetPlanner("rl") reports the same — the rest of the stack stays usable.
class RlPlanner : public rosim::planning::MotionPlanner
{
public:
  /// Observation/action space contract shared with the Python gym env.
  struct Spaces
  {
    static constexpr int OBS_DIM = 22;
    static constexpr int ACT_DIM = 6;
    static constexpr double MAX_ACTION_RAD = 0.1;
  };

  /// Reward signature for training-time evaluation (progress delta available
  /// via scene/robot state distance to goal; collision flag derivable).
  using RewardFn = std::function<double(const moveit::core::RobotState& state,
                                        const planning_scene::PlanningScene& scene,
                                        double progress_delta)>;

  explicit RlPlanner(const rclcpp::Node::SharedPtr& node,
                     const moveit::core::RobotModelPtr& model);

  std::string name() const override { return "rl"; }

  bool plan(const rosim::planning::PlanRequest& req,
            rosim::planning::PlanResult& result) override;

  /// Load a TorchScript policy. Requires -DROSIM_WITH_TORCH=ON at build time;
  /// otherwise always returns false with `reason` set.
  bool loadPolicy(const std::string& torchscript_path, std::string& reason);

  bool policyLoaded() const { return policy_loaded_; }

  /// Training hook: override the default reward
  ///   r = +1.0 * progress_delta - 100.0 * collision - 0.01 * |dq|
  void setRewardFn(RewardFn fn) { reward_fn_ = std::move(fn); }

  /// Builds the 22-dim observation for a state/goal pair (exposed for the
  /// future gym bridge services; pure, no ROS).
  static Eigen::VectorXd observation(const moveit::core::RobotState& state,
                                     const std::vector<double>& goal_joints,
                                     const std::string& group,
                                     const planning_scene::PlanningScene& scene);

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  bool policy_loaded_ = false;
  RewardFn reward_fn_;  ///< empty -> default reward
};

}  // namespace rosim::rl
