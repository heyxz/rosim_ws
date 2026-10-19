// RL MotionPlanner implementation: steps a TorchScript policy from the start
// state toward the joint goal, collision-checking every intermediate state.
// Without a loaded policy, plan() fails with a clear message so the rest of the
// stack (OMPL) stays usable.

#include "rosim_rl/rl_planner.h"

#include <cmath>
#include <limits>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#ifdef ROSIM_WITH_TORCH
#include <torch/script.h>
#endif

namespace rosim::rl
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("rosim.rl_planner");
}

RlPlanner::RlPlanner(const rclcpp::Node::SharedPtr& node,
                     const moveit::core::RobotModelPtr& model)
  : node_(node), model_(model)
{
  // Resolve the build flag outside the macro: preprocessor directives inside
  // macro arguments are not portable (GCC warns about it).
#ifdef ROSIM_WITH_TORCH
  const char* torch_state = "enabled";
#else
  const char* torch_state = "disabled";
#endif
  RCLCPP_INFO(LOGGER,
              "RlPlanner created (obs=%d act=%d, torch=%s). Load a policy via "
              "loadPolicy() before expecting plans.",
              Spaces::OBS_DIM, Spaces::ACT_DIM, torch_state);
}

bool RlPlanner::loadPolicy(const std::string& torchscript_path, std::string& reason)
{
#ifdef ROSIM_WITH_TORCH
  try
  {
    torch::jit::Module module = torch::jit::load(torchscript_path);
    (void)module;  // hold in a member once stepping is wired in
    policy_loaded_ = true;
    reason = "policy loaded from " + torchscript_path;
    return true;
  }
  catch (const c10::Error& ex)
  {
    reason = std::string("torchscript load failed: ") + ex.what();
    return false;
  }
#else
  (void)torchscript_path;
  reason = "built without ROSIM_WITH_TORCH — rebuild rosim_rl with "
           "-DROSIM_WITH_TORCH=ON (uses /opt/libtorch)";
  return false;
#endif
}

Eigen::VectorXd RlPlanner::observation(const moveit::core::RobotState& state,
                                       const std::vector<double>& goal_joints,
                                       const std::string& group,
                                       const planning_scene::PlanningScene& scene)
{
  Eigen::VectorXd obs(Spaces::OBS_DIM);
  obs.setZero();

  const moveit::core::JointModelGroup* jmg = state.getRobotModel()->getJointModelGroup(group);
  if (!jmg)
    return obs;

  // [0..5] current joints
  std::vector<double> joints;
  state.copyJointGroupPositions(jmg, joints);
  for (int i = 0; i < 6 && i < static_cast<int>(joints.size()); ++i)
    obs[i] = joints[i];

  // The 6-DOF policy contract is kept on the 7-DOF composite group: the first
  // six variables are the arm, the extra prismatic rail variable is held at
  // its current station (the policy does not command it).
  const size_t nvar = jmg->getVariableCount();
  std::vector<double> goal_full = goal_joints;
  if (goal_full.size() != nvar)
  {
    std::vector<double> current;
    state.copyJointGroupPositions(jmg, current);
    goal_full.resize(nvar);
    for (size_t i = 0; i < nvar; ++i)
      goal_full[i] = i < goal_joints.size() ? goal_joints[i] : current[i];
  }

  // [6..11] goal joints
  for (int i = 0; i < 6 && i < static_cast<int>(goal_full.size()); ++i)
    obs[6 + i] = goal_full[i];

  // [12..17] TCP pose (translation + log-orientation approximated by RPY)
  const Eigen::Isometry3d tf = state.getGlobalLinkTransform(state.getRobotModel()->getLinkModelNames().back());
  obs[12] = tf.translation().x();
  obs[13] = tf.translation().y();
  obs[14] = tf.translation().z();
  // small-angle euler is fine as an observation encoding
  const Eigen::Vector3d rpy = tf.linear().eulerAngles(0, 1, 2);
  obs[15] = rpy.x();
  obs[16] = rpy.y();
  obs[17] = rpy.z();

  // [18..20] direction to goal in TCP frame (goal joints FK - current TCP)
  moveit::core::RobotState goal_state(state);
  goal_state.setJointGroupPositions(jmg, goal_full);
  goal_state.updateLinkTransforms();
  const Eigen::Vector3d dir =
      (goal_state.getGlobalLinkTransform(state.getRobotModel()->getLinkModelNames().back()).translation()
       - tf.translation());
  obs[18] = dir.x();
  obs[19] = dir.y();
  obs[20] = dir.z();

  // [21] collision proximity: 1.0 = deepest penetration, 0.0 = free
  collision_detection::CollisionRequest creq;
  creq.distance = true;
  collision_detection::CollisionResult cres;
  scene.checkCollision(creq, cres, state);
  obs[21] = cres.collision ? 1.0 : std::min(1.0, std::max(0.0, -cres.distance));

  return obs;
}

bool RlPlanner::plan(const rosim::planning::PlanRequest& req,
                     rosim::planning::PlanResult& result)
{
  (void)req;  // consumed by the stepping loop once a policy is wired in

  if (!policy_loaded_)
  {
    result.error = "RL policy not loaded (see RlPlanner::loadPolicy)";
    return false;
  }

  // Intended stepping loop (wired once a policy exists):
  //   state = req.start
  //   loop:
  //     obs  = observation(state, req.goal_joint_positions, req.group, *req.scene)
  //     act  = policy->forward(obs) clamped to +/-Spaces::MAX_ACTION_RAD
  //     state.setVariablePositions(group, current + act)
  //     if scene->isStateColliding(state) -> terminal, plan fails
  //     if |goal - current| < eps -> success
  //   assemble states into RobotTrajectory + TOTG (same as OMPL branch)
  result.error = "RL policy stepping not implemented yet (skeleton)";
  return false;
}

namespace
{
[[maybe_unused]] const bool rl_planner_registered = [] {
  rosim::planning::MotionPlannerFactory::registerCreator(
      "rl", [](const rclcpp::Node::SharedPtr& node,
               const moveit::core::RobotModelPtr& model) {
        return std::make_shared<RlPlanner>(node, model);
      });
  return true;
}();
}  // namespace

}  // namespace rosim::rl
