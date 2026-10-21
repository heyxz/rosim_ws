#include "rosim_planning/motion_planner.h"
#include "rosim_planning/trajectory_timing.h"

#include <chrono>
#include <future>
#include <mutex>
#include <thread>

#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rosim_interfaces/srv/rl_plan.hpp>

namespace rosim::rl
{

/// \brief `MotionPlanner` that delegates the rollout to the Python policy node.
///
/// Why a service instead of ONNX inside this process: the policy's observation
/// has to be built exactly as the training code builds it (normalisation stats,
/// clearance model, joint scaling). Reusing the training code verbatim in Python
/// removes a whole class of sim2sim bugs; C++ keeps the parts that must match
/// the real scene - the trajectory assembly and the FCL collision check.
class RlRemotePlanner : public planning::MotionPlanner
{
public:
  RlRemotePlanner(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model)
    : node_(node), model_(model)
  {
    // The response must be deliverable while plan() is blocked waiting for it.
    // A MutuallyExclusive group (the node default) would serialise the service
    // handler and the client response callback -> guaranteed timeout.
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_ = node_->create_client<rosim_interfaces::srv::RlPlan>(
        "/rl_plan", rmw_qos_profile_services_default, callback_group_);
  }

  std::string name() const override
  {
    return "rl";
  }

  bool plan(const planning::PlanRequest& req, planning::PlanResult& result) override
  {
    const auto* group = model_->getJointModelGroup(req.group);
    if (!group)
    {
      result.error = "unknown group: " + req.group;
      return false;
    }
    const auto& names = group->getVariableNames();

    // 策略的变量顺序固定为 [rail_joint, j1..j6]，但组的变量顺序不保证，
    // 所以一律按名字映射；6 维请求表示"只有手臂"，导轨保持起点值。
    const std::size_t n_goal = req.goal_joint_positions.size();
    const bool full_goal = n_goal == names.size();
    // 一律带导轨发给策略节点（节点再按自己的策略维度裁剪），
    // 这样 6 维/7 维策略都能用同一套 C++。
    const bool with_rail = true;
    if (!full_goal && n_goal != 6 && n_goal != 7)
    {
      result.error = "goal size does not match group (" + std::to_string(names.size()) + " vars)";
      return false;
    }

    std::string rail_name;
    for (const auto& n : names)
      if (n.find("rail") != std::string::npos)
        rail_name = n;

    std::vector<std::string> policy_names;   // 发给策略的顺序
    if (with_rail && !rail_name.empty())
      policy_names.push_back(rail_name);
    for (int j = 1; j <= 6; ++j)
      policy_names.push_back("j" + std::to_string(j));

    auto goal_of = [&](const std::string& name) -> double
    {
      if (full_goal)
      {
        for (std::size_t i = 0; i < names.size(); ++i)
          if (names[i] == name)
            return req.goal_joint_positions[i];
        return 0.0;
      }
      if (name == rail_name && n_goal == 7)
        return req.goal_joint_positions[0];
      if (name.rfind("j", 0) == 0 && name.size() == 2 && name[1] >= '1' && name[1] <= '6')
        return req.goal_joint_positions[with_rail ? (name[1] - '0') : (name[1] - '0' - 1)];
      return req.start.getVariablePosition(name);       // 导轨保持不动
    };

    std::vector<double> policy_start, policy_goal;
    for (const auto& n : policy_names)
    {
      policy_start.push_back(req.start.getVariablePosition(n));
      policy_goal.push_back(goal_of(n));
    }

    if (!client_->wait_for_service(std::chrono::seconds(2)))
    {
      result.error = "rl_plan service not available (start rl_plan_node.py)";
      return false;
    }

    auto request = std::make_shared<rosim_interfaces::srv::RlPlan::Request>();
    request->start = policy_start;
    request->goal = policy_goal;

    const auto t0 = std::chrono::steady_clock::now();
    auto future = client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      result.error = "rl_plan timed out";
      return false;
    }
    const auto response = future.get();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!response->success || response->path.empty())
    {
      result.error = "rl policy failed: " + response->message;
      return false;
    }

    // ---- assemble the trajectory -------------------------------------------
    auto trajectory = std::make_unique<robot_trajectory::RobotTrajectory>(model_, req.group);
    const std::size_t n_arm = policy_names.size();
    const std::size_t n_points = response->path.size() / n_arm;
    moveit::core::RobotState state(req.start);
    for (std::size_t k = 0; k < n_points; ++k)
    {
      for (std::size_t j = 0; j < n_arm; ++j)
        state.setVariablePosition(policy_names[j], response->path[k * n_arm + j]);

      // The policy never sees MoveIt's scene; FCL is the authority. Reject the
      // plan as soon as any waypoint collides instead of executing into it.
      if (req.scene && req.scene->isStateColliding(state, req.group))
      {
        result.error = "rl path collides with the scene at waypoint " + std::to_string(k);
        return false;
      }
      state.update();
      trajectory->addSuffixWayPoint(state, 0.0);
    }

    // Time-parameterise against the URDF limits. Without this the path has no
    // timing of its own and the uniform-grid clamp would imply velocities well
    // above the joint limits (10 policy steps x 0.1 rad in 200 ms = 5 rad/s on
    // j2, whose limit is 4.0).
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.1, 0.05);
    totg.computeTimeStamps(*trajectory, req.velocity_scaling, req.acceleration_scaling);
    // Then put it on the uniform grid the controller expects (splines + 100 Hz).
    planning::regularizeTiming(*trajectory, model_, node_->get_logger());

    RCLCPP_INFO(node_->get_logger(),
                "rl plan: %zu points, %.1f ms, min clearance %.0f mm",
                n_points, elapsed_ms, response->min_clearance * 1000.0);
    result.success = true;
    result.trajectory = std::move(trajectory);
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  rclcpp::Client<rosim_interfaces::srv::RlPlan>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace rosim::rl

namespace
{
/// Register "rl" when this library is dlopen()ed by the planning node.
struct RlRegistrar
{
  RlRegistrar()
  {
    rosim::planning::MotionPlannerFactory::registerCreator(
        "rl", [](const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model) {
          return std::make_shared<rosim::rl::RlRemotePlanner>(node, model);
        });
  }
};
const bool kRegistered = (RlRegistrar{}, true);
}  // namespace
