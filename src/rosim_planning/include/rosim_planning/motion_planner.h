#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>

namespace rosim::planning
{

/// \brief A single free-space motion plan request (joint space goal).
struct PlanRequest
{
  explicit PlanRequest(const moveit::core::RobotState& start_state) : start(start_state)
  {
  }

  moveit::core::RobotState start;                                 ///< current state
  std::vector<double> goal_joint_positions;                       ///< group joint goal
  planning_scene::PlanningSceneConstPtr scene;                    ///< scene for collision checks
  std::string group = "manipulator";
  std::string planner_id = "RRTConnectkConfigDefault";
  double allowed_planning_time = 5.0;
  double velocity_scaling = 1.0;
  double acceleration_scaling = 1.0;
};

/// \brief Result of one plan() call. The trajectory (if success) is complete
/// with time parameterization and ready to be executed downstream.
struct PlanResult
{
  bool success = false;
  std::string error;
  std::unique_ptr<robot_trajectory::RobotTrajectory> trajectory;
};

/// \brief Abstract motion planner — the seam between OMPL today and RL tomorrow.
///
/// Implementations get the full planning scene (robot model + world collisions)
/// and produce a timed joint trajectory. Downstream execution/visualization is
/// identical for every implementation, so swapping "ompl" for "rl" requires no
/// changes anywhere else.
class MotionPlanner
{
public:
  virtual ~MotionPlanner() = default;

  virtual std::string name() const = 0;

  /// \brief Plan from req.start to the joint goal, collision-free, and apply
  /// time parameterization. May take seconds; call from a worker thread.
  virtual bool plan(const PlanRequest& req, PlanResult& result) = 0;

  /// 上一次 plan() 的判定理由（可为空）。装饰器会返回更具体的原因，
  /// 让调用方（焊缝跟随、plan_joint）不必 dynamic_cast 到具体类型。
  virtual std::string lastNote() const { return std::string(); }
};

using MotionPlannerPtr = std::shared_ptr<MotionPlanner>;

/// \brief String-keyed registry. Built-in "ompl" registers itself from this
/// library; rosim_rl registers "rl" at static-init when linked in.
/// A pluginlib upgrade later is possible without touching call sites.
class MotionPlannerFactory
{
public:
  using Creator = std::function<MotionPlannerPtr(const rclcpp::Node::SharedPtr&,
                                                 const moveit::core::RobotModelPtr&)>;

  static void registerCreator(const std::string& type, Creator creator);
  static MotionPlannerPtr create(const std::string& type,
                                 const rclcpp::Node::SharedPtr& node,
                                 const moveit::core::RobotModelPtr& model);
  static std::vector<std::string> registeredTypes();

private:
  static std::map<std::string, Creator>& registry();
};

}  // namespace rosim::planning
