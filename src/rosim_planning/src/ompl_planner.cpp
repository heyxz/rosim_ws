// OMPL planner backend. pluginlib-loads ompl_interface/OMPLPlanner in-process
// (no move_group), solves a joint-space PlanRequest against the FULL planning
// scene, and applies TOTG time parameterization before returning.

#include "rosim_planning/ompl_planner.h"

#include <sstream>

#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

namespace rosim::planning
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("rosim.ompl_planner");
}

OmplPlanner::OmplPlanner(const rclcpp::Node::SharedPtr& node,
                         const moveit::core::RobotModelPtr& model)
  : node_(node), model_(model)
{
  std::string plugin_name;
  if (!node_->get_parameter("planning_plugin", plugin_name))
  {
    // fall back to the known default rather than dying at startup
    plugin_name = "ompl_interface/OMPLPlanner";
    RCLCPP_WARN(LOGGER, "parameter 'planning_plugin' missing, defaulting to %s", plugin_name.c_str());
  }

  try
  {
    loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
        "moveit_core", "planning_interface::PlannerManager");
    planner_ = planning_interface::PlannerManagerPtr(
        loader_->createUnmanagedInstance(plugin_name));
    if (!planner_->initialize(model_, node_, node_->get_namespace()))
      throw std::runtime_error("failed to initialize planner " + plugin_name);
    RCLCPP_INFO(LOGGER, "loaded planning interface '%s'", planner_->getDescription().c_str());
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    std::stringstream avail;
    if (loader_)
      for (const auto& cls : loader_->getDeclaredClasses())
        avail << cls << " ";
    throw std::runtime_error(std::string("failed to load '") + plugin_name +
                             "': " + ex.what() + "\navailable: " + avail.str());
  }
}

bool OmplPlanner::plan(const PlanRequest& req, PlanResult& result)
{
  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(req.group);
  if (!group)
  {
    result.error = "unknown joint model group '" + req.group + "'";
    return false;
  }

  // ---- build the MotionPlanRequest --------------------------------------
  planning_interface::MotionPlanRequest mreq;
  planning_interface::MotionPlanResponse mres;

  mreq.group_name = req.group;
  mreq.planner_id = req.planner_id;
  mreq.allowed_planning_time = req.allowed_planning_time;
  mreq.start_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(req.start, mreq.start_state);

  moveit::core::RobotState goal_state(req.start);
  goal_state.setJointGroupPositions(group, req.goal_joint_positions);
  mreq.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal_state, group, 1e-4));

  // ---- solve -------------------------------------------------------------
  planning_interface::PlanningContextPtr context =
      planner_->getPlanningContext(req.scene, mreq, mres.error_code_);
  if (!context)
  {
    result.error = "no planning context (bad request or scene?)";
    return false;
  }

  context->solve(mres);
  if (mres.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    result.error = "OMPL solve failed, error code " + std::to_string(mres.error_code_.val);
    return false;
  }
  if (!mres.trajectory_ || mres.trajectory_->empty())
  {
    result.error = "OMPL returned an empty trajectory";
    return false;
  }

  // ---- time parameterization (TOTG) --------------------------------------
  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.002, 0.01, 0.001);
  if (!totg.computeTimeStamps(*mres.trajectory_, req.velocity_scaling, req.acceleration_scaling))
  {
    result.error = "time parameterization failed";
    return false;
  }

  result.trajectory = std::make_unique<robot_trajectory::RobotTrajectory>(*mres.trajectory_);
  result.success = true;
  return true;
}

void registerOmplPlanner()
{
  MotionPlannerFactory::registerCreator(
      "ompl", [](const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model) {
        return std::make_shared<OmplPlanner>(node, model);
      });
}

namespace
{
/// Self-registration so plain linkage of librosim_planning is enough.
[[maybe_unused]] const bool ompl_planner_registered = [] {
  registerOmplPlanner();
  return true;
}();
}  // namespace

}  // namespace rosim::planning
