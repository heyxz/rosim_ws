#pragma once

#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>

#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

/// \brief MotionPlanner backed by the moveit OMPL planning pipeline, embedded
/// in-process (no move_group node), following the pattern of MoveIt's
/// motion_planning_api tutorial:
///   pluginlib ClassLoader(PlannerManager) -> ompl_interface/OMPLPlanner
///   -> initialize(robot_model, node, ns) -> getPlanningContext(scene, req)
///   -> solve -> TOTG time parameterization.
class OmplPlanner : public MotionPlanner
{
public:
  OmplPlanner(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model);

  std::string name() const override { return "ompl"; }

  bool plan(const PlanRequest& req, PlanResult& result) override;

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> loader_;
  planning_interface::PlannerManagerPtr planner_;
};

/// Registers "ompl" in the factory (called from this library's static init).
void registerOmplPlanner();

}  // namespace rosim::planning
