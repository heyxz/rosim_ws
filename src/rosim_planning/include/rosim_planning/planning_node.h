#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_interfaces/action/follow_edge.hpp"
#include "rosim_interfaces/action/follow_seams.hpp"
#include "rosim_interfaces/srv/create_seam.hpp"
#include "rosim_interfaces/srv/go_home.hpp"
#include "rosim_interfaces/srv/import_model.hpp"
#include "rosim_interfaces/srv/jog_cartesian.hpp"
#include "rosim_interfaces/srv/jog_joint.hpp"
#include "rosim_interfaces/srv/move_to_pose.hpp"
#include "rosim_interfaces/srv/plan_joint.hpp"
#include "rosim_interfaces/srv/move_workpiece.hpp"
#include "rosim_interfaces/srv/set_planner.hpp"
#include "rosim_interfaces/srv/set_speed_scale.hpp"
#include "rosim_planning/edge_follower.h"
#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

using FollowEdge = rosim_interfaces::action::FollowEdge;
using FollowSeams = rosim_interfaces::action::FollowSeams;
using GoalHandleFollowEdge = rclcpp_action::ServerGoalHandle<FollowEdge>;
using GoalHandleFollowSeams = rclcpp_action::ServerGoalHandle<FollowSeams>;

/// \brief The rosim planning server node.
///
/// Responsibilities:
///  - load the robot model (URDF + SRDF + kinematics.yaml params)
///  - own the local PlanningScene; ImportModel injects workpiece collision meshes
///  - expose FollowEdge action (edge-following pipeline via EdgeFollower)
///  - expose MoveToPose service (minimal OMPL loop / debugging)
///  - execute trajectories by publishing to the JTC topic; monitor /joint_states
///  - publish /planning_scene and /display_planned_path for RViz
class PlanningNode : public rclcpp::Node
{
public:
  explicit PlanningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~PlanningNode() override;

  /// \brief Two-phase init: call once the node is owned by a shared_ptr
  /// (constructors may not call shared_from_this).
  void init();

private:
  // ---- infrastructure ----------------------------------------------------
  void loadRobotModel();
  void setupInterfaces();
  /// Push ROS params (ik_threads) into the EdgeFollower config.
  void applyFollowerConfig();
  std::string nextModelId() const;

  // ---- service callbacks ---------------------------------------------------
  void handleImportModel(const rosim_interfaces::srv::ImportModel::Request::SharedPtr req,
                         const rosim_interfaces::srv::ImportModel::Response::SharedPtr res);
  void handleSetSpeedScale(const rosim_interfaces::srv::SetSpeedScale::Request::SharedPtr req,
                           const rosim_interfaces::srv::SetSpeedScale::Response::SharedPtr res);
  void handleSetPlanner(const rosim_interfaces::srv::SetPlanner::Request::SharedPtr req,
                        const rosim_interfaces::srv::SetPlanner::Response::SharedPtr res);
  void handleMoveToPose(const rosim_interfaces::srv::MoveToPose::Request::SharedPtr req,
                        const rosim_interfaces::srv::MoveToPose::Response::SharedPtr res);
  void handleMoveWorkpiece(const rosim_interfaces::srv::MoveWorkpiece::Request::SharedPtr req,
                           const rosim_interfaces::srv::MoveWorkpiece::Response::SharedPtr res);
  void handleGoHome(const rosim_interfaces::srv::GoHome::Request::SharedPtr req,
                    const rosim_interfaces::srv::GoHome::Response::SharedPtr res);
  void handleCreateSeam(const rosim_interfaces::srv::CreateSeam::Request::SharedPtr req,
                        const rosim_interfaces::srv::CreateSeam::Response::SharedPtr res);
  void handleJogJoint(const rosim_interfaces::srv::JogJoint::Request::SharedPtr req,
                      const rosim_interfaces::srv::JogJoint::Response::SharedPtr res);
  void handleJogCartesian(const rosim_interfaces::srv::JogCartesian::Request::SharedPtr req,
                          const rosim_interfaces::srv::JogCartesian::Response::SharedPtr res);

  // ---- action server -------------------------------------------------------
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const FollowEdge::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleFollowEdge> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleFollowEdge> goal_handle);
  void executeFollowEdge(const std::shared_ptr<GoalHandleFollowEdge> goal_handle);

  // multi-seam batch (one continuous trajectory)
  rclcpp_action::GoalResponse handleSeamsGoal(const rclcpp_action::GoalUUID& uuid,
                                              std::shared_ptr<const FollowSeams::Goal> goal);
  rclcpp_action::CancelResponse handleSeamsCancel(
      const std::shared_ptr<GoalHandleFollowSeams> goal_handle);
  void handleSeamsAccepted(const std::shared_ptr<GoalHandleFollowSeams> goal_handle);
  void executeFollowSeams(const std::shared_ptr<GoalHandleFollowSeams> goal_handle);

  /// Block until the robot reaches the trajectory end (or cancel/timeout).
  /// Publishes progress feedback. Returns false on cancel/timeout.
  bool monitorExecution(const std::shared_ptr<GoalHandleFollowEdge>& goal_handle,
                        const trajectory_msgs::msg::JointTrajectory& trajectory);

  /// Wait until the controller settles on the trajectory's final waypoint.
  /// Generic form used by both single-edge and multi-seam execution.
  bool monitorTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                         const std::function<bool()>& canceled,
                         const std::function<void(float)>& progress);

  // ---- execution -----------------------------------------------------------
  void publishTrajectory(const robot_trajectory::RobotTrajectory& trajectory,
                         bool publish_display);

  /// Re-time a trajectory with a jerk-limited (double-S) profile.
  bool applyScurve(robot_trajectory::RobotTrajectory& trajectory);

  moveit::core::RobotState snapshotCurrentState();

  // ---- members -------------------------------------------------------------
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  MotionPlannerPtr planner_;
  // 测试/基准入口：用当前激活的规划器规划到关节目标（RL 接入后可直接验证）
  rclcpp::Service<rosim_interfaces::srv::PlanJoint>::SharedPtr plan_joint_srv_;
  void handlePlanJoint(const rosim_interfaces::srv::PlanJoint::Request::SharedPtr req,
                       const rosim_interfaces::srv::PlanJoint::Response::SharedPtr res);
  std::unique_ptr<EdgeFollower> follower_;
  std::string group_ = "manipulator";
  std::string active_planner_ = "ompl";
  // RL 失败时自动回退 OMPL：RL 在目标偏移大时会失败，作为"快速但不可靠"的
  // 前端规划器，失败必须有人兜底，否则调用方拿到硬失败。
  // ROSIM_RL_NO_FALLBACK=1 关闭（基准测试用，才能测到纯 RL 的成功率）。
  MotionPlannerPtr fallback_planner_;
  bool allow_fallback_ = true;
  // RL 的成功率随目标偏移明显下降（0.7 rad 100%、1.2 rad 80%、1.8 rad 40%，
  // 实测见 .scratch/rl_bench.md）。超过这个偏移就直接走 OMPL，避免"先失败
  // 再回退"把耗时从 13 ms 拉到 92 ms。
  double rl_max_delta_ = 1.5;
  double speed_scale_ = 1.0;
  double jerk_limit_ = 100.0;  ///< default jerk limit for the S-curve pass [rad/s^3]

  std::map<std::string, std::shared_ptr<rosim::core::WorkpieceModel>> workpieces_;
  uint32_t model_counter_ = 0;

  moveit::core::RobotStatePtr current_state_;
  bool have_state_ = false;
  std::mutex state_mutex_;

  std::atomic<bool> cancel_{ false };
  std::atomic<bool> busy_{ false };
  std::thread worker_;
  std::mutex worker_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  rclcpp::Service<rosim_interfaces::srv::ImportModel>::SharedPtr import_model_srv_;
  rclcpp::Service<rosim_interfaces::srv::SetSpeedScale>::SharedPtr set_speed_srv_;
  rclcpp::Service<rosim_interfaces::srv::SetPlanner>::SharedPtr set_planner_srv_;
  rclcpp::Service<rosim_interfaces::srv::MoveToPose>::SharedPtr move_to_pose_srv_;
  rclcpp::Service<rosim_interfaces::srv::MoveWorkpiece>::SharedPtr move_workpiece_srv_;
  rclcpp::Service<rosim_interfaces::srv::GoHome>::SharedPtr go_home_srv_;
  rclcpp::Service<rosim_interfaces::srv::JogJoint>::SharedPtr jog_joint_srv_;
  rclcpp::Service<rosim_interfaces::srv::CreateSeam>::SharedPtr create_seam_srv_;
  rclcpp::Service<rosim_interfaces::srv::JogCartesian>::SharedPtr jog_cartesian_srv_;

  rclcpp_action::Server<FollowEdge>::SharedPtr follow_edge_action_;
  rclcpp_action::Server<FollowSeams>::SharedPtr follow_seams_action_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
};

}  // namespace rosim::planning
