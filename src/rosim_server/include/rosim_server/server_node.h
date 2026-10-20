#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <nlohmann/json.hpp>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_core/seam_builder.h"

#include <rosim_interfaces/action/follow_edge.hpp>
#include <rosim_interfaces/action/follow_seams.hpp>
#include <rosim_interfaces/srv/create_seam.hpp>
#include <rosim_interfaces/srv/go_home.hpp>
#include <rosim_interfaces/srv/import_model.hpp>
#include <rosim_interfaces/srv/jog_cartesian.hpp>
#include <rosim_interfaces/srv/jog_joint.hpp>
#include <rosim_interfaces/srv/move_to_pose.hpp>
#include <rosim_interfaces/srv/move_workpiece.hpp>
#include <rosim_interfaces/srv/set_planner.hpp>
#include <rosim_interfaces/srv/set_speed_scale.hpp>

namespace rosim::server
{

class FrameRenderer;
class WsServer;

using FollowEdge = rosim_interfaces::action::FollowEdge;
using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowEdge>;

/// \brief The accelerator-card server node: owns the headless renderer and the
/// WebSocket endpoint, and bridges the browser UI to the rosim_planning node's
/// services/action. The browser is a thin client — this node does geometry
/// (OCC import/tessellation for display), rendering and all business logic.
class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ServerNode() override;

  /// Load the robot model, renderer, WebSocket server, ROS clients and start
  /// the render thread. Call once after construction.
  void init();

private:
  // ---- setup ----------------------------------------------------------------
  void loadRobotModel();
  void setupClients();
  void setupWebServer();

  // ---- WebSocket dispatch ---------------------------------------------------
  void onWsConnect();
  void onWsMessage(const std::string& text, std::function<void(bool)> set_binary,
                   std::function<void(bool)> set_scene,
                   std::function<void(bool)> set_workpiece);

  // ---- control handlers (run on the WS I/O thread; ROS calls are async) -----
  void handleImport(const nlohmann::json& j);
  void handleListDir(const nlohmann::json& j);
  void handleSelectEdge(const nlohmann::json& j);
  void handleCreateSeam(const nlohmann::json& j);
  void handleDeleteSeam(const nlohmann::json& j);
  void handleRenameSeam(const nlohmann::json& j);
  void handleReorderSeams(const nlohmann::json& j);
  void handleSelectSeam(const nlohmann::json& j);
  void handleFollowSeams(const nlohmann::json& j);
  void handleFollow(const nlohmann::json& j);
  void handleStop(const nlohmann::json& j);
  void handleGoHome(const nlohmann::json& j);
  void handleMoveToPose(const nlohmann::json& j);
  void handleSetWorkpiecePose(const nlohmann::json& j);
  void handleJogJoint(const nlohmann::json& j);
  void handleJogCartesian(const nlohmann::json& j);

  /// Turn an executed trajectory into the TCP polyline the UI draws.
  void publishTcpPath(const trajectory_msgs::msg::JointTrajectory& traj);
  nlohmann::json tcpPathJsonLocked() const;
  void broadcastTcpPathLocked();
  /// Re-broadcast the live trace tagged as final, so the viewer can replace the
  /// cheap live polyline with the shaded tube once the run is over.
  void broadcastTcpPathFinalLocked();
  void handleSetSpeed(const nlohmann::json& j);
  void handleSetPlanner(const nlohmann::json& j);
  void handleGetState(const nlohmann::json& j);

  // ---- helpers --------------------------------------------------------------
  void broadcastJson(nlohmann::json j);
  void sendError(const std::string& message);
  void broadcastEdgesLocked();
  struct Seam;  // defined below with the rest of the application state
  void broadcastSeamsLocked();
  nlohmann::json seamsJsonLocked();
  void saveSeamsLocked();
  void loadSeamsLocked();
  bool seamMatchesModelLocked(const Seam& seam) const;
  void broadcastStateLocked();
  nlohmann::json stateJsonLocked();
  void highlightSelectedEdge();

  // ---- WebGL geometry serialization (debug viewer) -------------------------
  void buildRobotScene();
  void broadcastScene();
  void broadcastWorkpiece();
  nlohmann::json workpieceJson();
  nlohmann::json robotStateJson();
  geometry_msgs::msg::Pose poseToMsg(const Eigen::Isometry3d& pose) const;
  Eigen::Isometry3d poseFromMsg(const geometry_msgs::msg::Pose& pose) const;
  Eigen::Isometry3d parseWorldPose(const nlohmann::json& j) const;

  // ---- render loop (dedicated thread) ---------------------------------------
  void renderLoop();
  void applyPendingJointState();

  // ---- parameters -----------------------------------------------------------
  int width_ = 1280;
  int height_ = 720;
  int fps_ = 12;
  int quality_ = 70;
  std::string ws_address_ = "0.0.0.0";
  int ws_port_ = 8765;
  std::string web_root_;

  // ---- rendering / web ------------------------------------------------------
  std::unique_ptr<FrameRenderer> renderer_;
  std::unique_ptr<WsServer> ws_;
  std::thread render_thread_;
  std::atomic<bool> running_{ true };
  std::mutex render_mutex_;

  // ---- robot model / display state ------------------------------------------
  moveit::core::RobotModelPtr model_;
  moveit::core::RobotStatePtr display_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  // RL 演示通道（方案 B）：rl_demo_node 发布策略规划的关节路径，直接复用
  // publishTcpPath 画出来。transient_local 是为了让服务端后启动也能拿到。
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr rl_demo_sub_;
  std::mutex joint_mutex_;
  sensor_msgs::msg::JointState::SharedPtr pending_joint_;

  // ---- application state (guarded by state_mutex_) --------------------------
  struct EdgeSummary
  {
    uint32_t index;
    double length;
    bool closed;
  };

  /// One user-defined weld seam: an ordered chain of edges plus the merged
  /// geometry (cached for display; the planner re-merges from `parts`).
  struct Seam
  {
    uint32_t id = 0;
    std::string name;
    std::vector<rosim::core::SeamPart> parts;
    std::vector<Eigen::Vector3d> polyline;  ///< world frame, travel order
    double length = 0.0;
    bool closed = false;
    std::string last_result;  ///< "" never run, "ok" / "failed" after a batch
    double last_run = 0.0;    ///< unix seconds of the last execution attempt
    /// geometric fingerprints, used to detect an edge-index shift when a
    /// configuration is reloaded against a different OCC build
    std::vector<double> part_lengths;
    std::vector<Eigen::Vector3d> part_starts;
    std::vector<Eigen::Vector3d> part_ends;
  };
  std::mutex state_mutex_;
  std::string model_id_;
  /// TCP polyline of the last executed motion (world frame), for the 3D view.
  std::vector<std::array<double, 4>> tcp_path_;  // x, y, z, speed [m/s]
  /// Live trace while a follow action runs: the render thread appends the actual
  /// TCP pose each frame and re-broadcasts the growing path so the browser draws
  /// it during execution instead of only after the goal finishes.
  bool follow_active_ = false;
  bool has_live_tcp_ = false;
  Eigen::Vector3d live_tcp_ = Eigen::Vector3d::Zero();
  std::chrono::steady_clock::time_point live_stamp_{};
  std::chrono::steady_clock::time_point live_broadcast_{};
  /// Render thread appends the measured TCP pose and flags live_dirty_; a 5 Hz
  /// executor-thread timer does the socket broadcast (broadcasting on the render
  /// thread blocked frame delivery).
  std::atomic<bool> live_dirty_{false};
  rclcpp::TimerBase::SharedPtr live_timer_;
  std::string file_path_;
  /// Directory the model picker starts in (empty: derive from file_path_ or cwd).
  std::string model_dir_;
  Eigen::Isometry3d world_pose_ = Eigen::Isometry3d::Identity();
  std::vector<EdgeSummary> edges_;
  int selected_edge_ = -1;

  std::vector<Seam> seams_;
  uint32_t seam_counter_ = 0;
  int selected_seam_ = -1;
  std::string seam_store_path_;
  double speed_scale_ = 1.0;
  std::string planner_type_ = "ompl";

  // Workpiece display geometry for the debug WebGL viewer (guarded by
  // state_mutex_, same as the fields above).
  rosim::core::MeshData workpiece_mesh_;
  std::vector<rosim::core::EdgeData> workpiece_edges_;

  // Static robot link geometry for the debug WebGL viewer. Built once in
  // init() from the RobotModel; immutable afterward (no lock needed).
  nlohmann::json robot_scene_;

  // ---- ROS clients ----------------------------------------------------------
  rclcpp::Client<rosim_interfaces::srv::ImportModel>::SharedPtr import_client_;
  rclcpp::Client<rosim_interfaces::srv::GoHome>::SharedPtr go_home_client_;
  rclcpp::Client<rosim_interfaces::srv::MoveToPose>::SharedPtr move_to_pose_client_;
  rclcpp::Client<rosim_interfaces::srv::MoveWorkpiece>::SharedPtr move_workpiece_client_;
  rclcpp::Client<rosim_interfaces::srv::JogJoint>::SharedPtr jog_joint_client_;
  rclcpp::Client<rosim_interfaces::srv::JogCartesian>::SharedPtr jog_cartesian_client_;
  rclcpp::Client<rosim_interfaces::srv::SetSpeedScale>::SharedPtr speed_client_;
  rclcpp::Client<rosim_interfaces::srv::SetPlanner>::SharedPtr planner_client_;
  rclcpp::Client<rosim_interfaces::srv::CreateSeam>::SharedPtr create_seam_client_;
  rclcpp_action::Client<rosim_interfaces::action::FollowSeams>::SharedPtr follow_seams_client_;
  rclcpp_action::Client<FollowEdge>::SharedPtr follow_client_;
};

}  // namespace rosim::server
