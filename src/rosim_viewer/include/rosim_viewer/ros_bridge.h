#pragma once

#include <memory>
#include <mutex>
#include <QString>
#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rosim_interfaces/action/follow_edge.hpp"
#include "rosim_interfaces/msg/edge_info.hpp"
#include "rosim_interfaces/srv/go_home.hpp"
#include "rosim_interfaces/srv/import_model.hpp"
#include "rosim_interfaces/srv/jog_joint.hpp"
#include "rosim_interfaces/srv/set_speed_scale.hpp"

namespace rosim::viewer
{

/// \brief All ROS interaction lives here; the GUI only sees Qt signals/slots.
///
/// Threading: an internal MultiThreadedExecutor spins in its own thread.
/// ROS callbacks emit Qt signals; receivers live on the GUI thread, so
/// AutoConnection resolves to queued delivery. Signal payloads are Qt/POD
/// types only (no Eigen) — the edge list is fetched explicitly after the
/// importFinished signal via lastImportedEdges() (mutex-guarded).
class RosBridge : public QObject
{
  Q_OBJECT

public:
  using FollowEdge = rosim_interfaces::action::FollowEdge;
  using GoalHandleFollowEdge = rclcpp_action::ClientGoalHandle<FollowEdge>;

  explicit RosBridge(QObject* parent = nullptr);
  ~RosBridge() override;

  rclcpp::Node::SharedPtr node() const { return node_; }

  void startSpin();
  void stopSpin();

  /// Last import result edges (call on the GUI thread after importFinished).
  std::vector<rosim_interfaces::msg::EdgeInfo> lastImportedEdges() const;

public slots:
  /// Import a STEP workpiece at a default pose in front of the robot.
  void sendImportModel(const QString& file_path, double px, double py, double pz,
                       double yaw_rad = 0.0);
  void sendSpeedScale(double scale);
  /// OMPL-planned return to the SRDF "home" state (never forced).
  void sendGoHome();
  /// Nudge one joint by delta rad (clamped + collision-checked server-side).
  void sendJogJoint(const QString& joint_name, double delta);
  void sendFollowEdge(const QString& model_id, uint32_t edge_index,
                      uint8_t orientation_mode, double tool_offset,
                      double approach_distance, double max_spacing,
                      double tangent_tilt = 0.0);
  void cancelFollowEdge();

signals:
  /// 100 Hz-ish joint states (queued to GUI thread).
  void jointStates(sensor_msgs::msg::JointState state);
  void importFinished(bool ok, QString model_id, QString message, int edge_count);
  void speedScaleResult(bool ok, QString message);
  void goHomeResult(bool ok, QString message);
  void jogResult(bool ok, QString message);
  void followFeedback(float progress, QString phase);
  void followResult(bool ok, QString message, double cartesian_fraction);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> spinning_{ false };

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<rosim_interfaces::srv::ImportModel>::SharedPtr import_client_;
  rclcpp::Client<rosim_interfaces::srv::SetSpeedScale>::SharedPtr speed_client_;
  rclcpp::Client<rosim_interfaces::srv::GoHome>::SharedPtr go_home_client_;
  rclcpp::Client<rosim_interfaces::srv::JogJoint>::SharedPtr jog_client_;
  rclcpp_action::Client<FollowEdge>::SharedPtr follow_client_;

  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleFollowEdge> active_goal_;

  mutable std::mutex import_mutex_;
  std::vector<rosim_interfaces::msg::EdgeInfo> last_edges_;
};

}  // namespace rosim::viewer
