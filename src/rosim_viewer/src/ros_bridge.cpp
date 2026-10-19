// ROS <-> Qt bridge for the legacy viewer: spins its own executor thread and
// translates service/action/topic events into queued Qt signals for the GUI.

#include "rosim_viewer/ros_bridge.h"

#include <chrono>
#include <cmath>

namespace rosim::viewer
{

RosBridge::RosBridge(QObject* parent) : QObject(parent)
{
  node_ = rclcpp::Node::make_shared("rosim_viewer");

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        emit jointStates(*msg);  // ROS thread -> GUI thread (queued)
      });

  import_client_ = node_->create_client<rosim_interfaces::srv::ImportModel>(
      "/rosim_planning/import_model");
  speed_client_ = node_->create_client<rosim_interfaces::srv::SetSpeedScale>(
      "/rosim_planning/set_speed_scale");
  go_home_client_ = node_->create_client<rosim_interfaces::srv::GoHome>(
      "/rosim_planning/go_home");
  jog_client_ = node_->create_client<rosim_interfaces::srv::JogJoint>(
      "/rosim_planning/jog_joint");
  follow_client_ = rclcpp_action::create_client<FollowEdge>(node_, "/rosim_planning/follow_edge");
}

RosBridge::~RosBridge()
{
  stopSpin();
}

void RosBridge::startSpin()
{
  if (spinning_.exchange(true))
    return;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

void RosBridge::stopSpin()
{
  if (!spinning_.exchange(false))
    return;
  if (executor_)
    executor_->cancel();
  if (spin_thread_.joinable())
    spin_thread_.join();
}

std::vector<rosim_interfaces::msg::EdgeInfo> RosBridge::lastImportedEdges() const
{
  std::lock_guard<std::mutex> lock(import_mutex_);
  return last_edges_;
}

void RosBridge::sendImportModel(const QString& file_path, double px, double py, double pz,
                                double yaw_rad)
{
  if (!import_client_->wait_for_service(std::chrono::seconds(2)))
  {
    emit importFinished(false, "", "planning node /import_model unavailable", 0);
    return;
  }
  auto req = std::make_shared<rosim_interfaces::srv::ImportModel::Request>();
  req->file_path = file_path.toStdString();
  req->world_pose.position.x = px;
  req->world_pose.position.y = py;
  req->world_pose.position.z = pz;
  // z-rotation quaternion from yaw
  req->world_pose.orientation.z = std::sin(yaw_rad / 2.0);
  req->world_pose.orientation.w = std::cos(yaw_rad / 2.0);

  import_client_->async_send_request(
      req, [this](rclcpp::Client<rosim_interfaces::srv::ImportModel>::SharedFuture future) {
        const auto res = future.get();
        {
          std::lock_guard<std::mutex> lock(import_mutex_);
          last_edges_ = res->edges;
        }
        emit importFinished(res->success, QString::fromStdString(res->model_id),
                            QString::fromStdString(res->message),
                            static_cast<int>(res->edges.size()));
      });
}

void RosBridge::sendSpeedScale(double scale)
{
  if (!speed_client_->wait_for_service(std::chrono::seconds(1)))
  {
    emit speedScaleResult(false, "set_speed_scale unavailable");
    return;
  }
  auto req = std::make_shared<rosim_interfaces::srv::SetSpeedScale::Request>();
  req->scale = scale;
  speed_client_->async_send_request(
      req, [this](rclcpp::Client<rosim_interfaces::srv::SetSpeedScale>::SharedFuture future) {
        const auto res = future.get();
        emit speedScaleResult(res->success, QString::fromStdString(res->message));
      });
}

void RosBridge::sendGoHome()
{
  if (!go_home_client_->wait_for_service(std::chrono::seconds(1)))
  {
    emit goHomeResult(false, "go_home unavailable");
    return;
  }
  auto req = std::make_shared<rosim_interfaces::srv::GoHome::Request>();
  go_home_client_->async_send_request(
      req, [this](rclcpp::Client<rosim_interfaces::srv::GoHome>::SharedFuture future) {
        const auto res = future.get();
        emit goHomeResult(res->success, QString::fromStdString(res->message));
      });
}

void RosBridge::sendJogJoint(const QString& joint_name, double delta)
{
  if (!jog_client_->wait_for_service(std::chrono::seconds(1)))
  {
    emit jogResult(false, "jog_joint unavailable");
    return;
  }
  auto req = std::make_shared<rosim_interfaces::srv::JogJoint::Request>();
  req->joint_name = joint_name.toStdString();
  req->delta = delta;
  jog_client_->async_send_request(
      req, [this](rclcpp::Client<rosim_interfaces::srv::JogJoint>::SharedFuture future) {
        const auto res = future.get();
        emit jogResult(res->success, QString::fromStdString(res->message));
      });
}

void RosBridge::sendFollowEdge(const QString& model_id, uint32_t edge_index,
                               uint8_t orientation_mode, double tool_offset,
                               double approach_distance, double max_spacing,
                               double tangent_tilt)
{
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    emit followResult(false, "planning node /follow_edge unavailable", 0.0);
    return;
  }

  auto goal = FollowEdge::Goal();
  goal.model_id = model_id.toStdString();
  goal.edge_index = edge_index;
  goal.orientation_mode = orientation_mode;
  goal.tool_offset = tool_offset;
  goal.approach_distance = approach_distance;
  goal.max_waypoint_spacing = max_spacing;
  goal.tangent_tilt = tangent_tilt;
  // FIXED mode: torch straight DOWN (x,y,z,w = 1,0,0,0 — 180° about x).
  // Identity would point the torch UP and bury the approach pose in/under the
  // workpiece, killing IK.
  if (orientation_mode == FollowEdge::Goal::ORIENTATION_FIXED)
  {
    goal.fixed_orientation.x = 1.0;
    goal.fixed_orientation.y = 0.0;
    goal.fixed_orientation.z = 0.0;
    goal.fixed_orientation.w = 0.0;
  }

  using SendGoalOptions =
      rclcpp_action::Client<FollowEdge>::SendGoalOptions;
  SendGoalOptions opts;
  opts.goal_response_callback =
      [this](GoalHandleFollowEdge::SharedPtr goal_handle) {
        if (!goal_handle)
        {
          emit followResult(false, "goal rejected by planning node", 0.0);
          return;
        }
        std::lock_guard<std::mutex> lock(goal_mutex_);
        active_goal_ = goal_handle;
      };
  opts.feedback_callback =
      [this](GoalHandleFollowEdge::SharedPtr,
             const std::shared_ptr<const FollowEdge::Feedback> feedback) {
        emit followFeedback(feedback->progress, QString::fromStdString(feedback->phase));
      };
  opts.result_callback = [this](const GoalHandleFollowEdge::WrappedResult& wrapped) {
    if (wrapped.result)
      emit followResult(wrapped.result->success,
                        QString::fromStdString(wrapped.result->message),
                        wrapped.result->cartesian_fraction);
    else
      emit followResult(false, "goal aborted", 0.0);
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_ = nullptr;
  };
  follow_client_->async_send_goal(goal, opts);
}

void RosBridge::cancelFollowEdge()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_)
    follow_client_->async_cancel_goal(active_goal_);
}

}  // namespace rosim::viewer
