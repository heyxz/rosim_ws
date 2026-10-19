// rosim_server node: headless scene rendering plus the WebSocket bridge. Owns
// the display robot model, workpiece display mesh/edges, render thread and the
// browser protocol; forwards control messages to rosim_planning services and
// the FollowEdge action, then pushes status/scene updates back to clients.

#include "rosim_server/server_node.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <set>
#include <fstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_interfaces/srv/create_seam.hpp"
#include "rosim_server/frame_renderer.h"
#include "rosim_server/ws_server.h"

namespace rosim::server
{

namespace
{
using ImportModel = rosim_interfaces::srv::ImportModel;
using GoHome = rosim_interfaces::srv::GoHome;
using MoveToPose = rosim_interfaces::srv::MoveToPose;
using MoveWorkpiece = rosim_interfaces::srv::MoveWorkpiece;
using JogJoint = rosim_interfaces::srv::JogJoint;
using JogCartesian = rosim_interfaces::srv::JogCartesian;
using SetSpeedScale = rosim_interfaces::srv::SetSpeedScale;
using SetPlanner = rosim_interfaces::srv::SetPlanner;
using CreateSeam = rosim_interfaces::srv::CreateSeam;
using FollowSeamsAction = rosim_interfaces::action::FollowSeams;
using FollowSeamsGoalHandle = rclcpp_action::ClientGoalHandle<FollowSeamsAction>;

/// TANGENT-mode lead angle used when a client omits tangent_tilt (production
/// HMI does not expose process parameters). Zero lays the torch axis exactly
/// on the seam; a straight torch body then sweeps through the part and the
/// cartesian run truncates at the first contact (typically <10% of top-face
/// edges). ~34 deg lifts the trailing body clear while the wire tip stays on
/// the edge; the follower flips the sign automatically when the other side is
/// infeasible.
constexpr double kDefaultTangentTilt = 0.6;

/// Build a {"type":"status","ok":…,"message":…} frame without relying on
/// heterogeneous initializer-lists (nlohmann can't deduce those uniformly).
nlohmann::json statusJson(bool ok, const std::string& message)
{
  nlohmann::json j;
  j["type"] = "status";
  j["ok"] = ok;
  j["message"] = message;
  return j;
}

/// Serialize an Eigen pose as {pos:[x,y,z], quat:[x,y,z,w]}. A quaternion on
/// the wire sidesteps the row/column-major ambiguity of a flat 4x4 matrix.
void poseToJson(nlohmann::json& j, const Eigen::Isometry3d& pose)
{
  const Eigen::Quaterniond q(pose.linear());
  j["pos"] = { pose.translation().x(), pose.translation().y(), pose.translation().z() };
  j["quat"] = { q.x(), q.y(), q.z(), q.w() };
}

std::string base64Encode(const unsigned char* data, size_t len);
std::string meshToBase64(const double* verts, size_t vcount, const uint32_t* tris,
                         size_t tcount);

/// Serialize one URDF collision shape as primitive params (box/cylinder/sphere)
/// or a packed triangle soup (mesh). Returns false for unsupported shape types.
bool shapeToJson(nlohmann::json& j, const shapes::Shape* shape)
{
  switch (shape->type)
  {
    case shapes::CYLINDER:
    {
      const auto* cyl = static_cast<const shapes::Cylinder*>(shape);
      j["kind"] = "cylinder";
      j["radius"] = cyl->radius;
      j["height"] = cyl->length;
      return true;
    }
    case shapes::BOX:
    {
      const auto* box = static_cast<const shapes::Box*>(shape);
      j["kind"] = "box";
      j["size"] = { box->size[0], box->size[1], box->size[2] };
      return true;
    }
    case shapes::SPHERE:
    {
      const auto* sph = static_cast<const shapes::Sphere*>(shape);
      j["kind"] = "sphere";
      j["radius"] = sph->radius;
      return true;
    }
    case shapes::MESH:
    {
      const auto* mesh = static_cast<const shapes::Mesh*>(shape);
      j["kind"] = "mesh";
      j["vcount"] = mesh->vertex_count;
      j["tcount"] = mesh->triangle_count;
      j["data"] = meshToBase64(mesh->vertices, mesh->vertex_count,
                               reinterpret_cast<const uint32_t*>(mesh->triangles),
                               mesh->triangle_count);
      return true;
    }
    default:
      return false;
  }
}

/// Pack a triangle soup (float32 vertices + uint32 indices) into a base64
/// string. JSON doubles are ~2-3x the wire size of raw floats, and STL links
/// are the dominant payload — this keeps the robot scene a few MB instead of
/// tens of MB.
std::string meshToBase64(const double* verts, size_t vcount, const uint32_t* tris, size_t tcount)
{
  std::vector<unsigned char> bytes(vcount * 12 + tcount * 12);
  unsigned char* p = bytes.data();
  for (size_t i = 0; i < vcount * 3; ++i)
  {
    const float f = static_cast<float>(verts[i]);
    std::memcpy(p, &f, 4);
    p += 4;
  }
  for (size_t i = 0; i < tcount * 3; ++i)
  {
    std::memcpy(p, &tris[i], 4);
    p += 4;
  }
  return base64Encode(bytes.data(), bytes.size());
}

std::string base64Encode(const unsigned char* data, size_t len)
{
  static const char* tbl =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  for (size_t i = 0; i < len; i += 3)
  {
    unsigned int n = static_cast<unsigned int>(data[i]) << 16;
    if (i + 1 < len)
      n |= static_cast<unsigned int>(data[i + 1]) << 8;
    if (i + 2 < len)
      n |= static_cast<unsigned int>(data[i + 2]);
    out.push_back(tbl[(n >> 18) & 63]);
    out.push_back(tbl[(n >> 12) & 63]);
    out.push_back(i + 1 < len ? tbl[(n >> 6) & 63] : '=');
    out.push_back(i + 2 < len ? tbl[n & 63] : '=');
  }
  return out;
}
}  // namespace

ServerNode::ServerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("rosim_server", options)
{
  declare_parameter("width", 1280);
  declare_parameter("height", 720);
  declare_parameter("fps", 12);
  declare_parameter("quality", 70);
  declare_parameter("ws_address", "0.0.0.0");
  declare_parameter("ws_port", 8765);
  declare_parameter("web_root", "");

  width_ = get_parameter("width").as_int();
  height_ = get_parameter("height").as_int();
  fps_ = get_parameter("fps").as_int();
  quality_ = get_parameter("quality").as_int();
  ws_address_ = get_parameter("ws_address").as_string();
  ws_port_ = get_parameter("ws_port").as_int();
  web_root_ = get_parameter("web_root").as_string();

  // Where seam definitions live. Kept next to the user's ROS state by default;
  // an env override lets tests / packaged configs point elsewhere. Persisting
  // them is what turns "seams" into a reusable production configuration.
  {
    const char* env = std::getenv("ROSIM_SEAM_STORE");
    if (env && *env)
    {
      seam_store_path_ = env;
    }
    else
    {
      const char* home = std::getenv("HOME");
      seam_store_path_ = std::string(home ? home : ".") + "/.ros/rosim/seams.json";
    }
    RCLCPP_INFO(get_logger(), "seam store: %s", seam_store_path_.c_str());
  }

  if (fps_ < 1)
    fps_ = 1;
  if (quality_ < 1 || quality_ > 100)
    quality_ = 70;

  // Default the web root to the installed share dir; only overridden by param.
  if (web_root_.empty())
  {
    try
    {
      web_root_ = ament_index_cpp::get_package_share_directory("rosim_server") + "/web";
    }
    catch (const std::exception&)
    {
      // Running from a non-installed workspace: no HTTP page, WS still works.
      web_root_.clear();
    }
  }
}

ServerNode::~ServerNode()
{
  running_ = false;
  if (render_thread_.joinable())
    render_thread_.join();

  // Cancel any in-flight follow action before tearing down the node.
  if (follow_client_)
    follow_client_->async_cancel_all_goals([](auto) {});

  if (ws_)
    ws_->stop();
}

void ServerNode::init()
{
  loadRobotModel();
  buildRobotScene();
  setupClients();
  setupWebServer();

  render_thread_ = std::thread([this]() { renderLoop(); });
  RCLCPP_INFO(get_logger(), "rosim_server ready: %s:%d (%dx%d @ %d fps)",
              ws_address_.c_str(), ws_port_, width_, height_, fps_);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void ServerNode::loadRobotModel()
{
  // Display only needs FK, so skip the kinematics solvers (no kinematics.yaml
  // needed on this node). The planning node keeps the full IK stack.
  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoader loader(shared_from_this(), opts);
  model_ = loader.getModel();
  if (model_)
  {
    display_state_ = std::make_shared<moveit::core::RobotState>(model_);
    display_state_->setToDefaultValues();
    // Compute link transforms now so robotStateJson() has valid poses before
    // the first /joint_states message arrives.
    display_state_->update();
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "robot model unavailable — rendering without the robot (is "
                "robot_description set on this node?)");
  }
}

void ServerNode::setupClients()
{
  import_client_ = create_client<ImportModel>("/rosim_planning/import_model");
  go_home_client_ = create_client<GoHome>("/rosim_planning/go_home");
  move_to_pose_client_ = create_client<MoveToPose>("/rosim_planning/move_to_pose");
  move_workpiece_client_ =
      create_client<MoveWorkpiece>("/rosim_planning/move_workpiece");
  jog_joint_client_ = create_client<JogJoint>("/rosim_planning/jog_joint");
  jog_cartesian_client_ = create_client<JogCartesian>("/rosim_planning/jog_cartesian");
  speed_client_ = create_client<SetSpeedScale>("/rosim_planning/set_speed_scale");
  planner_client_ = create_client<SetPlanner>("/rosim_planning/set_planner");
  create_seam_client_ = create_client<CreateSeam>("/rosim_planning/create_seam");
  follow_client_ = rclcpp_action::create_client<FollowEdge>(this, "/rosim_planning/follow_edge");
  follow_seams_client_ =
      rclcpp_action::create_client<FollowSeamsAction>(this, "/rosim_planning/follow_seams");

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        pending_joint_ = std::move(msg);
      });
}

void ServerNode::setupWebServer()
{
  renderer_ = std::make_unique<FrameRenderer>(width_, height_);
  renderer_->init(model_.get() ? model_ : nullptr);

  ws_ = std::make_unique<WsServer>(ws_address_, static_cast<unsigned short>(ws_port_),
                                   web_root_);
  ws_->setMessageHandler([this](const std::string& text, WsServer::BinarySetter set_binary,
                                WsServer::BinarySetter set_scene,
                                WsServer::BinarySetter set_workpiece) {
    onWsMessage(text, std::move(set_binary), std::move(set_scene),
                std::move(set_workpiece));
  });
  ws_->setConnectHandler([this]() { onWsConnect(); });

  if (!ws_->start())
    RCLCPP_FATAL(get_logger(), "failed to bind WebSocket server on %s:%d",
                 ws_address_.c_str(), ws_port_);
}

// ---------------------------------------------------------------------------
// WebSocket dispatch
// ---------------------------------------------------------------------------
void ServerNode::onWsConnect()
{
  // A fresh browser knows nothing about the current scene — push the snapshot.
  broadcastStateLocked();
}

void ServerNode::onWsMessage(const std::string& text, std::function<void(bool)> set_binary,
                             std::function<void(bool)> set_scene,
                             std::function<void(bool)> set_workpiece)
{
  nlohmann::json j;
  try
  {
    j = nlohmann::json::parse(text);
  }
  catch (const std::exception& ex)
  {
    sendError(std::string("invalid JSON: ") + ex.what());
    return;
  }

  const std::string type = j.value("type", "");
  if (type == "subscribe_stream")
  {
    if (set_binary)
      set_binary(true);
    return;
  }
  if (type == "unsubscribe_stream")
  {
    if (set_binary)
      set_binary(false);
    return;
  }
  if (type == "subscribe_scene")
  {
    if (set_scene)
      set_scene(true);
    // Push the current geometry snapshot so the viewer can build the scene.
    broadcastScene();
    broadcastSeamsLocked();  // seams are user data, not part of the scene
    broadcastTcpPathLocked();
    return;
  }
  if (type == "unsubscribe_scene")
  {
    if (set_scene)
      set_scene(false);
    return;
  }
  if (type == "subscribe_workpiece")
  {
    // Production viewer: the part geometry only. No robot scene and no
    // robot_state stream — an HMI panel should not pay for the 1.26M-triangle
    // robot model. Debug scene subscribers receive this broadcast as well.
    if (set_workpiece)
      set_workpiece(true);
    broadcastWorkpiece();
    broadcastSeamsLocked();
    return;
  }
  if (type == "unsubscribe_workpiece")
  {
    if (set_workpiece)
      set_workpiece(false);
    return;
  }
  if (type == "import")
    handleImport(j);
  else if (type == "select_edge")
    handleSelectEdge(j);
  else if (type == "create_seam")
    handleCreateSeam(j);
  else if (type == "delete_seam")
    handleDeleteSeam(j);
  else if (type == "rename_seam")
    handleRenameSeam(j);
  else if (type == "reorder_seams")
    handleReorderSeams(j);
  else if (type == "select_seam")
    handleSelectSeam(j);
  else if (type == "follow_seams")
    handleFollowSeams(j);
  else if (type == "follow")
    handleFollow(j);
  else if (type == "clear_tcp_path")
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      tcp_path_.clear();
    }
    broadcastTcpPathLocked();
  }
  else if (type == "stop")
    handleStop(j);
  else if (type == "go_home")
    handleGoHome(j);
  else if (type == "move_to_pose")
    handleMoveToPose(j);
  else if (type == "set_workpiece_pose")
    handleSetWorkpiecePose(j);
  else if (type == "jog_joint")
    handleJogJoint(j);
  else if (type == "jog_cartesian")
    handleJogCartesian(j);
  else if (type == "set_speed")
    handleSetSpeed(j);
  else if (type == "set_planner")
    handleSetPlanner(j);
  else if (type == "get_state")
    handleGetState(j);
  else
    sendError("unknown message type: " + type);
}

// ---------------------------------------------------------------------------
// Control handlers
// ---------------------------------------------------------------------------
void ServerNode::handleImport(const nlohmann::json& j)
{
  if (!j.contains("file_path") || !j["file_path"].is_string())
  {
    sendError("import: missing file_path");
    return;
  }
  const std::string path = j["file_path"].get<std::string>();
  const Eigen::Isometry3d pose = parseWorldPose(j.contains("pose") ? j["pose"] : nlohmann::json());

  auto req = std::make_shared<ImportModel::Request>();
  req->file_path = path;
  req->world_pose = poseToMsg(pose);

  import_client_->async_send_request(
      req, [this, path, pose](rclcpp::Client<ImportModel>::SharedFuture future) {
        auto resp = future.get();
        if (!resp->success)
        {
          sendError("import failed: " + resp->message);
          return;
        }

        // Planning scene is registered; now build the display geometry locally
        // (solid tessellation + edge tubes) for the renderer and the WebGL
        // debug viewer.
        rosim::core::MeshData mesh;
        std::vector<rosim::core::EdgeData> edges_full;
        try
        {
          auto wm = rosim::core::OccGeometryService::importModel(path, pose, resp->model_id);
          rosim::core::OccGeometryService::tessellate(wm->shape, 3e-4, 0.35, mesh);
          edges_full = std::move(wm->edges);
          std::lock_guard<std::mutex> lock(render_mutex_);
          renderer_->clearWorkpieces();
          renderer_->addWorkpiece(resp->model_id, mesh, edges_full, pose);
        }
        catch (const std::exception& ex)
        {
          // Planning import succeeded; display-only tessellation is non-fatal.
          RCLCPP_WARN(get_logger(), "display tessellation failed: %s", ex.what());
        }

        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          model_id_ = resp->model_id;
          file_path_ = path;
          world_pose_ = pose;
          selected_edge_ = -1;
          workpiece_mesh_ = std::move(mesh);
          workpiece_edges_ = std::move(edges_full);
          edges_.clear();
          edges_.reserve(resp->edges.size());
          for (const auto& e : resp->edges)
            edges_.push_back({ e.edge_index, e.length, e.closed });
        }

        highlightSelectedEdge();  // clears any previous highlight
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          seams_.clear();
          seam_counter_ = 0;
          selected_seam_ = -1;
          loadSeamsLocked();
        }
        broadcastEdgesLocked();
        broadcastSeamsLocked();
        broadcastStateLocked();
        broadcastWorkpiece();  // push it to debug and production workpiece viewers
        broadcastJson(statusJson(true, "imported " + resp->model_id));
      });
}

void ServerNode::handleSetWorkpiecePose(const nlohmann::json& j)
{
  if (!j.contains("pose") || !j["pose"].is_object())
  {
    sendError("set_workpiece_pose: missing pose");
    return;
  }
  const Eigen::Isometry3d pose = parseWorldPose(j["pose"]);

  std::string model_id;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_id = model_id_;
  }
  if (model_id.empty())
  {
    sendError("set_workpiece_pose: no workpiece imported");
    return;
  }

  auto req = std::make_shared<MoveWorkpiece::Request>();
  req->model_id = model_id;
  req->world_pose = poseToMsg(pose);
  move_workpiece_client_->async_send_request(
      req, [this, pose](rclcpp::Client<MoveWorkpiece>::SharedFuture future) {
        auto resp = future.get();
        if (!resp->success)
        {
          sendError("move workpiece failed: " + resp->message);
          return;
        }

        // Mirror the planning node's change locally: edges are world-frame
        // polylines, the display mesh is local and carries the new pose.
        std::string model_id;
        rosim::core::MeshData mesh;
        std::vector<rosim::core::EdgeData> edges;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          model_id = model_id_;
          const Eigen::Isometry3d delta = pose * world_pose_.inverse();
          for (auto& e : workpiece_edges_)
            for (auto& p : e.polyline)
              p = delta * p;
          world_pose_ = pose;
          mesh = workpiece_mesh_;
          edges = workpiece_edges_;
        }

        {
          std::lock_guard<std::mutex> lock(render_mutex_);
          renderer_->clearWorkpieces();
          // Keep the MJPEG stream camera where the operator left it.
          renderer_->addWorkpiece(model_id, mesh, edges, pose, /*reset_camera=*/false);
        }
        highlightSelectedEdge();
        broadcastWorkpiece();
        broadcastJson(statusJson(true, "workpiece moved"));
      });
}

void ServerNode::handleSelectEdge(const nlohmann::json& j)
{
  const int index = j.value("edge_index", -1);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    selected_edge_ = index;
  }
  highlightSelectedEdge();
  broadcastJson(statusJson(true, "selected edge " + std::to_string(index)));
}

void ServerNode::handleFollow(const nlohmann::json& j)
{
  std::string model_id;
  int edge_index;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_id = model_id_;
    edge_index = j.value("edge_index", selected_edge_);
  }
  if (model_id.empty())
  {
    sendError("follow: no model imported");
    return;
  }
  const bool seam_mode = j.contains("edge_indices") && j["edge_indices"].is_array() &&
                         !j["edge_indices"].empty();
  if (edge_index < 0 && !seam_mode)
  {
    sendError("follow: select a seam or an edge first");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    selected_edge_ = edge_index;
  }
  highlightSelectedEdge();

  FollowEdge::Goal goal;
  goal.model_id = model_id;
  goal.edge_index = static_cast<uint32_t>(edge_index);
  // Seam planning: forward the chain when the caller selected a seam.
  if (j.contains("edge_indices") && j["edge_indices"].is_array())
    for (const auto& v : j["edge_indices"])
      if (v.is_number_integer())
        goal.edge_indices.push_back(v.get<uint32_t>());
  goal.orientation_mode = j.value("orientation_mode", 0);
  goal.tangent_tilt = j.value("tangent_tilt", kDefaultTangentTilt);
  goal.tool_offset = j.value("tool_offset", 0.0);
  goal.approach_distance = j.value("approach_distance", 0.08);
  goal.max_waypoint_spacing = j.value("max_waypoint_spacing", 0.005);
  goal.corner_max_deviation = j.value("corner_max_deviation", 0.0);
  goal.corner_threshold_deg = j.value("corner_threshold_deg", 0.0);
  goal.corner_orient_window = j.value("corner_orient_window", 0.0);
  goal.time_profile = static_cast<uint8_t>(j.value("time_profile", 0));
  goal.seam_constant_speed = j.value("seam_constant_speed", false);
  goal.weld_speed = j.value("weld_speed", 0.0);
  goal.seam_dwell = j.value("seam_dwell", 0.0);
  // 0 auto / 1 forward / 2 reverse (mirrors the action constants).
  goal.direction = static_cast<uint8_t>(j.value("direction", 0));
  goal.corner_max_deviation = j.value("corner_max_deviation", 0.0);
  goal.corner_threshold_deg = j.value("corner_threshold_deg", 0.0);
  goal.corner_orient_window = j.value("corner_orient_window", 0.0);
  goal.time_profile = static_cast<uint8_t>(j.value("time_profile", 0));
  goal.seam_constant_speed = j.value("seam_constant_speed", false);
  goal.weld_speed = j.value("weld_speed", 0.0);
  goal.seam_dwell = j.value("seam_dwell", 0.0);
  goal.seam_constant_speed = j.value("seam_constant_speed", false);
  goal.weld_speed = j.value("weld_speed", 0.0);
  goal.seam_dwell = j.value("seam_dwell", 0.0);
  if (j.contains("fixed_orientation") && j["fixed_orientation"].is_object())
  {
    goal.fixed_orientation.x = j["fixed_orientation"].value("x", 0.0);
    goal.fixed_orientation.y = j["fixed_orientation"].value("y", 0.0);
    goal.fixed_orientation.z = j["fixed_orientation"].value("z", 0.0);
    goal.fixed_orientation.w = j["fixed_orientation"].value("w", 1.0);
  }

  auto options = rclcpp_action::Client<FollowEdge>::SendGoalOptions();
  options.goal_response_callback = [this](std::shared_ptr<FollowGoalHandle> handle) {
    broadcastJson(statusJson(static_cast<bool>(handle),
                             handle ? "follow goal accepted" : "follow goal rejected (busy or bad edge)"));
  };
  options.feedback_callback =
      [this](std::shared_ptr<FollowGoalHandle>,
             const std::shared_ptr<const FollowEdge::Feedback> feedback) {
        nlohmann::json j;
        j["type"] = "follow_feedback";
        j["progress"] = feedback->progress;
        j["phase"] = feedback->phase;
        j["note"] = feedback->note;
        broadcastJson(std::move(j));
      };
  options.result_callback = [this](const FollowGoalHandle::WrappedResult& result) {
    nlohmann::json j;
    j["type"] = "follow_result";
    j["success"] = result.result->success;
    j["error_code"] = result.result->error_code;
    j["message"] = result.result->message;
    j["cartesian_fraction"] = result.result->cartesian_fraction;
    j["direction_used"] = result.result->direction_used;
    broadcastJson(std::move(j));
    if (!result.result->tcp_samples.empty())
    {
      nlohmann::json sj;
      sj["type"] = "tcp_samples";
      sj["points"] = result.result->tcp_samples;
      broadcastJson(std::move(sj));
    }
    publishTcpPath(result.result->trajectory);
  };

  follow_client_->async_send_goal(goal, options);
}

void ServerNode::publishTcpPath(const trajectory_msgs::msg::JointTrajectory& traj)
{
  if (traj.points.empty() || traj.joint_names.empty())
    return;

  // Forward kinematics on a local state: the display state is owned by the
  // render thread and must not be mutated from the executor thread.
  moveit::core::RobotState st(model_);
  st.setToDefaultValues();

  std::vector<std::array<double, 4>> path;  // x, y, z, TCP speed [m/s]
  path.reserve(traj.points.size());
  Eigen::Vector3d prev = Eigen::Vector3d::Zero();
  double prev_t = 0.0;
  bool have_prev = false;
  std::vector<double> prev_q;
  for (const auto& pt : traj.points)
  {
    if (pt.positions.size() != traj.joint_names.size())
      continue;
    st.setVariablePositions(traj.joint_names, pt.positions);
    st.update();
    const Eigen::Vector3d p = st.getGlobalLinkTransform("tool_tip").translation();
    const double t = rclcpp::Duration(pt.time_from_start).seconds();
    double v = 0.0;
    if (have_prev && t > prev_t + 1e-6)
      v = (p - prev).norm() / (t - prev_t);

    // The trajectory controller interpolates between waypoints; joining them
    // with straight segments makes small corner arcs (r ~ 1 mm) collapse into
    // a visible diagonal. Interpolate the joints in between so the drawn line
    // follows the commanded motion instead of the chord.
    if (have_prev && prev_q.size() == pt.positions.size())
    {
      const double gap = (p - prev).norm();
      const int sub = std::min(40, static_cast<int>(std::ceil(gap / 1.5e-3)));
      for (int k = 1; k < sub; ++k)
      {
        const double a = static_cast<double>(k) / sub;
        std::vector<double> q(prev_q.size());
        for (std::size_t j = 0; j < q.size(); ++j)
          q[j] = prev_q[j] + a * (pt.positions[j] - prev_q[j]);
        st.setVariablePositions(traj.joint_names, q);
        st.update();
        const Eigen::Vector3d pi = st.getGlobalLinkTransform("tool_tip").translation();
        path.push_back({ pi.x(), pi.y(), pi.z(), v });
      }
      st.setVariablePositions(traj.joint_names, pt.positions);
      st.update();
    }

    path.push_back({ p.x(), p.y(), p.z(), v });
    prev = p;
    prev_t = t;
    prev_q = pt.positions;
    have_prev = true;
  }
  if (path.size() < 2)
    return;

  // Decimate: the browser only draws a few hundred segments anyway, and long
  // seams can produce thousands of 4 mm samples.
  constexpr std::size_t kMaxPoints = 900;
  if (path.size() > kMaxPoints)
  {
    std::vector<std::array<double, 4>> thin;
    thin.reserve(kMaxPoints);
    const double step = static_cast<double>(path.size() - 1) / (kMaxPoints - 1);
    for (std::size_t i = 0; i < kMaxPoints; ++i)
      thin.push_back(path[static_cast<std::size_t>(i * step + 0.5)]);
    path.swap(thin);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    tcp_path_ = path;
  }
  broadcastTcpPathLocked();
}

nlohmann::json ServerNode::tcpPathJsonLocked() const
{
  nlohmann::json j;
  j["type"] = "tcp_path";
  j["points"] = nlohmann::json::array();
  for (const auto& p : tcp_path_)
    j["points"].push_back({ p[0], p[1], p[2], p[3] });
  return j;
}

void ServerNode::broadcastTcpPathLocked()
{
  std::vector<std::array<double, 4>> snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot = tcp_path_;
  }
  nlohmann::json j;
  j["type"] = "tcp_path";
  j["points"] = nlohmann::json::array();
  for (const auto& p : snapshot)
    j["points"].push_back({ p[0], p[1], p[2], p[3] });
  broadcastJson(std::move(j));
}

void ServerNode::handleStop(const nlohmann::json& /*j*/)
{
  // Cancels every outstanding follow goal; the action server's cancel handler
  // sets its cancel_ flag, which the running pipeline checks between phases.
  follow_client_->async_cancel_all_goals([](auto) {});
  broadcastJson(statusJson(true, "stop requested"));
}

void ServerNode::handleGoHome(const nlohmann::json& /*j*/)
{
  go_home_client_->async_send_request(
      std::make_shared<GoHome::Request>(),
      [this](rclcpp::Client<GoHome>::SharedFuture future) {
        auto resp = future.get();
        broadcastJson(statusJson(resp->success, resp->message));
      });
}

void ServerNode::handleMoveToPose(const nlohmann::json& j)
{
  if (!j.contains("pose"))
  {
    sendError("move_to_pose: missing pose");
    return;
  }
  auto req = std::make_shared<MoveToPose::Request>();
  req->target_pose = poseToMsg(parseWorldPose(j["pose"]));

  move_to_pose_client_->async_send_request(
      req, [this](rclcpp::Client<MoveToPose>::SharedFuture future) {
        auto resp = future.get();
        broadcastJson(statusJson(resp->success, resp->message));
      });
}

void ServerNode::handleJogJoint(const nlohmann::json& j)
{
  auto req = std::make_shared<JogJoint::Request>();
  req->joint_name = j.value("joint_name", "");
  req->delta = j.value("delta", 0.0);
  if (req->joint_name.empty())
  {
    sendError("jog_joint: missing joint_name");
    return;
  }
  jog_joint_client_->async_send_request(
      req, [this](rclcpp::Client<JogJoint>::SharedFuture future) {
        auto resp = future.get();
        broadcastJson(statusJson(resp->success, resp->message));
      });
}

void ServerNode::handleJogCartesian(const nlohmann::json& j)
{
  auto req = std::make_shared<JogCartesian::Request>();
  req->axis = static_cast<uint8_t>(j.value("axis", 0));
  req->delta = j.value("delta", 0.0);
  if (req->axis > 5 || std::fabs(req->delta) < 1e-9)
  {
    sendError("jog_cartesian: need axis 0..5 and a non-zero delta");
    return;
  }
  jog_cartesian_client_->async_send_request(
      req, [this](rclcpp::Client<JogCartesian>::SharedFuture future) {
        auto resp = future.get();
        // Successful jogs are silent: the UI follows the joint stream and
        // would otherwise be flooded at ~10 Hz. Failures must be visible.
        if (!resp->success)
          broadcastJson(statusJson(false, resp->message));
      });
}

void ServerNode::handleSetSpeed(const nlohmann::json& j)
{
  auto req = std::make_shared<SetSpeedScale::Request>();
  req->scale = j.value("scale", 1.0);
  speed_client_->async_send_request(
      req, [this, scale = req->scale](rclcpp::Client<SetSpeedScale>::SharedFuture future) {
        auto resp = future.get();
        if (resp->success)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          speed_scale_ = scale;
        }
        broadcastJson(statusJson(resp->success, resp->message));
      });
}

void ServerNode::handleSetPlanner(const nlohmann::json& j)
{
  auto req = std::make_shared<SetPlanner::Request>();
  req->planner_type = j.value("planner_type", "ompl");
  planner_client_->async_send_request(
      req, [this, type = req->planner_type](rclcpp::Client<SetPlanner>::SharedFuture future) {
        auto resp = future.get();
        if (resp->success)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          planner_type_ = type;
        }
        broadcastJson(statusJson(resp->success, resp->message));
      });
}

void ServerNode::handleGetState(const nlohmann::json& /*j*/)
{
  broadcastStateLocked();
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void ServerNode::broadcastJson(nlohmann::json j)
{
  ws_->broadcastText(j.dump());
}

void ServerNode::sendError(const std::string& message)
{
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
  broadcastJson(statusJson(false, message));
}

void ServerNode::broadcastEdgesLocked()
{
  nlohmann::json j;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    j["type"] = "edges";
    j["model_id"] = model_id_;
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& e : edges_)
    {
      nlohmann::json o;
      o["index"] = e.index;
      o["length"] = e.length;
      o["closed"] = e.closed;
      arr.push_back(std::move(o));
    }
    j["edges"] = std::move(arr);
  }
  broadcastJson(std::move(j));
}

void ServerNode::broadcastStateLocked()
{
  nlohmann::json j;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    j["type"] = "state";
    j["model_id"] = model_id_;
    j["file_path"] = file_path_;
    j["selected_edge"] = selected_edge_;
    j["selected_seam"] = selected_seam_;
    j["speed_scale"] = speed_scale_;
    j["planner"] = planner_type_;
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& e : edges_)
    {
      nlohmann::json o;
      o["index"] = e.index;
      o["length"] = e.length;
      o["closed"] = e.closed;
      arr.push_back(std::move(o));
    }
    j["edges"] = std::move(arr);
  }
  broadcastJson(std::move(j));
}

void ServerNode::buildRobotScene()
{
  nlohmann::json robot = nlohmann::json::array();
  if (model_)
  {
    for (const moveit::core::LinkModel* link : model_->getLinkModels())
    {
      if (!link)
        continue;
      const auto& shapes = link->getShapes();
      const auto& origins = link->getCollisionOriginTransforms();
      if (shapes.empty())
        continue;  // pure frames (e.g. tool0) carry no geometry

      nlohmann::json lj;
      lj["link"] = link->getName();
      nlohmann::json sj = nlohmann::json::array();
      for (size_t s = 0; s < shapes.size(); ++s)
      {
        nlohmann::json o;
        if (!shapeToJson(o, shapes[s].get()))
          continue;
        if (s < origins.size())
          poseToJson(o, origins[s]);
        sj.push_back(std::move(o));
      }
      lj["shapes"] = std::move(sj);
      robot.push_back(std::move(lj));
    }
  }
  robot_scene_ = std::move(robot);
}

void ServerNode::broadcastScene()
{
  // Static robot geometry first, then the workpiece (if imported). Split into
  // two messages so re-import resends only the workpiece, not the whole robot.
  nlohmann::json scene;
  scene["type"] = "scene";
  scene["robot"] = robot_scene_;
  ws_->broadcastScene(scene.dump());

  broadcastWorkpiece();
}

void ServerNode::broadcastWorkpiece()
{
  nlohmann::json wp = workpieceJson();
  if (!wp.is_null())
    ws_->broadcastWorkpiece(wp.dump());
}

nlohmann::json ServerNode::workpieceJson()
{
  nlohmann::json j;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (model_id_.empty())
    return j;  // null — no workpiece imported yet

  j["type"] = "workpiece";
  j["model_id"] = model_id_;
  poseToJson(j, world_pose_);

  nlohmann::json mesh;
  const size_t vcount = workpiece_mesh_.vertices.size() / 3;
  const size_t tcount = workpiece_mesh_.triangles.size() / 3;
  mesh["vcount"] = vcount;
  mesh["tcount"] = tcount;
  mesh["data"] = meshToBase64(workpiece_mesh_.vertices.data(), vcount,
                              workpiece_mesh_.triangles.data(), tcount);
  j["mesh"] = std::move(mesh);

  nlohmann::json edges = nlohmann::json::array();
  for (const auto& e : workpiece_edges_)
  {
    nlohmann::json o;
    o["index"] = e.index;
    o["closed"] = e.closed;
    o["length"] = e.length;
    nlohmann::json pts = nlohmann::json::array();
    for (const auto& p : e.polyline)
    {
      pts.push_back(p.x());
      pts.push_back(p.y());
      pts.push_back(p.z());
    }
    o["points"] = std::move(pts);
    edges.push_back(std::move(o));
  }
  j["edges"] = std::move(edges);
  return j;
}

nlohmann::json ServerNode::robotStateJson()
{
  // Render thread only: reads display_state_, which is mutated by
  // applyPendingJointState() on this same thread (never concurrently).
  nlohmann::json j;
  j["type"] = "robot_state";
  nlohmann::json links = nlohmann::json::array();
  if (model_ && display_state_)
  {
    for (const moveit::core::LinkModel* link : model_->getLinkModels())
    {
      if (!link || link->getShapes().empty())
        continue;
      const Eigen::Isometry3d pose = display_state_->getGlobalLinkTransform(link->getName());
      const Eigen::Quaterniond q(pose.linear());
      nlohmann::json o;
      o["name"] = link->getName();
      o["pos"] = { pose.translation().x(), pose.translation().y(), pose.translation().z() };
      o["quat"] = { q.x(), q.y(), q.z(), q.w() };
      links.push_back(std::move(o));
    }
  }
  j["links"] = std::move(links);

  // Joint values and TCP pose for the jog panel (same 12 Hz stream, tiny payload).
  nlohmann::json joints = nlohmann::json::array();
  if (model_ && display_state_)
  {
    for (const auto& name : model_->getVariableNames())
    {
      nlohmann::json o;
      o["name"] = name;
      o["value"] = display_state_->getVariablePosition(name);
      joints.push_back(std::move(o));
    }
    if (model_->hasLinkModel("tool_tip"))
    {
      const Eigen::Isometry3d tf = display_state_->getGlobalLinkTransform("tool_tip");
      const Eigen::Vector3d rpy = tf.linear().eulerAngles(0, 1, 2);
      j["tcp"] = { tf.translation().x(), tf.translation().y(), tf.translation().z(),
                   rpy.x(), rpy.y(), rpy.z() };
    }
  }
  j["joints"] = std::move(joints);
  return j;
}

void ServerNode::highlightSelectedEdge()
{
  int selected;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    selected = selected_edge_;
  }
  std::lock_guard<std::mutex> lock(render_mutex_);
  renderer_->setHighlightedEdge(selected);
}

geometry_msgs::msg::Pose ServerNode::poseToMsg(const Eigen::Isometry3d& pose) const
{
  geometry_msgs::msg::Pose p;
  p.position.x = pose.translation().x();
  p.position.y = pose.translation().y();
  p.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

Eigen::Isometry3d ServerNode::parseWorldPose(const nlohmann::json& j) const
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  if (j.contains("position") && j["position"].is_object())
  {
    t.translation().x() = j["position"].value("x", 0.0);
    t.translation().y() = j["position"].value("y", 0.0);
    t.translation().z() = j["position"].value("z", 0.0);
  }
  if (j.contains("orientation") && j["orientation"].is_object())
  {
    const double x = j["orientation"].value("x", 0.0);
    const double y = j["orientation"].value("y", 0.0);
    const double z = j["orientation"].value("z", 0.0);
    const double w = j["orientation"].value("w", 1.0);
    const Eigen::Quaterniond q(w, x, y, z);
    if (q.norm() > 1e-8)
      t.linear() = q.normalized().toRotationMatrix();
  }
  return t;
}

// ---------------------------------------------------------------------------
// Render loop
// ---------------------------------------------------------------------------
void ServerNode::renderLoop()
{
  const auto period = std::chrono::milliseconds(1000 / fps_);
  size_t rendered = 0;
  size_t scene_pushes = 0;
  size_t last_bytes = 0;
  auto last_log = std::chrono::steady_clock::now();
  while (running_.load())
  {
    const auto start = std::chrono::steady_clock::now();
    // Capture/broadcast only when a client is watching. Production clients (and
    // an idle server) cost zero GL work this way; the WebGL debug viewer only
    // needs the cheap link-transform serialization, no GL capture.
    const bool want_frames = ws_->binarySubscriberCount() > 0;
    const bool want_scene = ws_->sceneSubscriberCount() > 0;

    std::vector<uint8_t> jpeg;
    std::string robot_state;
    if (want_frames || want_scene)
    {
      std::lock_guard<std::mutex> lock(render_mutex_);
      applyPendingJointState();
      if (want_frames)
        jpeg = renderer_->captureJpeg(quality_);
      if (want_scene)
        robot_state = robotStateJson().dump();
    }

    if (!jpeg.empty())
    {
      ws_->broadcastBinary(jpeg);
      last_bytes = jpeg.size();
      ++rendered;
    }
    if (!robot_state.empty())
    {
      ws_->broadcastScene(robot_state);
      ++scene_pushes;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::seconds(5))
    {
      if (rendered == 0 && scene_pushes == 0)
      {
        RCLCPP_INFO(get_logger(), "render: idle (no frame/scene subscribers)");
      }
      else
      {
        const double elapsed = std::chrono::duration<double>(now - last_log).count();
        RCLCPP_INFO(get_logger(), "render: %.1f fps, %zu bytes/frame, %.1f scene/s",
                    rendered / elapsed, last_bytes, scene_pushes / elapsed);
      }
      rendered = 0;
      scene_pushes = 0;
      last_log = now;
    }
    std::this_thread::sleep_until(start + period);
  }
}

void ServerNode::applyPendingJointState()
{
  // Called on the render thread while render_mutex_ is held.
  sensor_msgs::msg::JointState::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    msg = std::move(pending_joint_);
    pending_joint_.reset();
  }
  if (!msg || !display_state_)
    return;
  try
  {
    display_state_->setVariablePositions(msg->name, msg->position);
    display_state_->update();
    renderer_->updateRobot(*display_state_);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "joint_states mapping failed: %s", ex.what());
  }
}

// ---------------------------------------------------------------------------
// Seams (weld work units)
// ---------------------------------------------------------------------------
nlohmann::json ServerNode::seamsJsonLocked()
{
  nlohmann::json arr = nlohmann::json::array();
  for (const auto& sm : seams_)
  {
    nlohmann::json o;
    o["id"] = sm.id;
    o["name"] = sm.name;
    o["length"] = sm.length;
    o["closed"] = sm.closed;
    o["last_result"] = sm.last_result;
    o["last_run"] = sm.last_run;
    nlohmann::json parts = nlohmann::json::array();
    for (const auto& p : sm.parts)
    {
      nlohmann::json q;
      q["edge_index"] = p.edge_index;
      q["reversed"] = p.reversed;
      parts.push_back(std::move(q));
    }
    o["parts"] = std::move(parts);
    nlohmann::json poly = nlohmann::json::array();
    for (const auto& p : sm.polyline)
      poly.push_back({ p.x(), p.y(), p.z() });
    o["polyline"] = std::move(poly);
    arr.push_back(std::move(o));
  }
  return arr;
}

void ServerNode::broadcastSeamsLocked()
{
  nlohmann::json j;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    j["type"] = "seams";
    j["model_id"] = model_id_;
    j["seams"] = seamsJsonLocked();
  }
  broadcastJson(std::move(j));
}

bool ServerNode::seamMatchesModelLocked(const Seam& seam) const
{
  for (std::size_t i = 0; i < seam.parts.size(); ++i)
  {
    const auto it = std::find_if(workpiece_edges_.begin(), workpiece_edges_.end(),
                                 [&](const rosim::core::EdgeData& e) {
                                   return e.index == seam.parts[i].edge_index;
                                 });
    if (it == workpiece_edges_.end())
      return false;
    if (i >= seam.part_lengths.size() || i >= seam.part_starts.size())
      return false;
    if (std::abs(it->length - seam.part_lengths[i]) > 1e-3)
      return false;
    if ((it->polyline.front() - seam.part_starts[i]).norm() > 1e-3 ||
        (it->polyline.back() - seam.part_ends[i]).norm() > 1e-3)
      return false;
  }
  return true;
}

void ServerNode::saveSeamsLocked()
{
  if (model_id_.empty())
    return;
  nlohmann::json j;
  j["schema"] = "rosim.seams/1";
  j["model"]["file_path"] = file_path_;
  const Eigen::Quaterniond q(world_pose_.linear());
  j["model"]["pose"] = { world_pose_.translation().x(), world_pose_.translation().y(),
                         world_pose_.translation().z(), q.x(), q.y(), q.z(), q.w() };
  nlohmann::json arr = nlohmann::json::array();
  for (const auto& sm : seams_)
  {
    nlohmann::json o;
    o["id"] = sm.id;
    o["name"] = sm.name;
    o["length"] = sm.length;
    o["closed"] = sm.closed;
    o["last_result"] = sm.last_result;
    o["last_run"] = sm.last_run;
    nlohmann::json parts = nlohmann::json::array();
    for (std::size_t i = 0; i < sm.parts.size(); ++i)
    {
      nlohmann::json p;
      p["edge_index"] = sm.parts[i].edge_index;
      p["reversed"] = sm.parts[i].reversed;
      if (i < sm.part_lengths.size())
      {
        p["length"] = sm.part_lengths[i];
        p["start"] = { sm.part_starts[i].x(), sm.part_starts[i].y(), sm.part_starts[i].z() };
        p["end"] = { sm.part_ends[i].x(), sm.part_ends[i].y(), sm.part_ends[i].z() };
      }
      parts.push_back(std::move(p));
    }
    o["parts"] = std::move(parts);
    nlohmann::json poly = nlohmann::json::array();
    for (const auto& p : sm.polyline)
      poly.push_back({ p.x(), p.y(), p.z() });
    o["polyline"] = std::move(poly);
    arr.push_back(std::move(o));
  }
  j["seams"] = std::move(arr);

  try
  {
    const std::filesystem::path path(seam_store_path_);
    std::filesystem::create_directories(path.parent_path());
    std::ofstream f(path);
    if (!f)
    {
      RCLCPP_WARN(get_logger(), "cannot write seam store %s", seam_store_path_.c_str());
      return;
    }
    f << j.dump(1);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "seam store write failed: %s", e.what());
  }
}

void ServerNode::loadSeamsLocked()
{
  if (seam_store_path_.empty())
    return;
  std::ifstream f(seam_store_path_);
  if (!f)
    return;
  try
  {
    const auto j = nlohmann::json::parse(f);
    if (j.value("schema", "") != "rosim.seams/1")
      return;
    if (j["model"].value("file_path", "") != file_path_)
      return;
    const auto& pose = j["model"]["pose"];
    if (pose.size() != 7)
      return;
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    p.linear() = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix();
    if ((p.translation() - world_pose_.translation()).norm() > 1e-6)
      return;

    for (const auto& o : j["seams"])
    {
      Seam sm;
      sm.id = o.value("id", 0u);
      sm.name = o.value("name", std::string("seam"));
      sm.length = o.value("length", 0.0);
      sm.closed = o.value("closed", false);
      sm.last_result = o.value("last_result", std::string(""));
      sm.last_run = o.value("last_run", 0.0);
      for (const auto& pj : o["parts"])
      {
        rosim::core::SeamPart part;
        part.edge_index = pj.value("edge_index", 0u);
        part.reversed = pj.value("reversed", false);
        sm.parts.push_back(part);
        sm.part_lengths.push_back(pj.value("length", 0.0));
        const auto& a = pj["start"];
        const auto& b = pj["end"];
        sm.part_starts.push_back(Eigen::Vector3d(a[0], a[1], a[2]));
        sm.part_ends.push_back(Eigen::Vector3d(b[0], b[1], b[2]));
      }
      for (const auto& pj : o["polyline"])
        sm.polyline.push_back(Eigen::Vector3d(pj[0], pj[1], pj[2]));

      if (!seamMatchesModelLocked(sm))
      {
        RCLCPP_WARN(get_logger(),
                    "seam '%s' dropped: edge indices no longer match this model (OCC version?)",
                    sm.name.c_str());
        continue;
      }
      seam_counter_ = std::max(seam_counter_, sm.id);
      seams_.push_back(std::move(sm));
    }
    RCLCPP_INFO(get_logger(), "loaded %zu seams from %s", seams_.size(), seam_store_path_.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(get_logger(), "seam store read failed: %s", e.what());
  }
}

void ServerNode::handleCreateSeam(const nlohmann::json& j)
{
  std::string model_id;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_id = model_id_;
  }
  if (model_id.empty())
  {
    sendError("create_seam: no model imported");
    return;
  }
  if (!j.contains("edge_indices") || !j["edge_indices"].is_array() || j["edge_indices"].empty())
  {
    sendError("create_seam: no edges selected");
    return;
  }
  auto req = std::make_shared<CreateSeam::Request>();
  req->model_id = model_id;
  for (const auto& v : j["edge_indices"])
    if (v.is_number_integer())
      req->edge_indices.push_back(v.get<uint32_t>());
  req->endpoint_tolerance = j.value("tolerance", 0.0);
  const std::string wanted_name = j.value("name", "");

  create_seam_client_->async_send_request(
      req, [this, wanted_name](rclcpp::Client<CreateSeam>::SharedFuture future) {
        auto resp = future.get();
        if (!resp->success)
        {
          sendError("create_seam: " + resp->message);
          return;
        }
        Seam sm;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          sm.id = ++seam_counter_;
          sm.name = wanted_name.empty() ? ("焊缝 #" + std::to_string(sm.id)) : wanted_name;
          sm.length = resp->length;
          sm.closed = resp->closed;
          for (const auto& p : resp->parts)
            sm.parts.push_back({ p.edge_index, p.reversed });
          for (const auto& p : resp->polyline)
            sm.polyline.push_back(Eigen::Vector3d(p.x, p.y, p.z));

          // fingerprints so a later reload can detect an edge-index shift
          for (const auto& part : sm.parts)
          {
            const auto it = std::find_if(workpiece_edges_.begin(), workpiece_edges_.end(),
                                         [&](const rosim::core::EdgeData& e) {
                                           return e.index == part.edge_index;
                                         });
            if (it == workpiece_edges_.end())
            {
              sendError("create_seam: planner returned an unknown edge");
              return;
            }
            sm.part_lengths.push_back(it->length);
            sm.part_starts.push_back(it->polyline.front());
            sm.part_ends.push_back(it->polyline.back());
          }
          seams_.push_back(std::move(sm));
          saveSeamsLocked();
        }
        broadcastSeamsLocked();
        broadcastStateLocked();
        broadcastJson(statusJson(true, "seam created: " + seams_.back().name));
      });
}

void ServerNode::handleDeleteSeam(const nlohmann::json& j)
{
  const uint32_t id = j.value("id", 0u);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = std::find_if(seams_.begin(), seams_.end(),
                                 [id](const Seam& s) { return s.id == id; });
    if (it == seams_.end())
    {
      sendError("delete_seam: unknown id");
      return;
    }
    seams_.erase(it);
    if (selected_seam_ == static_cast<int>(id))
      selected_seam_ = -1;
    saveSeamsLocked();
  }
  broadcastSeamsLocked();
  broadcastStateLocked();
  broadcastJson(statusJson(true, "seam deleted"));
}

void ServerNode::handleRenameSeam(const nlohmann::json& j)
{
  const uint32_t id = j.value("id", 0u);
  const std::string name = j.value("name", "");
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto& s : seams_)
      if (s.id == id)
      {
        s.name = name.empty() ? s.name : name;
        saveSeamsLocked();
        break;
      }
  }
  broadcastSeamsLocked();
  broadcastJson(statusJson(true, "seam renamed"));
}

void ServerNode::handleReorderSeams(const nlohmann::json& j)
{
  if (!j.contains("order") || !j["order"].is_array())
    return;
  std::vector<uint32_t> order;
  for (const auto& v : j["order"])
    if (v.is_number_integer())
      order.push_back(v.get<uint32_t>());
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<Seam> reordered;
    for (const uint32_t id : order)
    {
      const auto it = std::find_if(seams_.begin(), seams_.end(),
                                   [id](const Seam& s) { return s.id == id; });
      if (it != seams_.end())
        reordered.push_back(*it);
    }
    for (const auto& s : seams_)  // keep any seam not listed at the end
      if (std::none_of(order.begin(), order.end(), [&](uint32_t id) { return id == s.id; }))
        reordered.push_back(s);
    seams_ = std::move(reordered);
    saveSeamsLocked();
  }
  broadcastSeamsLocked();
}

void ServerNode::handleSelectSeam(const nlohmann::json& j)
{
  const uint32_t id = j.value("id", 0u);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto it = std::find_if(seams_.begin(), seams_.end(),
                                 [id](const Seam& s) { return s.id == id; });
    if (it == seams_.end())
    {
      sendError("select_seam: unknown id");
      return;
    }
    selected_seam_ = static_cast<int>(id);
    selected_edge_ = -1;
  }
  highlightSelectedEdge();
  broadcastStateLocked();
}

void ServerNode::handleFollowSeams(const nlohmann::json& j)
{
  std::string model_id;
  std::vector<Seam> selected;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    model_id = model_id_;
    if (j.contains("ids") && j["ids"].is_array() && !j["ids"].empty())
    {
      for (const auto& v : j["ids"])
        if (v.is_number_integer())
        {
          const uint32_t id = v.get<uint32_t>();
          const auto it = std::find_if(seams_.begin(), seams_.end(),
                                       [id](const Seam& s) { return s.id == id; });
          if (it != seams_.end())
            selected.push_back(*it);
        }
    }
    else
    {
      selected = seams_;  // no selection: run the whole list in order
    }
  }
  if (model_id.empty())
  {
    sendError("follow_seams: no model imported");
    return;
  }
  if (selected.empty())
  {
    sendError("follow_seams: no seams to run");
    return;
  }
  if (!follow_seams_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    sendError("follow_seams: planner not ready");
    return;
  }

  FollowSeamsAction::Goal goal;
  goal.model_id = model_id;
  goal.orientation_mode = static_cast<uint8_t>(j.value("orientation_mode", 0));
  goal.tangent_tilt = j.value("tangent_tilt", kDefaultTangentTilt);
  goal.tool_offset = j.value("tool_offset", 0.0);
  goal.approach_distance = j.value("approach_distance", 0.08);
  goal.max_waypoint_spacing = j.value("max_waypoint_spacing", 0.005);

  const uint8_t direction = static_cast<uint8_t>(j.value("direction", 0));
  std::vector<uint32_t> ids;
  ids.reserve(selected.size());
  for (const auto& sm : selected)
  {
    rosim_interfaces::msg::SeamSpec spec;
    spec.edge_indices.reserve(sm.parts.size());
    for (const auto& part : sm.parts)
      spec.edge_indices.push_back(part.edge_index);
    spec.direction = direction;
    goal.seams.push_back(std::move(spec));
    ids.push_back(sm.id);
  }

  auto options = rclcpp_action::Client<FollowSeamsAction>::SendGoalOptions();
  options.goal_response_callback = [this](const std::shared_ptr<FollowSeamsGoalHandle> handle) {
    broadcastJson(statusJson(static_cast<bool>(handle),
                             handle ? "seam batch accepted" : "seam batch rejected (busy?)"));
  };
  options.feedback_callback = [this](std::shared_ptr<FollowSeamsGoalHandle>,
                                     const std::shared_ptr<const FollowSeamsAction::Feedback> fb) {
    nlohmann::json o;
    o["type"] = "seams_feedback";
    o["progress"] = fb->progress;
    o["seam_index"] = fb->seam_index;
    o["seam_count"] = fb->seam_count;
    o["phase"] = fb->phase;
    o["note"] = fb->note;
    broadcastJson(std::move(o));
  };
  options.result_callback = [this, ids](const FollowSeamsGoalHandle::WrappedResult& res) {
    const auto& r = *res.result;
    const std::set<std::size_t> failed(r.failed_indices.begin(), r.failed_indices.end());
    nlohmann::json failed_ids = nlohmann::json::array();
    for (const std::size_t k : failed)
      if (k < ids.size())
        failed_ids.push_back(ids[k]);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double now = this->now().seconds();
      for (std::size_t k = 0; k < ids.size(); ++k)
        for (auto& sm : seams_)
          if (sm.id == ids[k])
          {
            sm.last_result = failed.count(k) ? "failed" : "ok";
            sm.last_run = now;
          }
      saveSeamsLocked();
    }

    nlohmann::json o;
    o["type"] = "seams_result";
    o["success"] = r.success;
    o["executed"] = r.executed;
    o["failed"] = r.failed;
    o["message"] = r.message;
    o["failed_ids"] = std::move(failed_ids);
    o["failed_messages"] = r.failed_messages;
    broadcastJson(std::move(o));
    broadcastSeamsLocked();  // refreshed last_result marks
    publishTcpPath(r.trajectory);
  };

  follow_seams_client_->async_send_goal(goal, options);
  broadcastJson(statusJson(true, "running " + std::to_string(ids.size()) + " seams"));
}

}  // namespace rosim::server


