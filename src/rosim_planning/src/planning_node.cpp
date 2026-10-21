// rosim_planning node. Owns the RobotModel and PlanningScene, imports
// workpieces, serves the FollowEdge action plus MoveToPose / GoHome / JogJoint /
// MoveWorkpiece / SetPlanner / SetSpeedScale, and executes planned trajectories
// by publishing to /arm_controller/joint_trajectory.

#include "rosim_planning/planning_node.h"

#include <cstdlib>

#include "rosim_planning/fallback_planner.h"
#include "rosim_planning/trajectory_timing.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

#include <dlfcn.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <shape_msgs/msg/mesh.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometric_shapes/shape_operations.h>

#include "rosim_core/seam_builder.h"
#include "rosim_planning/ompl_planner.h"

namespace rosim::planning
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("rosim.planning_node");

/// RAII flag reset: handlers arm the exclusivity flag with busy_.exchange(true)
/// and this guard releases it on every exit path (including early returns).
/// Replaces three identical locals that previously duplicated the pattern.
struct BusyGuard
{
  std::atomic<bool>& flag;
  ~BusyGuard() { flag = false; }
};

/// geometry_msgs::msg::Pose -> Eigen::Isometry3d. A zero/near-zero orientation
/// quaternion is treated as identity (no rotation) rather than a NaN matrix.
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                       pose.orientation.z);
  if (q.norm() > 1e-8)
    t.linear() = q.normalized().toRotationMatrix();
  return t;
}
}  // namespace

PlanningNode::PlanningNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("rosim_planning", options)
{
}

void PlanningNode::init()
{
  allow_fallback_ = std::getenv("ROSIM_RL_NO_FALLBACK") == nullptr;
  if (const char* d = std::getenv("ROSIM_RL_MAX_DELTA"))
  {
    rl_max_delta_ = std::atof(d);
  }
  registerOmplPlanner();  // idempotent; ensures "ompl" exists even if only this lib is linked

  // Optional extension point: dlopen extra planner libraries (e.g.
  // librosim_rl.so). Their static MotionPlannerFactory registrations then run
  // inside this process, making e.g. set_planner "rl" work without rebuilding
  // this node.
  std::vector<std::string> extra_libs;
  declare_parameter<std::vector<std::string>>("planner_libraries", std::vector<std::string>());
  // IK / 可行性扫描的并行线程数：0=自动(min(hw-2,12))，1=完全串行（老行为），
  // 便于在同一台机器上做 A/B 对比或给别的进程让出核心。
  declare_parameter<int>("ik_threads", 0);
  declare_parameter<double>("jerk_limit", 100.0);
  get_parameter("jerk_limit", jerk_limit_);
  // 每次随边任务里允许走几次顺序 walk（1=只在第一次尝试；<=0=每次调用都走，老行为）。
  declare_parameter<int>("ik_walk_attempts", 1);
  // 是否先用一次便宜的「一致性限幅单解」再考虑 walk。
  declare_parameter<bool>("ik_cheap_solve_first", true);
  // 重试类候选（后续逼近/退刀变体、备用站位）用的随机种子数；首次尝试仍用 60。
  declare_parameter<int>("ik_seed_attempts_retry", 12);
  get_parameter("planner_libraries", extra_libs);
  {
    for (const auto& lib : extra_libs)
    {
      void* handle = dlopen(lib.c_str(), RTLD_NOW | RTLD_GLOBAL);
      if (!handle)
        RCLCPP_WARN(LOGGER, "dlopen('%s') failed: %s", lib.c_str(), dlerror());
      else
        RCLCPP_INFO(LOGGER, "loaded planner library '%s'", lib.c_str());
    }
  }

  loadRobotModel();
  setupInterfaces();
  RCLCPP_INFO(LOGGER, "rosim_planning ready (planner: %s, group: %s)",
              active_planner_.c_str(), group_.c_str());
}

PlanningNode::~PlanningNode()
{
  cancel_ = true;
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable())
    worker_.join();
}

void PlanningNode::loadRobotModel()
{
  // Reads robot_description + robot_description_semantic parameters and loads
  // the kinematics solvers from the kinematics.yaml params on this node.
  auto shared_this = shared_from_this();
  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = true;
  robot_model_loader::RobotModelLoader loader(shared_this, opts);
  model_ = loader.getModel();
  if (!model_)
  {
    RCLCPP_FATAL(LOGGER,
                 "failed to load robot model — are robot_description AND "
                 "robot_description_semantic set on this node?");
    throw std::runtime_error("robot model load failed");
  }
  if (!model_->hasJointModelGroup(group_))
  {
    RCLCPP_FATAL(LOGGER, "model has no group '%s'", group_.c_str());
    throw std::runtime_error("missing joint model group");
  }

  scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
  current_state_ = std::make_shared<moveit::core::RobotState>(model_);
  current_state_->setToDefaultValues();
  current_state_->setToDefaultValues(model_->getJointModelGroup(group_), "home");

  // 默认规划器：auto = 直线（接近/退刀，确定性）→ RL（需要绕障时）→ OMPL（兜底）。
  // 焊缝段本身就是确定性采样，不经过这里。
  declare_parameter<std::string>("default_planner", "auto");
  get_parameter("default_planner", active_planner_);
  planner_ = makePlanner(active_planner_);
  follower_ = std::make_unique<EdgeFollower>(shared_from_this(), model_, planner_);
  applyFollowerConfig();
}

void PlanningNode::applyFollowerConfig()
{
  if (!follower_)
    return;
  EdgeFollower::Config cfg;
  cfg.parallel_threads = static_cast<int>(get_parameter("ik_threads").as_int());
  cfg.walk_attempts = static_cast<int>(get_parameter("ik_walk_attempts").as_int());
  cfg.cheap_solve_first = get_parameter("ik_cheap_solve_first").as_bool();
  cfg.seed_attempts_retry = static_cast<int>(get_parameter("ik_seed_attempts_retry").as_int());
  follower_->setConfig(cfg);
}

void PlanningNode::setupInterfaces()
{
  auto shared_this = shared_from_this();

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        try
        {
          current_state_->setVariablePositions(msg->name, msg->position);
          current_state_->update();  // consumers read global link transforms
          have_state_ = true;
        }
        catch (const std::exception& ex)
        {
          RCLCPP_WARN_THROTTLE(LOGGER, *get_clock(), 5000,
                               "joint_states mapping failed: %s", ex.what());
        }
      });

  planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(1).transient_local());
  display_traj_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 1);
  trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 1);

  import_model_srv_ = create_service<rosim_interfaces::srv::ImportModel>(
      "~/import_model",
      [this](const rosim_interfaces::srv::ImportModel::Request::SharedPtr req,
             const rosim_interfaces::srv::ImportModel::Response::SharedPtr res) {
        handleImportModel(req, res);
      });
  move_workpiece_srv_ = create_service<rosim_interfaces::srv::MoveWorkpiece>(
      "~/move_workpiece",
      [this](const rosim_interfaces::srv::MoveWorkpiece::Request::SharedPtr req,
             const rosim_interfaces::srv::MoveWorkpiece::Response::SharedPtr res) {
        handleMoveWorkpiece(req, res);
      });
  set_speed_srv_ = create_service<rosim_interfaces::srv::SetSpeedScale>(
      "~/set_speed_scale",
      [this](const rosim_interfaces::srv::SetSpeedScale::Request::SharedPtr req,
             const rosim_interfaces::srv::SetSpeedScale::Response::SharedPtr res) {
        handleSetSpeedScale(req, res);
      });
  set_planner_srv_ = create_service<rosim_interfaces::srv::SetPlanner>(
      "~/set_planner",
      [this](const rosim_interfaces::srv::SetPlanner::Request::SharedPtr req,
             const rosim_interfaces::srv::SetPlanner::Response::SharedPtr res) {
        handleSetPlanner(req, res);
      });
  plan_joint_srv_ = create_service<rosim_interfaces::srv::PlanJoint>(
      "~/plan_joint",
      [this](const rosim_interfaces::srv::PlanJoint::Request::SharedPtr req,
             const rosim_interfaces::srv::PlanJoint::Response::SharedPtr res) {
        handlePlanJoint(req, res);
      });
  move_to_pose_srv_ = create_service<rosim_interfaces::srv::MoveToPose>(
      "~/move_to_pose",
      [this](const rosim_interfaces::srv::MoveToPose::Request::SharedPtr req,
             const rosim_interfaces::srv::MoveToPose::Response::SharedPtr res) {
        handleMoveToPose(req, res);
      });
  go_home_srv_ = create_service<rosim_interfaces::srv::GoHome>(
      "~/go_home",
      [this](const rosim_interfaces::srv::GoHome::Request::SharedPtr req,
             const rosim_interfaces::srv::GoHome::Response::SharedPtr res) {
        handleGoHome(req, res);
      });
  jog_joint_srv_ = create_service<rosim_interfaces::srv::JogJoint>(
      "~/jog_joint",
      [this](const rosim_interfaces::srv::JogJoint::Request::SharedPtr req,
             const rosim_interfaces::srv::JogJoint::Response::SharedPtr res) {
        handleJogJoint(req, res);
      });
  jog_cartesian_srv_ = create_service<rosim_interfaces::srv::JogCartesian>(
      "~/jog_cartesian",
      [this](const rosim_interfaces::srv::JogCartesian::Request::SharedPtr req,
             const rosim_interfaces::srv::JogCartesian::Response::SharedPtr res) {
        handleJogCartesian(req, res);
      });
  create_seam_srv_ = create_service<rosim_interfaces::srv::CreateSeam>(
      "~/create_seam",
      [this](const rosim_interfaces::srv::CreateSeam::Request::SharedPtr req,
             const rosim_interfaces::srv::CreateSeam::Response::SharedPtr res) {
        handleCreateSeam(req, res);
      });

  action_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  follow_edge_action_ = rclcpp_action::create_server<FollowEdge>(
      shared_this, "~/follow_edge",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const FollowEdge::Goal> goal) {
        return handleGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleFollowEdge> goal_handle) {
        return handleCancel(std::move(goal_handle));
      },
      [this](const std::shared_ptr<GoalHandleFollowEdge> goal_handle) {
        handleAccepted(std::move(goal_handle));
      },
      rcl_action_server_get_default_options(), action_group_);

  follow_seams_action_ = rclcpp_action::create_server<FollowSeams>(
      shared_this, "~/follow_seams",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const FollowSeams::Goal> goal) {
        return handleSeamsGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleFollowSeams> goal_handle) {
        return handleSeamsCancel(std::move(goal_handle));
      },
      [this](const std::shared_ptr<GoalHandleFollowSeams> goal_handle) {
        handleSeamsAccepted(std::move(goal_handle));
      },
      rcl_action_server_get_default_options(), action_group_);
}

std::string PlanningNode::nextModelId() const
{
  return "workpiece_" + std::to_string(model_counter_ + 1);
}

// ---------------------------------------------------------------------------
// ImportModel
// ---------------------------------------------------------------------------
void PlanningNode::handleImportModel(
    const rosim_interfaces::srv::ImportModel::Request::SharedPtr req,
    const rosim_interfaces::srv::ImportModel::Response::SharedPtr res)
{
  // Import mutates workpieces_ and the shared PlanningScene; refuse while a
  // follow/plan task runs on the worker thread (same race the other exclusive
  // handlers guard against).
  if (busy_.exchange(true))
  {
    res->success = false;
    res->message = "another task is running";
    return;
  }
  BusyGuard guard{ busy_ };

  try
  {
    const Eigen::Isometry3d pose = poseToIsometry(req->world_pose);

    const std::string model_id = nextModelId();
    auto model = rosim::core::OccGeometryService::importModel(req->file_path, pose, model_id);

    // collision mesh: coarser tessellation is plenty for collision checking
    rosim::core::MeshData mesh;
    rosim::core::OccGeometryService::tessellate(model->shape, 2e-3, 0.5, mesh);
    shapes::ShapePtr collision_shape =
        rosim::core::OccGeometryService::toCollisionMesh(mesh, pose);

    // Import replaces the workpiece (the WebUI shows exactly one): drop the
    // previous collision objects, otherwise a re-import at a new pose leaves
    // invisible ghost obstacles from the old pose in the planning scene.
    for (const auto& entry : workpieces_)
    {
      moveit_msgs::msg::CollisionObject remove;
      remove.header.frame_id = model_->getModelFrame();
      remove.id = entry.first;
      remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      scene_->processCollisionObjectMsg(remove);
    }
    workpieces_.clear();

    if (collision_shape)
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = model_->getModelFrame();
      obj.id = model_id;
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      obj.meshes.push_back(shape_msgs::msg::Mesh());
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(collision_shape.get(), mesh_msg);
      obj.meshes.back() = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
      obj.mesh_poses.push_back(geometry_msgs::msg::Pose());

      scene_->processCollisionObjectMsg(obj);
      moveit_msgs::msg::PlanningScene scene_msg;
      scene_->getPlanningSceneMsg(scene_msg);
      planning_scene_pub_->publish(scene_msg);
    }

    ++model_counter_;
    workpieces_[model_id] = model;

    res->success = true;
    res->message = "imported " + std::to_string(model->edges.size()) + " edges";
    res->model_id = model_id;
    res->edges.resize(model->edges.size());
    for (size_t i = 0; i < model->edges.size(); ++i)
    {
      const auto& e = model->edges[i];
      res->edges[i].model_id = model_id;
      res->edges[i].edge_index = e.index;
      res->edges[i].length = e.length;
      res->edges[i].closed = e.closed;
      res->edges[i].polyline.resize(e.polyline.size());
      for (size_t k = 0; k < e.polyline.size(); ++k)
      {
        res->edges[i].polyline[k].x = e.polyline[k].x();
        res->edges[i].polyline[k].y = e.polyline[k].y();
        res->edges[i].polyline[k].z = e.polyline[k].z();
      }
    }
    RCLCPP_INFO(LOGGER, "imported model '%s' from %s (%zu edges)",
                model_id.c_str(), req->file_path.c_str(), model->edges.size());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("import failed: ") + ex.what();
    RCLCPP_ERROR(LOGGER, "import failed: %s", ex.what());
  }
}

// ---------------------------------------------------------------------------
// MoveWorkpiece
// ---------------------------------------------------------------------------
void PlanningNode::handleMoveWorkpiece(
    const rosim_interfaces::srv::MoveWorkpiece::Request::SharedPtr req,
    const rosim_interfaces::srv::MoveWorkpiece::Response::SharedPtr res)
{
  // Moving swaps the collision object; refuse while a plan/execution runs.
  if (busy_.exchange(true))
  {
    res->success = false;
    res->message = "another task is running";
    return;
  }
  BusyGuard guard{ busy_ };

  auto it = req->model_id.empty() ? workpieces_.begin() : workpieces_.find(req->model_id);
  if (it == workpieces_.end())
  {
    res->success = false;
    res->message = "unknown workpiece";
    return;
  }

  try
  {
    const Eigen::Isometry3d new_pose = poseToIsometry(req->world_pose);
    auto& model = *it->second;
    const Eigen::Isometry3d delta = new_pose * model.world_pose.inverse();

    // Edges are stored in the world frame; shift them with the part.
    for (auto& edge : model.edges)
      for (auto& p : edge.polyline)
        p = delta * p;
    model.world_pose = new_pose;

    // Replace the collision object at the new pose. The coarse collision mesh
    // is cheap to rebuild; the STEP file is not re-read.
    moveit_msgs::msg::CollisionObject remove;
    remove.header.frame_id = model_->getModelFrame();
    remove.id = it->first;
    remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene_->processCollisionObjectMsg(remove);

    rosim::core::MeshData mesh;
    rosim::core::OccGeometryService::tessellate(model.shape, 2e-3, 0.5, mesh);
    shapes::ShapePtr collision_shape =
        rosim::core::OccGeometryService::toCollisionMesh(mesh, new_pose);
    if (collision_shape)
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = model_->getModelFrame();
      obj.id = it->first;
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      obj.meshes.push_back(shape_msgs::msg::Mesh());
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(collision_shape.get(), mesh_msg);
      obj.meshes.back() = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
      obj.mesh_poses.push_back(geometry_msgs::msg::Pose());
      scene_->processCollisionObjectMsg(obj);
    }

    moveit_msgs::msg::PlanningScene scene_msg;
    scene_->getPlanningSceneMsg(scene_msg);
    planning_scene_pub_->publish(scene_msg);

    res->success = true;
    res->message = "moved " + it->first;
    RCLCPP_INFO(LOGGER, "moved workpiece '%s' to (%.3f, %.3f, %.3f)", it->first.c_str(),
                new_pose.translation().x(), new_pose.translation().y(),
                new_pose.translation().z());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("move failed: ") + ex.what();
    RCLCPP_ERROR(LOGGER, "move workpiece failed: %s", ex.what());
  }
}

// ---------------------------------------------------------------------------
// SetSpeedScale / SetPlanner
// ---------------------------------------------------------------------------
void PlanningNode::handleCreateSeam(
    const rosim_interfaces::srv::CreateSeam::Request::SharedPtr req,
    const rosim_interfaces::srv::CreateSeam::Response::SharedPtr res)
{
  const auto it = workpieces_.find(req->model_id);
  if (it == workpieces_.end())
  {
    res->success = false;
    res->message = "model not imported: " + req->model_id;
    return;
  }
  if (req->edge_indices.empty())
  {
    res->success = false;
    res->message = "no edges selected";
    return;
  }
  std::vector<uint32_t> idx(req->edge_indices.begin(), req->edge_indices.end());
  const double tol = req->endpoint_tolerance > 0.0 ? req->endpoint_tolerance : 5e-4;
  const auto seam = rosim::core::buildSeam(it->second->edges, idx, tol);
  if (!seam.ok)
  {
    res->success = false;
    res->message = seam.message;
    return;
  }
  res->success = true;
  res->message = seam.message;
  res->length = seam.length;
  res->closed = seam.closed;
  res->parts.resize(seam.parts.size());
  for (std::size_t i = 0; i < seam.parts.size(); ++i)
  {
    res->parts[i].edge_index = seam.parts[i].edge_index;
    res->parts[i].reversed = seam.parts[i].reversed;
  }
  res->polyline.resize(seam.polyline.size());
  for (std::size_t i = 0; i < seam.polyline.size(); ++i)
  {
    res->polyline[i].x = seam.polyline[i].x();
    res->polyline[i].y = seam.polyline[i].y();
    res->polyline[i].z = seam.polyline[i].z();
  }
  RCLCPP_INFO(get_logger(), "create_seam: %zu edges -> %.3f m%s (%s)", idx.size(), seam.length,
              seam.closed ? " closed" : "", seam.message.c_str());
}

void PlanningNode::handleSetSpeedScale(
    const rosim_interfaces::srv::SetSpeedScale::Request::SharedPtr req,
    const rosim_interfaces::srv::SetSpeedScale::Response::SharedPtr res)
{
  if (req->scale < 0.05 || req->scale > 1.0)
  {
    res->success = false;
    res->message = "scale must be within [0.05, 1.0]";
    return;
  }
  speed_scale_ = req->scale;
  res->success = true;
  res->message = "speed scale = " + std::to_string(speed_scale_);
}

void PlanningNode::handlePlanJoint(
    const rosim_interfaces::srv::PlanJoint::Request::SharedPtr req,
    const rosim_interfaces::srv::PlanJoint::Response::SharedPtr res)
{
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();
  try
  {
    PlanRequest preq(*current_state_);
    preq.goal_joint_positions = req->goal;
    preq.scene = scene_;               // same scene the weld pipeline plans in
    preq.group = group_;
    preq.allowed_planning_time = 5.0;
    preq.velocity_scaling = speed_scale_;
    preq.acceleration_scaling = speed_scale_;

    PlanResult result;
    const bool ok = planner_ && planner_->plan(preq, result) && result.success;
    std::string note;
    if (auto* fb = dynamic_cast<FallbackPlanner*>(planner_.get()))
      note = fb->lastFallback() ? fb->lastNote() : std::string();

    const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    res->success = ok;
    res->message = ok ? ((note.empty() ? active_planner_ : note) + ", " +
                         std::to_string(result.trajectory->size()) + " points, " +
                         std::to_string(static_cast<int>(ms)) + " ms")
                      : ("plan failed: " + result.error);
    if (ok)
    {
      moveit_msgs::msg::RobotTrajectory msg;
      result.trajectory->getRobotTrajectoryMsg(msg);
      res->trajectory = msg.joint_trajectory;
    }
    RCLCPP_INFO(LOGGER, "plan_joint: %s", res->message.c_str());
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = std::string("plan_joint threw: ") + ex.what();
  }
}

std::string PlanningNode::plannerLabel() const
{
  // 只报"真正出解的那一级"：auto / rl 这些只是选择项，读者要的是 line/rl/ompl。
  std::string who = active_planner_;
  std::string why;
  if (auto* fb = dynamic_cast<const FallbackPlanner*>(planner_.get()))
  {
    if (!fb->lastSuccessName().empty())
      who = fb->lastSuccessName();
    if (fb->lastFallback() && !fb->lastNote().empty())
      why = "（" + fb->lastNote() + "）";
  }
  return who + why;
}

MotionPlannerPtr PlanningNode::makePlanner(const std::string& type)
{
  const std::string key = type + (allow_fallback_ ? "+fb" : "");
  const auto cached = planner_cache_.find(key);
  if (cached != planner_cache_.end())
    return cached->second;                      // 复用：不销毁旧实例

  MotionPlannerPtr next;
  if (type == "auto")
  {
    // 三段分工：先试确定性直线（可复现、3 ms 量级），直线走不了（碰撞/IK 分支
    // 跳变）再让 RL 绕障，仍不行才用 OMPL。RL 的服务节点没起也不会卡住——
    // RlRemotePlanner 会直接返回失败，链条继续往下走。
    auto ompl = MotionPlannerFactory::create("ompl", shared_from_this(), model_);
    auto rl = MotionPlannerFactory::create("rl", shared_from_this(), model_);
    auto rl_chain = std::make_shared<FallbackPlanner>(rl, ompl, rl_max_delta_, get_logger());
    next = std::make_shared<FallbackPlanner>(
        MotionPlannerFactory::create("line", shared_from_this(), model_), rl_chain, 1e9,
        get_logger());
  }
  else
  {
    next = MotionPlannerFactory::create(type, shared_from_this(), model_);
    // 非 OMPL 的规划器外面套回退：焊缝跟随的接近/退刀段也直接调 plan()，
    // 规则必须对所有调用方一致。
    if (type != "ompl" && allow_fallback_)
    {
      auto fallback = MotionPlannerFactory::create("ompl", shared_from_this(), model_);
      const double gate = (type == "rl") ? rl_max_delta_ : 1e9;
      next = std::make_shared<FallbackPlanner>(next, fallback, gate, get_logger());
    }
  }
  planner_cache_[key] = next;
  return next;
}

void PlanningNode::handleSetPlanner(
    const rosim_interfaces::srv::SetPlanner::Request::SharedPtr req,
    const rosim_interfaces::srv::SetPlanner::Response::SharedPtr res)
{
  try
  {
    auto next = makePlanner(req->planner_type);
    planner_ = next;
    follower_ = std::make_unique<EdgeFollower>(shared_from_this(), model_, planner_);
    applyFollowerConfig();  // switching planners must keep the thread budget
    active_planner_ = req->planner_type;
    res->success = true;
    res->message = "active planner: " + active_planner_;
  }
  catch (const std::exception& ex)
  {
    res->success = false;
    res->message = ex.what();
  }
}

// ---------------------------------------------------------------------------
// MoveToPose (minimal OMPL loop, also handy for smoke tests)
// ---------------------------------------------------------------------------
void PlanningNode::handleMoveToPose(
    const rosim_interfaces::srv::MoveToPose::Request::SharedPtr req,
    const rosim_interfaces::srv::MoveToPose::Response::SharedPtr res)
{
  if (busy_.exchange(true))
  {
    res->success = false;
    res->message = "another task is running";
    return;
  }
  BusyGuard guard{ busy_ };

  const Eigen::Isometry3d target = poseToIsometry(req->target_pose);

  const auto start = snapshotCurrentState();
  moveit::core::RobotStatePtr solution;
  if (!follower_->solveIkWithRailAllocation(target, start, scene_, solution))
  {
    res->success = false;
    res->message = "no collision-free IK for target pose (arm and rail envelope)";
    return;
  }

  PlanRequest preq(start);
  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_);
  solution->copyJointGroupPositions(group, preq.goal_joint_positions);
  preq.scene = scene_;
  preq.group = group_;
  preq.velocity_scaling = speed_scale_;
  preq.acceleration_scaling = speed_scale_;

  PlanResult result;
  if (!planner_->plan(preq, result) || !result.success)
  {
    res->success = false;
    res->message = "OMPL failed: " + result.error;
    return;
  }

  publishTrajectory(*result.trajectory, true);
  res->success = true;
  res->message = "executing planned motion (" +
                 std::to_string(result.trajectory->size()) + " waypoints)";
}

// ---------------------------------------------------------------------------
// GoHome: OMPL-planned, collision-free return to the SRDF "home" state
// ---------------------------------------------------------------------------
void PlanningNode::handleGoHome(const rosim_interfaces::srv::GoHome::Request::SharedPtr /*req*/,
                                const rosim_interfaces::srv::GoHome::Response::SharedPtr res)
{
  if (busy_.exchange(true))
  {
    res->success = false;
    res->message = "another task is running";
    return;
  }
  BusyGuard guard{ busy_ };

  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_);

  moveit::core::RobotState goal = snapshotCurrentState();
  if (!goal.setToDefaultValues(group, "home"))
  {
    res->success = false;
    res->message = "SRDF has no named state 'home'";
    return;
  }
  goal.update();

  auto log_contacts = [&](const char* label, const moveit::core::RobotState& state) {
    collision_detection::CollisionRequest creq;
    creq.contacts = true;
    creq.max_contacts = 8;
    collision_detection::CollisionResult cres;
    scene_->checkCollision(creq, cres, state);
    for (const auto& entry : cres.contacts)
      for (const auto& c : entry.second)
        RCLCPP_WARN(LOGGER, "%s contact: %s <-> %s depth=%.4f at (%.3f, %.3f, %.3f)", label,
                    entry.first.first.c_str(), entry.first.second.c_str(), c.depth, c.pos.x(),
                    c.pos.y(), c.pos.z());
    return cres.collision;
  };

  // never force into a bad state: if home is currently in collision (e.g. a
  // workpiece was dropped on it), refuse instead of planning into it
  if (log_contacts("home", goal))
  {
    res->success = false;
    res->message = "home state is in collision with the current scene";
    return;
  }

  const moveit::core::RobotState start = snapshotCurrentState();
  // OMPL rejects out-of-collision starts with a bare -26 (START_STATE_INVALID);
  // name the cause (typically the torch grazing a seam end) so the operator
  // knows to jog clear instead of retrying homing.
  if (log_contacts("current state", start))
  {
    res->success = false;
    res->message = "current state is in collision; jog clear before homing";
    return;
  }

  std::vector<double> now, home;
  start.copyJointGroupPositions(group, now);
  goal.copyJointGroupPositions(group, home);
  double max_diff = 0.0;
  for (size_t i = 0; i < now.size(); ++i)
    max_diff = std::max(max_diff, std::fabs(now[i] - home[i]));
  if (max_diff < 1e-3)
  {
    res->success = true;
    res->message = "already at home";
    return;
  }

  PlanRequest preq(start);
  preq.goal_joint_positions = home;
  preq.scene = scene_;
  preq.group = group_;
  preq.velocity_scaling = speed_scale_;
  preq.acceleration_scaling = speed_scale_;

  PlanResult result;
  if (!planner_->plan(preq, result) || !result.success)
  {
    res->success = false;
    res->message = "OMPL failed: " + result.error;
    return;
  }

  publishTrajectory(*result.trajectory, true);
  res->success = true;
  res->message = "OMPL planned path to home executing (" +
                 std::to_string(result.trajectory->size()) + " waypoints)";
}

// ---------------------------------------------------------------------------
// JogJoint: small single-joint nudge, limit-clamped and collision-checked
// ---------------------------------------------------------------------------
void PlanningNode::handleJogJoint(const rosim_interfaces::srv::JogJoint::Request::SharedPtr req,
                                  const rosim_interfaces::srv::JogJoint::Response::SharedPtr res)
{
  if (busy_.exchange(true))
  {
    res->success = false;
    res->message = "another task is running";
    return;
  }
  BusyGuard guard{ busy_ };

  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_);
  const auto& var_names = group->getVariableNames();
  const auto it = std::find(var_names.begin(), var_names.end(), req->joint_name);
  const moveit::core::JointModel* joint_model = model_->getJointModel(req->joint_name);
  if (it == var_names.end() || joint_model == nullptr ||
      joint_model->getVariableCount() != 1)
  {
    res->success = false;
    res->message = "unknown or multi-DOF joint '" + req->joint_name + "'";
    return;
  }
  const size_t idx = static_cast<size_t>(it - var_names.begin());
  // The rail is prismatic: its jog unit is metres, not radians, and a full
  // 0.5 m nudge is far too coarse for the track.
  const bool prismatic = joint_model->getType() == moveit::core::JointModel::PRISMATIC;
  const double max_delta = prismatic ? 0.2 : 0.5;
  if (std::fabs(req->delta) > max_delta)
  {
    res->success = false;
    res->message = prismatic ? "delta too large (max 0.2 m per jog)"
                             : "delta too large (max 0.5 rad per jog)";
    return;
  }

  const moveit::core::RobotState start = snapshotCurrentState();
  std::vector<double> pos;
  start.copyJointGroupPositions(group, pos);
  const double before = pos[idx];
  pos[idx] += req->delta;

  moveit::core::RobotState target(start);
  target.setVariablePositions(var_names, pos);
  target.enforceBounds();  // clamp to limits
  target.update();
  std::vector<double> clamped;
  target.copyJointGroupPositions(group, clamped);
  const double moved = clamped[idx] - before;
  if (std::fabs(moved) < 1e-6)
  {
    res->success = false;
    res->message = req->joint_name + " is at its joint limit";
    return;
  }

  // collision-check the interpolated path (endpoint included)
  moveit::core::RobotState probe(start);
  const int steps = std::max(2, static_cast<int>(std::ceil(std::fabs(moved) / 0.05)) + 1);
  for (int k = 1; k <= steps; ++k)
  {
    const double s = static_cast<double>(k) / steps;
    std::vector<double> p(pos.size());
    for (size_t i = 0; i < pos.size(); ++i)
      p[i] = (i == idx) ? before + s * moved : pos[i];
    probe.setVariablePositions(var_names, p);
    probe.update();
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    scene_->checkCollision(creq, cres, probe);
    if (cres.collision)
    {
      res->success = false;
      res->message = "jog would collide at " + std::to_string(100 * k / steps) + "% of travel";
      return;
    }
  }

  robot_trajectory::RobotTrajectory traj(model_, group);
  traj.addSuffixWayPoint(start, 0.0);
  traj.addSuffixWayPoint(target, 0.0);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.002, 0.01, 0.001);
  if (!totg.computeTimeStamps(traj, speed_scale_, speed_scale_))
  {
    res->success = false;
    res->message = "time parameterization failed";
    return;
  }

  publishTrajectory(traj, true);
  res->success = true;
  res->message = req->joint_name + " jog " + std::to_string(moved) +
                 (prismatic ? " m executing" : " rad executing");
}

void PlanningNode::handleJogCartesian(
    const rosim_interfaces::srv::JogCartesian::Request::SharedPtr req,
    const rosim_interfaces::srv::JogCartesian::Response::SharedPtr res)
{
  if (busy_.exchange(true))
  {
    res->success = false;
    res->message = "another task is running";
    return;
  }
  BusyGuard guard{ busy_ };

  if (req->axis > 5)
  {
    res->success = false;
    res->message = "axis must be 0..5 (x,y,z,roll,pitch,yaw)";
    return;
  }
  const bool rotation = req->axis >= 3;
  const double max_delta = rotation ? 0.15 : 0.05;  // rad / m per jog step
  if (std::fabs(req->delta) > max_delta)
  {
    res->success = false;
    res->message = std::string("delta too large (max ") +
                   (rotation ? "0.15 rad" : "0.05 m") + " per cartesian jog";
    return;
  }

  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_);
  const moveit::core::RobotState start = snapshotCurrentState();
  static const std::string kTip = "tool_tip";  // the torch tip, not tool0

  Eigen::Isometry3d target = start.getGlobalLinkTransform(kTip);
  if (!rotation)
  {
    target.translation()[req->axis] += req->delta;
  }
  else
  {
    const Eigen::Vector3d axis = Eigen::Vector3d::Unit(req->axis - 3);
    target.linear() = Eigen::AngleAxisd(req->delta, axis).toRotationMatrix() * target.linear();
  }

  // Keep the rail where it is: a jog should feel local, not re-station the cell.
  moveit::core::RobotStatePtr solution;
  if (!follower_->solveIkMultiSeed(target, start, scene_, solution))
  {
    res->success = false;
    res->message = "target is out of reach or in collision";
    return;
  }

  // Collision-check the interpolated joint path (same rule as joint jog).
  std::vector<double> from, to;
  start.copyJointGroupPositions(group, from);
  solution->copyJointGroupPositions(group, to);
  double max_joint_delta = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i)
    max_joint_delta = std::max(max_joint_delta, std::fabs(to[i] - from[i]));

  moveit::core::RobotState probe(start);
  const auto& names = group->getVariableNames();
  const int steps = std::max(2, static_cast<int>(std::ceil(max_joint_delta / 0.05)) + 1);
  for (int k = 1; k <= steps; ++k)
  {
    const double s = static_cast<double>(k) / steps;
    std::vector<double> p(from.size());
    for (std::size_t i = 0; i < from.size(); ++i)
      p[i] = from[i] + s * (to[i] - from[i]);
    probe.setVariablePositions(names, p);
    probe.update();
    collision_detection::CollisionRequest creq;
    collision_detection::CollisionResult cres;
    scene_->checkCollision(creq, cres, probe);
    if (cres.collision)
    {
      res->success = false;
      res->message = "jog would collide at " + std::to_string(100 * k / steps) + "% of travel";
      return;
    }
  }

  robot_trajectory::RobotTrajectory traj(model_, group);
  traj.addSuffixWayPoint(start, 0.0);
  traj.addSuffixWayPoint(*solution, 0.0);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.002, 0.01, 0.001);
  if (!totg.computeTimeStamps(traj, speed_scale_, speed_scale_))
  {
    res->success = false;
    res->message = "time parameterization failed";
    return;
  }
  publishTrajectory(traj, true);
  res->success = true;
  res->message = std::string(rotation ? "cartesian jog " : "cartesian jog ") +
                 std::to_string(req->delta) + (rotation ? " rad" : " m") + " executing";
}

// ---------------------------------------------------------------------------
// FollowEdge action
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse PlanningNode::handleGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                     std::shared_ptr<const FollowEdge::Goal> goal)
{
  if (busy_.load())
  {
    RCLCPP_WARN(LOGGER, "follow_edge rejected: busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (workpieces_.find(goal->model_id) == workpieces_.end())
  {
    RCLCPP_WARN(LOGGER, "follow_edge rejected: unknown model '%s'", goal->model_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(LOGGER, "follow_edge accepted: model=%s edge=%u",
              goal->model_id.c_str(), goal->edge_index);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanningNode::handleCancel(
    const std::shared_ptr<GoalHandleFollowEdge> /*goal_handle*/)
{
  RCLCPP_INFO(LOGGER, "follow_edge cancel requested");
  cancel_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanningNode::handleAccepted(const std::shared_ptr<GoalHandleFollowEdge> goal_handle)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable())
    worker_.join();
  cancel_ = false;
  worker_ = std::thread([this, goal_handle]() { executeFollowEdge(goal_handle); });
}

void PlanningNode::executeFollowEdge(const std::shared_ptr<GoalHandleFollowEdge> goal_handle)
{
  busy_ = true;
  auto goal = goal_handle->get_goal();

  std::vector<double> planned_samples;  // flat x,y,z of the planned seam samples

  auto finish = [&](bool success, uint8_t code, double fraction, const std::string& message,
                    robot_trajectory::RobotTrajectory* trajectory,
                    uint8_t direction_used = FollowEdge::Result::DIRECTION_NONE) {
    auto result = std::make_shared<FollowEdge::Result>();
    result->success = success;
    result->error_code = code;
    result->cartesian_fraction = fraction;
    result->message = message;
    result->direction_used = direction_used;
    if (trajectory)
    {
      moveit_msgs::msg::RobotTrajectory rt_msg;
      trajectory->getRobotTrajectoryMsg(rt_msg);
      result->trajectory = rt_msg.joint_trajectory;
      result->tcp_samples = planned_samples;
    }
    const bool canceled = goal_handle->is_canceling();
    if (canceled)
      goal_handle->canceled(result);
    else if (success)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
    busy_ = false;
  };

  // ---- resolve the edge ---------------------------------------------------
  auto feedback = std::make_shared<FollowEdge::Feedback>();
  bool planner_reported = false;
  auto publish_feedback = [&](float progress, const std::string& phase) {
    feedback->progress = progress;
    feedback->phase = phase;
    // 规划完成、开始沿缝走的那一刻，报告实际用的规划器（只报一次）
    if (!planner_reported && phase == "following")
    {
      planner_reported = true;
      feedback->note = "规划完成：规划器 " + plannerLabel();
    }
    else
    {
      feedback->note.clear();
    }
    goal_handle->publish_feedback(feedback);
  };
  publish_feedback(0.0f, "resolving");

  const auto it = workpieces_.find(goal->model_id);
  if (it == workpieces_.end())
    return finish(false, FollowEdge::Result::ERROR_INVALID, 0.0, "model vanished", nullptr);

  const auto& edges = it->second->edges;

  // Seam planning: the goal may list several endpoint-connected edges. They are
  // merged into one polyline and planned as a single continuous weld; the
  // single-edge path (edge_index) stays for compatibility.
  rosim::core::EdgeData seam_edge;
  const rosim::core::EdgeData* edge_ptr = nullptr;
  if (!goal->edge_indices.empty())
  {
    std::vector<uint32_t> idx(goal->edge_indices.begin(), goal->edge_indices.end());
    const auto merged = rosim::core::buildSeam(edges, idx);
    if (!merged.ok)
      return finish(false, FollowEdge::Result::ERROR_INVALID, 0.0, "seam: " + merged.message,
                    nullptr);
    seam_edge.index = 0;
    seam_edge.length = merged.length;
    seam_edge.closed = merged.closed;
    seam_edge.polyline = merged.polyline;
    edge_ptr = &seam_edge;
    RCLCPP_INFO(get_logger(), "seam: merged %zu edges, %.3f m%s", idx.size(), merged.length,
                merged.closed ? " (closed)" : "");
  }
  else
  {
    const auto edge_it = std::find_if(edges.begin(), edges.end(),
                                      [&](const rosim::core::EdgeData& e) {
                                        return e.index == goal->edge_index;
                                      });
    if (edge_it == edges.end())
      return finish(false, FollowEdge::Result::ERROR_INVALID, 0.0, "edge index out of range",
                    nullptr);
    edge_ptr = &*edge_it;
  }

  // ---- run the pipeline ----------------------------------------------------
  EdgeFollowParams params;
  params.orientation_mode = goal->orientation_mode;
  params.fixed_orientation = Eigen::Quaterniond(goal->fixed_orientation.w,
                                                goal->fixed_orientation.x,
                                                goal->fixed_orientation.y,
                                                goal->fixed_orientation.z);
  params.tangent_tilt = goal->tangent_tilt;
  params.tool_offset = goal->tool_offset;
  params.approach_distance = goal->approach_distance;
  params.max_waypoint_spacing = goal->max_waypoint_spacing;
  params.velocity_scaling = speed_scale_;
  params.acceleration_scaling = speed_scale_;
  params.scurve = goal->time_profile == 1;
  params.jerk_limit = jerk_limit_;
  params.seam_constant_speed = goal->seam_constant_speed;
  params.weld_speed = goal->weld_speed;
  params.seam_dwell = goal->seam_dwell;
  params.direction = goal->direction;
  params.corner_max_deviation = goal->corner_max_deviation;
  params.corner_threshold_deg = goal->corner_threshold_deg;
  params.corner_orient_window = goal->corner_orient_window;

  const auto start = snapshotCurrentState();
  scene_->setCurrentState(start);

  // Corner notices go through feedback.note so the UI can show them before
  // the trajectory is published (i.e. before the arm starts moving).
  auto publish_note = [&](const std::string& text) {
    auto fb = std::make_shared<FollowEdge::Feedback>();
    fb->progress = 0.1f;
    fb->phase = "resolving";
    fb->note = text;
    goal_handle->publish_feedback(fb);
  };
  EdgeFollowOutcome outcome = follower_->execute(*edge_ptr, params, start, scene_, cancel_,
                                                 publish_feedback, publish_note);

  planned_samples.reserve(outcome.planned_samples.size() * 3);
  for (const auto& f : outcome.planned_samples)
  {
    planned_samples.push_back(f.translation().x());
    planned_samples.push_back(f.translation().y());
    planned_samples.push_back(f.translation().z());
  }

  if (!outcome.success)
    return finish(false, outcome.error_code, outcome.cartesian_fraction, outcome.message, nullptr,
                  outcome.direction_used);

  // ---- execute -------------------------------------------------------------
  publishTrajectory(*outcome.trajectory, true);

  moveit_msgs::msg::RobotTrajectory rt_msg;
  outcome.trajectory->getRobotTrajectoryMsg(rt_msg);
  if (!monitorExecution(goal_handle, rt_msg.joint_trajectory))
  {
    return finish(false, FollowEdge::Result::ERROR_CANCELED, outcome.cartesian_fraction,
                  "canceled during execution", nullptr, outcome.direction_used);
  }

  finish(true, FollowEdge::Result::ERROR_NONE, outcome.cartesian_fraction, outcome.message,
         outcome.trajectory.get(), outcome.direction_used);
}

// ---------------------------------------------------------------------------
// FollowSeams — plan several seams into ONE continuous trajectory
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse PlanningNode::handleSeamsGoal(
    const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const FollowSeams::Goal> goal)
{
  if (busy_.load())
  {
    RCLCPP_WARN(LOGGER, "follow_seams rejected: busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (workpieces_.find(goal->model_id) == workpieces_.end())
  {
    RCLCPP_WARN(LOGGER, "follow_seams rejected: unknown model '%s'", goal->model_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->seams.empty())
  {
    RCLCPP_WARN(LOGGER, "follow_seams rejected: no seams");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(LOGGER, "follow_seams accepted: model=%s seams=%zu", goal->model_id.c_str(),
              goal->seams.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanningNode::handleSeamsCancel(
    const std::shared_ptr<GoalHandleFollowSeams> /*goal_handle*/)
{
  RCLCPP_INFO(LOGGER, "follow_seams cancel requested");
  cancel_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanningNode::handleSeamsAccepted(const std::shared_ptr<GoalHandleFollowSeams> goal_handle)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable())
    worker_.join();
  cancel_ = false;
  worker_ = std::thread([this, goal_handle]() { executeFollowSeams(goal_handle); });
}

void PlanningNode::executeFollowSeams(const std::shared_ptr<GoalHandleFollowSeams> goal_handle)
{
  busy_ = true;
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowSeams::Result>();
  auto feedback = std::make_shared<FollowSeams::Feedback>();

  const std::size_t n = goal->seams.size();
  feedback->seam_count = static_cast<uint8_t>(std::min<std::size_t>(n, 255));

  std::vector<uint8_t> failed_idx;
  std::vector<std::string> failed_msg;
  std::size_t executed = 0;
  // NOTE: "single seam" is decided from how many seams actually planned, further
  // below - a two-seam request where one fails must still keep the weld profile
  // of the survivor.

  auto finish = [&](bool success, const std::string& msg) {
    result->success = success;
    result->message = msg;
    result->executed = static_cast<uint8_t>(std::min<std::size_t>(executed, 255));
    result->failed = static_cast<uint8_t>(std::min<std::size_t>(failed_idx.size(), 255));
    result->failed_indices = failed_idx;
    result->failed_messages = failed_msg;
    // keep the merged trajectory in the result for tooling / debugging
    if (goal_handle->is_canceling())
      goal_handle->canceled(result);
    else if (success)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
    busy_ = false;
  };

  const auto it = workpieces_.find(goal->model_id);
  if (it == workpieces_.end())
    return finish(false, "model vanished");
  if (n == 0)
    return finish(false, "no seams in goal");

  robot_trajectory::RobotTrajectory combined(model_, model_->getJointModelGroup(group_));
  moveit::core::RobotState current = snapshotCurrentState();
  scene_->setCurrentState(current);

  for (std::size_t i = 0; i < n; ++i)
  {
    if (cancel_.load())
      break;

    const auto& spec = goal->seams[i];
    feedback->seam_index = static_cast<uint8_t>(std::min<std::size_t>(i, 255));
    feedback->phase = "resolving";
    feedback->progress = static_cast<float>(i) / static_cast<float>(n);
    goal_handle->publish_feedback(feedback);

    std::vector<uint32_t> idx(spec.edge_indices.begin(), spec.edge_indices.end());
    if (idx.empty())
    {
      failed_idx.push_back(static_cast<uint8_t>(i));
      failed_msg.push_back("seam has no edges");
      continue;
    }
    const auto merged = rosim::core::buildSeam(it->second->edges, idx);
    if (!merged.ok)
    {
      failed_idx.push_back(static_cast<uint8_t>(i));
      failed_msg.push_back("merge: " + merged.message);
      continue;
    }

    rosim::core::EdgeData seam_edge;
    seam_edge.index = 0;
    seam_edge.length = merged.length;
    seam_edge.closed = merged.closed;
    seam_edge.polyline = merged.polyline;

    EdgeFollowParams params;
    params.direction = spec.direction;
    params.corner_max_deviation = goal->corner_max_deviation;
    params.corner_threshold_deg = goal->corner_threshold_deg;
    params.corner_orient_window = goal->corner_orient_window;
    params.orientation_mode = goal->orientation_mode;
    params.fixed_orientation = Eigen::Quaterniond(goal->fixed_orientation.w,
                                                  goal->fixed_orientation.x,
                                                  goal->fixed_orientation.y,
                                                  goal->fixed_orientation.z);
    params.tangent_tilt = goal->tangent_tilt;
    params.tool_offset = goal->tool_offset;
    params.approach_distance = goal->approach_distance;
    params.max_waypoint_spacing = goal->max_waypoint_spacing;
    params.velocity_scaling = speed_scale_;
    params.acceleration_scaling = speed_scale_;
    params.scurve = goal->time_profile == 1;
    params.jerk_limit = jerk_limit_;
    params.seam_constant_speed = goal->seam_constant_speed;
    params.weld_speed = goal->weld_speed;
    params.seam_dwell = goal->seam_dwell;

    // Per-seam feedback is mapped into the batch's 0..1 progress.
    bool seam_planner_reported = false;
    auto seam_feedback = [&](float p, const std::string& phase) {
      feedback->seam_index = static_cast<uint8_t>(std::min<std::size_t>(i, 255));
      feedback->phase = phase;
      feedback->progress =
          (static_cast<float>(i) + std::min(std::max(p, 0.0f), 1.0f)) / static_cast<float>(n);
      if (!seam_planner_reported && phase == "following")
      {
        seam_planner_reported = true;
        feedback->note = "焊缝 " + std::to_string(i + 1) + " 规划完成：规划器 " + plannerLabel();
      }
      else
      {
        feedback->note.clear();
      }
      goal_handle->publish_feedback(feedback);
    };

    auto seam_note = [&](const std::string& text) {
      feedback->note = text;
      goal_handle->publish_feedback(feedback);
    };
    auto outcome =
        follower_->execute(seam_edge, params, current, scene_, cancel_, seam_feedback, seam_note);
    if (!outcome.success || !outcome.trajectory)
    {
      failed_idx.push_back(static_cast<uint8_t>(i));
      failed_msg.push_back(outcome.message.empty() ? "planning failed" : outcome.message);
      continue;
    }

    if (combined.size() == 0)
    {
      // The first planned seam is used verbatim: it already carries the weld
      // profile (constant seam speed, corner slowdowns, strike/extinguish ramps).
      combined = *outcome.trajectory;
    }
    else
    {
      // Multi-seam: append. The first waypoint of this seam is the pose the
      // previous one ended at (both are `current`), so drop it.
      for (std::size_t k = 0; k < outcome.trajectory->size(); ++k)
      {
        if (k == 0 && combined.size() > 0)
          continue;
        combined.addSuffixWayPoint(outcome.trajectory->getWayPoint(k), 0.0);
      }
    }
    current = outcome.trajectory->getLastWayPoint();
    scene_->setCurrentState(current);
    ++executed;
  }

  if (executed == 0)
    return finish(false, "no seam could be planned");
  const bool single_seam = executed == 1;
  if (cancel_.load())
    return finish(false, "canceled before execution");

  // Batch re-timing. A single seam is used verbatim: the edge follower already
  // timed it (TOTG or the requested double-S pass) and applied the weld profile
  // (constant seam speed, corner slowdowns, strike/extinguish ramps). Re-timing
  // it here is exactly what used to flatten that profile and make "double-S"
  // look non-uniform on screen. Multi-seam batches still get one global pass so
  // the robot does not stop at every seam boundary.
  if (single_seam)
  {
    RCLCPP_INFO(LOGGER, "single seam: keeping the planned weld profile");
  }
  else
  {
    try
    {
      trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.002, 0.01, 0.001);
      if (!totg.computeTimeStamps(combined, speed_scale_, speed_scale_))
        RCLCPP_WARN(LOGGER, "batch TOTG failed; keeping per-seam timings");
      if (goal->time_profile == 1)
        applyScurve(combined);
      // The global pass re-times the batch, so the per-seam uniform grid is gone.
      // Put the result back on an executable grid with consistent tangents,
      // otherwise a multi-seam batch is choppy again even though every single
      // seam was smooth (issue: TOTG produces 0.02 s next to 0.4 s segments).
      rosim::planning::regularizeTiming(combined, model_, LOGGER);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(LOGGER, "batch TOTG threw: %s", e.what());
    }
  }

  RCLCPP_INFO(LOGGER, "batch: publishing %zu waypoints, duration %.1fs", combined.size(),
              combined.size() > 1
                  ? combined.getWayPointDurationFromStart(combined.size() - 1)
                  : 0.0);
  publishTrajectory(combined, true);
  RCLCPP_INFO(LOGGER, "batch: published, entering monitor");
  moveit_msgs::msg::RobotTrajectory rt_msg;
  combined.getRobotTrajectoryMsg(rt_msg);
  // The server draws the TCP path from this trajectory; without it the batch
  // view would only show the last seam's motion.
  result->trajectory = rt_msg.joint_trajectory;

  feedback->phase = "executing";
  const bool ok = monitorTrajectory(
      rt_msg.joint_trajectory, [&]() { return cancel_.load() || goal_handle->is_canceling(); },
      [&](float p) {
        feedback->phase = "executing";
        feedback->progress = 0.9f + 0.1f * p;
        goal_handle->publish_feedback(feedback);
      });
  if (!ok)
    return finish(false, "canceled during execution");

  feedback->phase = "done";
  feedback->progress = 1.0f;
  goal_handle->publish_feedback(feedback);
  finish(true, "executed " + std::to_string(executed) + ", skipped " +
                   std::to_string(failed_idx.size()));
}

bool PlanningNode::monitorExecution(
    const std::shared_ptr<GoalHandleFollowEdge>& goal_handle,
    const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  return monitorTrajectory(
      trajectory, [&]() { return cancel_.load() || goal_handle->is_canceling(); },
      [&](float p) {
        auto feedback = std::make_shared<FollowEdge::Feedback>();
        feedback->phase = "executing";
        feedback->progress = p;
        goal_handle->publish_feedback(feedback);
      });
}

bool PlanningNode::monitorTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                     const std::function<bool()>& canceled,
                                     const std::function<void(float)>& progress)
{
  if (trajectory.points.empty())
    return true;

  const auto& final_point = trajectory.points.back();
  const double total_time = final_point.time_from_start.sec +
                            final_point.time_from_start.nanosec * 1e-9;
  const auto t0 = std::chrono::steady_clock::now();
  const double timeout = total_time + 4.0;

  while (rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (canceled())
      return false;

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (progress)
      progress(static_cast<float>(std::min(1.0, elapsed / std::max(total_time, 0.1))));
    static auto last_dbg = std::chrono::steady_clock::now();
    if (std::chrono::steady_clock::now() - last_dbg > std::chrono::seconds(5))
    {
      last_dbg = std::chrono::steady_clock::now();
      RCLCPP_INFO(LOGGER, "monitor: elapsed %.1fs / %.1fs", elapsed, total_time);
    }

    if (elapsed > timeout)
    {
      RCLCPP_WARN(LOGGER, "execution timed out after %.1fs", timeout);
      return true;  // trajectory was sent; treat as done for this V1
    }

    // reached the goal?
    const auto state = snapshotCurrentState();
    const auto& names = trajectory.joint_names;
    double max_err = 0.0;
    for (size_t i = 0; i < names.size() && i < final_point.positions.size(); ++i)
    {
      const double target = final_point.positions[i];
      max_err = std::max(max_err, std::abs(state.getVariablePosition(names[i]) - target));
    }
    // Report completion only when the controller has actually settled on the
    // final waypoint (mixed units: rad and metres for the rail). The old 0.02
    // window could return while the last segment was still executing, so the
    // next action sampled a moving, near-contact start state.
    // The pose check alone is not enough: a closed seam ends where it started,
    // and a batch that revisits its final pose (e.g. the same seam twice) would
    // be reported "done" the moment the arm happens to be near that pose. Require
    // most of the planned duration to have elapsed as well.
    if (max_err < 0.003 && elapsed > 0.3 && elapsed > 0.9 * total_time)
      return true;
  }
  return true;
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
bool PlanningNode::applyScurve(robot_trajectory::RobotTrajectory& trajectory)
{
  const auto* group = trajectory.getGroup();
  if (!group || trajectory.size() < 3)
    return false;

  std::unordered_map<std::string, double> vlim, alim, jlim;
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
  {
    const auto& bounds = jm->getVariableBounds();
    const auto& names = jm->getVariableNames();
    const bool prismatic = jm->getType() == moveit::core::JointModel::PRISMATIC;
    for (std::size_t i = 0; i < names.size() && i < bounds.size(); ++i)
    {
      vlim[names[i]] = bounds[i].max_velocity_ * speed_scale_;
      alim[names[i]] = (bounds[i].max_acceleration_ > 1e-9 ? bounds[i].max_acceleration_ : 1.0) * speed_scale_;
      jlim[names[i]] = bounds[i].max_jerk_ > 1e-9
                           ? bounds[i].max_jerk_
                           : (prismatic ? jerk_limit_ * 0.05 : jerk_limit_);
    }
  }
  if (!trajectory_processing::RuckigSmoothing::applySmoothing(trajectory, vlim, alim, jlim))
  {
    RCLCPP_WARN(LOGGER, "Ruckig smoothing failed; keeping the TOTG profile");
    return false;
  }
  return true;
}

void PlanningNode::publishTrajectory(const robot_trajectory::RobotTrajectory& trajectory,
                                     bool publish_display)
{
  moveit_msgs::msg::RobotTrajectory rt_msg;
  trajectory.getRobotTrajectoryMsg(rt_msg);
  trajectory_pub_->publish(rt_msg.joint_trajectory);

  if (publish_display)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = model_->getName();
    display.trajectory.resize(1);
    display.trajectory[0] = rt_msg;
    moveit_msgs::msg::RobotState start_state_msg;
    moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), start_state_msg);
    display.trajectory_start = start_state_msg;
    display_traj_pub_->publish(display);
  }
}

moveit::core::RobotState PlanningNode::snapshotCurrentState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return *current_state_;
}

}  // namespace rosim::planning
