/// Per-waypoint diagnosis of ONE edge: for every TCP frame along the edge
/// (same frame generation + resampling as EdgeFollower) classify
///   OK      collision-free IK exists
///   COLLIDE IK exists but every solution collides
///   NO-IK   no IK solution at all (reach / limits)
/// to explain mid-edge cartesian truncations.
///
///   ros2 run rosim_planning traverse_probe --ros-args --params-file <file>
/// params: workpiece, wp_x/wp_y/wp_z/wp_yaw_deg, edge_index, tilt_deg, mode

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.h>

#include "rosim_core/edge_frame_generator.h"
#include "rosim_core/occ_geometry_service.h"
#include "rosim_planning/edge_follower.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traverse_probe");

  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = true;
  robot_model_loader::RobotModelLoader loader(node, opts);
  auto model = loader.getModel();
  if (!model)
  {
    std::cerr << "robot model load failed\n";
    return 1;
  }
  const std::string group_name = "manipulator";
  const auto* group = model->getJointModelGroup(group_name);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);

  std::string step_path;
  double wx = 0.61, wy = -0.15, wz = 0.45, yaw_deg = 90.0, tilt_deg = 40.0;
  int edge_index = 0, mode = 0;
  node->declare_parameter<std::string>("workpiece", "");
  node->declare_parameter<double>("wp_x", wx);
  node->declare_parameter<double>("wp_y", wy);
  node->declare_parameter<double>("wp_z", wz);
  node->declare_parameter<double>("wp_yaw_deg", yaw_deg);
  node->declare_parameter<double>("tilt_deg", tilt_deg);
  node->declare_parameter<int>("edge_index", edge_index);
  node->declare_parameter<int>("mode", mode);
  node->get_parameter("workpiece", step_path);
  node->get_parameter("wp_x", wx);
  node->get_parameter("wp_y", wy);
  node->get_parameter("wp_z", wz);
  node->get_parameter("wp_yaw_deg", yaw_deg);
  node->get_parameter("tilt_deg", tilt_deg);
  node->get_parameter("edge_index", edge_index);
  node->get_parameter("mode", mode);
  if (step_path.empty())
  {
    std::cerr << "workpiece param not set\n";
    return 1;
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(wx, wy, wz);
  pose.linear() =
      Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto wp = rosim::core::OccGeometryService::importModel(step_path, pose, "probe_piece");
  if (!wp)
  {
    std::cerr << "STEP import failed\n";
    return 1;
  }
  rosim::core::MeshData mesh;
  rosim::core::OccGeometryService::tessellate(wp->shape, 2e-3, 0.5, mesh);
  shapes::ShapePtr collision_shape = rosim::core::OccGeometryService::toCollisionMesh(mesh, pose);
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = model->getModelFrame();
    obj.id = "probe_piece";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(collision_shape.get(), mesh_msg);
    obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
    obj.mesh_poses.push_back(geometry_msgs::msg::Pose());
    scene->processCollisionObjectMsg(obj);
  }

  const auto edge_it = std::find_if(
      wp->edges.begin(), wp->edges.end(),
      [edge_index](const rosim::core::EdgeData& e) {
        return e.index == static_cast<uint32_t>(edge_index);
      });
  if (edge_it == wp->edges.end())
  {
    std::cerr << "edge " << edge_index << " not found\n";
    return 1;
  }

  // resample + frames, verbatim from EdgeFollower::execute
  rosim::core::EdgeFrameGenerator::Config cfg;
  cfg.orientation_mode = static_cast<uint8_t>(mode);
  cfg.tangent_tilt = tilt_deg * M_PI / 180.0;
  const double spacing = std::min(0.005, edge_it->length / 4.0);
  rosim::core::EdgeData resampled;
  resampled.index = edge_it->index;
  resampled.closed = edge_it->closed;
  resampled.length = edge_it->length;
  {
    double acc = 0.0;
    resampled.polyline.push_back(edge_it->polyline.front());
    for (size_t i = 1; i < edge_it->polyline.size(); ++i)
    {
      acc += (edge_it->polyline[i] - edge_it->polyline[i - 1]).norm();
      if (acc >= spacing)
      {
        resampled.polyline.push_back(edge_it->polyline[i]);
        acc = 0.0;
      }
    }
    if ((resampled.polyline.back() - edge_it->polyline.back()).norm() > 1e-6)
      resampled.polyline.push_back(edge_it->polyline.back());
  }
  const auto frames = rosim::core::EdgeFrameGenerator::generate(resampled.polyline, cfg);

  rosim::planning::EdgeFollower follower(node, model, nullptr);
  {
    rosim::planning::EdgeFollower::Config cfg;
    cfg.random_seed_attempts = 300;  // exhaustive: distinguish NO-IK from seed luck
    follower.setConfig(cfg);
  }
  moveit::core::RobotState seed(model);
  seed.setToDefaultValues();
  seed.setToDefaultValues(group, "home");
  seed.update();

  const std::string tip = "tool_tip";
  printf("edge %d: %zu frames, spacing %.4f m, mode %d, tilt %.1f deg\n", edge_index,
         frames.size(), spacing, mode, tilt_deg);
  printf("%-6s %-6s %-24s %s\n", "idx", "s[m]", "tcp xyz", "status");

  int first_bad = -1;
  for (size_t i = 0; i < frames.size(); ++i)
  {
    moveit::core::RobotStatePtr sol;
    const bool clean = follower.solveIkMultiSeed(frames[i], seed, scene, sol);

    // raw IK without collision filter to separate NO-IK from COLLIDE
    bool raw = false;
    if (!clean)
    {
      moveit::core::RobotState probe(seed);
      raw = probe.setFromIK(group, frames[i], tip, 0.05);
    }

    const char* status = clean ? "OK" : (raw ? "COLLIDE" : "NO-IK");
    if (!clean && first_bad < 0)
      first_bad = static_cast<int>(i);
    if (i % 4 == 0 || !clean)
      printf("%-6zu %-6.3f (%6.3f,%6.3f,%6.3f) %s\n", i,
             spacing * static_cast<double>(i), frames[i].translation().x(),
             frames[i].translation().y(), frames[i].translation().z(), status);
    if (!clean && first_bad >= 0 && static_cast<int>(i) > first_bad + 8)
    {
      printf("... stopping after 8 consecutive bad waypoints\n");
      break;
    }
  }

  // classify the approach candidates EdgeFollower tries, at several retreat
  // distances d, to see whether a shorter approach clears the reach boundary
  printf("approach candidates (back-off = -tangent, hover = world +Z):\n");
  for (const double d : { 0.08, 0.06, 0.04, 0.02 })
  {
    Eigen::Isometry3d back_off = frames.front() * Eigen::Translation3d(0, 0, -d);
    Eigen::Isometry3d hover = back_off;
    hover.translation() = frames.front().translation() + Eigen::Vector3d(0, 0, d);
    std::vector<std::pair<const char*, Eigen::Isometry3d>> cands;
    cands.emplace_back("back-off", back_off);
    cands.emplace_back("hover  ", hover);
    for (const auto& [name, pose] : cands)
    {
      moveit::core::RobotStatePtr sol;
      const bool clean = follower.solveIkMultiSeed(pose, seed, scene, sol);
      bool raw = false;
      if (!clean)
      {
        moveit::core::RobotState probe(seed);
        raw = probe.setFromIK(group, pose, tip, 0.05);
      }
      printf("  d=%.2f %s (%6.3f,%6.3f,%6.3f) %s\n", d, name, pose.translation().x(),
             pose.translation().y(), pose.translation().z(),
             clean ? "OK" : (raw ? "COLLIDE" : "NO-IK"));
    }
  }
  rclcpp::shutdown();
  return 0;
}
