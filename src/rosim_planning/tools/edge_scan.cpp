/// Scan every workpiece edge for IK reachability with the exact approach pose
/// the edge follower would use (TANGENT orientation, approach back-off), so
/// demo placements can be chosen from measured facts.
///
///   ros2 run rosim_planning edge_scan --ros-args --params-file <file>
/// with robot_description / robot_description_semantic on the node, plus:
///   workpiece (string, STEP path), wp_x/wp_y/wp_z, wp_yaw_deg

#include <cmath>
#include <iostream>
#include <set>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <moveit/collision_detection/collision_common.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.h>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_planning/edge_follower.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("edge_scan");

  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = true;
  robot_model_loader::RobotModelLoader loader(node, opts);
  auto model = loader.getModel();
  if (!model)
  {
    std::cerr << "robot model load failed\n";
    return 1;
  }
  const std::string group = "manipulator";
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);

  // ---- workpiece into the scene (mirrors PlanningNode::handleImportModel) ----
  std::string step_path;
  double wx = 0.0, wy = 0.0, wz = 0.0, yaw_deg = 0.0, tilt_deg = 0.0;
  node->declare_parameter<std::string>("workpiece", "");
  node->declare_parameter<double>("wp_x", wx);
  node->declare_parameter<double>("wp_y", wy);
  node->declare_parameter<double>("wp_z", wz);
  node->declare_parameter<double>("wp_yaw_deg", yaw_deg);
  node->declare_parameter<double>("wp_tilt_deg", tilt_deg);
  node->get_parameter("workpiece", step_path);
  node->get_parameter("wp_x", wx);
  node->get_parameter("wp_y", wy);
  node->get_parameter("wp_z", wz);
  node->get_parameter("wp_yaw_deg", yaw_deg);
  node->get_parameter("wp_tilt_deg", tilt_deg);
  if (step_path.empty())
  {
    std::cerr << "workpiece param not set\n";
    return 1;
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(wx, wy, wz);
  pose.linear() = Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto wp = rosim::core::OccGeometryService::importModel(step_path, pose, "scan_piece");
  if (!wp)
  {
    std::cerr << "STEP import failed: " << step_path << "\n";
    return 1;
  }
  rosim::core::MeshData mesh;
  rosim::core::OccGeometryService::tessellate(wp->shape, 2e-3, 0.5, mesh);
  shapes::ShapePtr collision_shape = rosim::core::OccGeometryService::toCollisionMesh(mesh, pose);
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = model->getModelFrame();
    obj.id = "scan_piece";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(collision_shape.get(), mesh_msg);
    obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
    obj.mesh_poses.push_back(geometry_msgs::msg::Pose());
    scene->processCollisionObjectMsg(obj);
  }

  // ---- probe every edge with the follower's own IK machinery -------------
  rosim::planning::EdgeFollower follower(node, model, nullptr);
  const auto* jmg = model->getJointModelGroup(group);
  moveit::core::RobotState current(model);
  current.setToDefaultValues();
  current.setToDefaultValues(jmg, "home");
  current.update();

  // Optional comma-separated edge filter, e.g. edge_indices:=0,2,49
  std::string only;
  node->declare_parameter<std::string>("edge_indices", "");
  node->get_parameter("edge_indices", only);
  std::set<unsigned> wanted;
  {
    std::stringstream ss(only);
    std::string tok;
    while (std::getline(ss, tok, ','))
      if (!tok.empty())
        wanted.insert(static_cast<unsigned>(std::stoul(tok)));
  }

  // IK reachability independent of collisions: with no validity callback the
  // solver ignores the workpiece. Distinguishes "out of reach" from "the pose
  // itself intersects the part".
  auto plainIk = [&](const Eigen::Isometry3d& pose, moveit::core::RobotState& out) {
    moveit::core::RobotState s(model);
    s.setToDefaultValues();
    s.setToDefaultValues(jmg, "home");
    s.update();
    if (s.setFromIK(jmg, pose, "tool_tip", 0.05))
    {
      out = s;
      return true;
    }
    for (int i = 0; i < 30; ++i)
    {
      s.setToRandomPositions(jmg);
      s.update();
      if (s.setFromIK(jmg, pose, "tool_tip", 0.02))
      {
        out = s;
        return true;
      }
    }
    return false;
  };

  auto collisionPairs = [&](const moveit::core::RobotState& st) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 4;
    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, st, scene->getAllowedCollisionMatrix());
    std::string s;
    for (const auto& c : res.contacts)
    {
      if (!s.empty())
        s += ",";
      s += c.first.first + "-" + c.first.second;
    }
    return s;
  };

  const auto& edges = wp->edges;
  printf("%-6s %-8s %-9s %-9s %-24s %s\n", "edge", "len[m]", "collfree", "reach",
         "approach xyz", "contacts / reversed");
  int reachable = 0;
  for (const auto& e : edges)
  {
    if (!wanted.empty() && !wanted.count(e.index))
      continue;
    rosim::core::EdgeFrameGenerator::Config cfg;
    cfg.tangent_tilt = tilt_deg * M_PI / 180.0;
    const auto frames = rosim::core::EdgeFrameGenerator::generate(e.polyline, cfg);
    if (frames.empty())
      continue;
    const Eigen::Isometry3d approach = frames.front() * Eigen::Translation3d(0, 0, -0.08);
    moveit::core::RobotStatePtr sol;
    const bool ok = follower.solveIkMultiSeed(approach, current, scene, sol);
    if (ok)
      ++reachable;

    moveit::core::RobotState plain(model);
    const bool reach = plainIk(approach, plain);
    const std::string contacts = reach ? collisionPairs(plain) : "";

    // Traversing the edge the other way: regenerate frames on the reversed
    // polyline (the tangent flips, so a torch pointing "up" along a vertical
    // edge turns to point "down"). If only the forward direction is blocked,
    // the reversed toolpath often starts in free space.
    std::vector<Eigen::Vector3d> rev_poly(e.polyline.rbegin(), e.polyline.rend());
    const auto rev_frames = rosim::core::EdgeFrameGenerator::generate(rev_poly, cfg);
    bool ok_rev = false, reach_rev = false;
    moveit::core::RobotState plain_rev(model);
    if (!rev_frames.empty())
    {
      const Eigen::Isometry3d approach_rev =
          rev_frames.front() * Eigen::Translation3d(0, 0, -0.08);
      moveit::core::RobotStatePtr sol_rev;
      ok_rev = follower.solveIkMultiSeed(approach_rev, current, scene, sol_rev);
      reach_rev = plainIk(approach_rev, plain_rev);
    }

    printf("%-6u %-8.3f %-9s %-9s (%.3f, %.3f, %.3f)      %s | rev: %s/%s\n", e.index,
           e.length, ok ? "YES" : "no", reach ? "yes" : "no", approach.translation().x(),
           approach.translation().y(), approach.translation().z(),
           contacts.empty() ? "-" : contacts.c_str(), ok_rev ? "YES" : "no",
           reach_rev ? "reach" : "unreach");
  }
  printf("reachable: %d / %zu\n", reachable, edges.size());

  rclcpp::shutdown();
  return 0;
}
