/// Probe one TCP pose with many unconstrained random-seed IK solves and print
/// every distinct solution branch found (with self-collision status and
/// distance from home). Answers "does a natural solution exist at all?".
///
///   ros2 run rosim_planning ik_probe --ros-args --params-file <file> \
///     -p px:=0.45 -p py:=-0.15 -p pz:=0.35 -p qx:=1.0 -p qy:=0.0 -p qz:=0.0 -p qw:=0.0

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.h>

#include "rosim_core/occ_geometry_service.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ik_probe");

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
  const std::string tip = "tool_tip";
  auto group = model->getJointModelGroup(group_name);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);

  // optional workpiece STEP in the scene, to test collision-filtered branches
  std::string step_path;
  double wx = 0.0, wy = 0.0, wz = 0.0, yaw_deg = 0.0;
  node->declare_parameter<std::string>("workpiece", "");
  node->declare_parameter<double>("wp_x", wx);
  node->declare_parameter<double>("wp_y", wy);
  node->declare_parameter<double>("wp_z", wz);
  node->declare_parameter<double>("wp_yaw_deg", yaw_deg);
  node->get_parameter("workpiece", step_path);
  node->get_parameter("wp_x", wx);
  node->get_parameter("wp_y", wy);
  node->get_parameter("wp_z", wz);
  node->get_parameter("wp_yaw_deg", yaw_deg);
  if (!step_path.empty())
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(wx, wy, wz);
    pose.linear() =
        Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto wp = rosim::core::OccGeometryService::importModel(step_path, pose, "probe_piece");
    if (!wp)
    {
      std::cerr << "STEP import failed: " << step_path << "\n";
      return 1;
    }
    rosim::core::MeshData mesh;
    rosim::core::OccGeometryService::tessellate(wp->shape, 2e-3, 0.5, mesh);
    shapes::ShapePtr collision_shape = rosim::core::OccGeometryService::toCollisionMesh(mesh, pose);
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = model->getModelFrame();
    obj.id = "probe_piece";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(collision_shape.get(), mesh_msg);
    obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
    obj.mesh_poses.push_back(geometry_msgs::msg::Pose());
    scene->processCollisionObjectMsg(obj);
    printf("workpiece in scene: %s\n", step_path.c_str());
  }

  double px = 0, py = 0, pz = 0, qx = 0, qy = 0, qz = 0, qw = 1;
  int attempts = 200;
  node->declare_parameter<double>("px", px);
  node->declare_parameter<double>("py", py);
  node->declare_parameter<double>("pz", pz);
  node->declare_parameter<double>("qx", qx);
  node->declare_parameter<double>("qy", qy);
  node->declare_parameter<double>("qz", qz);
  node->declare_parameter<double>("qw", qw);
  node->declare_parameter<int>("attempts", attempts);
  node->get_parameter("px", px);
  node->get_parameter("py", py);
  node->get_parameter("pz", pz);
  node->get_parameter("qx", qx);
  node->get_parameter("qy", qy);
  node->get_parameter("qz", qz);
  node->get_parameter("qw", qw);
  node->get_parameter("attempts", attempts);

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(px, py, pz);
  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();
  target.linear() = q.toRotationMatrix();

  moveit::core::RobotState home(model);
  home.setToDefaultValues();
  home.setToDefaultValues(group, "home");
  home.update();
  std::vector<double> home_joints;
  home.copyJointGroupPositions(group, home_joints);

  moveit::core::RobotState probe(model);
  probe.setToDefaultValues();
  probe.setToRandomPositions();

  // mimic EdgeFollower::solveIkMultiSeed: validity callback + 0.05s timeout
  auto validity = [&scene](moveit::core::RobotState* s, const moveit::core::JointModelGroup*,
                           const double*) {
    s->updateCollisionBodyTransforms();
    return !scene->isStateColliding(*s);
  };

  // distinct solutions keyed by rounded joints
  std::map<std::vector<int>, std::vector<double>> found;
  int solved = 0;
  for (int i = 0; i < attempts; ++i)
  {
    probe.setToRandomPositions(group);
    moveit::core::RobotState attempt(probe);
    if (!attempt.setFromIK(group, target, tip, std::vector<double>(), 0.05, validity))
      continue;
    ++solved;
    std::vector<double> j;
    attempt.copyJointGroupPositions(group, j);
    std::vector<int> key;
    for (double v : j)
      key.push_back(static_cast<int>(std::round(v / 0.087)));  // ~5 deg buckets
    found.emplace(key, j);
  }

  printf("target tip (%.3f, %.3f, %.3f) q(%.2f, %.2f, %.2f, %.2f)\n", px, py, pz, qx, qy, qz, qw);
  printf("solved %d / %d attempts, %zu distinct branches\n", solved, attempts, found.size());
  for (const auto& [key, j] : found)
  {
    moveit::core::RobotState s(model);
    s.setToDefaultValues();
    s.setJointGroupPositions(group, j);
    s.updateCollisionBodyTransforms();
    const bool collide = scene->isStateColliding(s);
    double dist = 0;
    for (size_t k = 0; k < j.size(); ++k)
    {
      const double d = j[k] - home_joints[k];
      dist += d * d;
    }
    printf("%s  |q-home|=%.2f  q = %.3f %.3f %.3f %.3f %.3f %.3f\n", collide ? "COLLIDE" : "ok    ",
           std::sqrt(dist), j[0], j[1], j[2], j[3], j[4], j[5]);
  }

  rclcpp::shutdown();
  return 0;
}
