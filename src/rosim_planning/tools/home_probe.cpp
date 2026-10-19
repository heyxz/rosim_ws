/// Probe candidate "home" poses for self-collision and report TCP placement,
/// so the SRDF home state can be picked from measured facts, not hand math.
/// Run with the same parameters as rosim_planning_node:
///   ros2 run rosim_planning home_probe --ros-args -p robot_description:=... \
///     -p robot_description_semantic:=... [-p <kinematics.yaml>]
/// Optional obstacle: set -p obstacle_box:="x0 x1 y0 y1 z0 z1" (world frame) to
/// emulate a workpiece and print the colliding contact pairs for the SRDF home
/// state:
///   ros2 run rosim_planning home_probe --ros-args ... \
///     -p obstacle_box:="1.0 1.3 0 0.16 0.6 0.71"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("home_probe");

  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoader loader(node, opts);
  auto model = loader.getModel();
  if (!model)
  {
    std::cerr << "robot model load failed\n";
    return 1;
  }
  const std::string group = "manipulator";
  auto jmg = model->getJointModelGroup(group);
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);

  // Optional world-frame box obstacle to emulate the workpiece.
  const std::string box_arg = node->declare_parameter<std::string>("obstacle_box", "");
  if (!box_arg.empty())
  {
    double x0, x1, y0, y1, z0, z1;
    if (std::sscanf(box_arg.c_str(), "%lf %lf %lf %lf %lf %lf", &x0, &x1, &y0, &y1, &z0,
                    &z1) != 6)
    {
      std::cerr << "obstacle_box needs 6 numbers\n";
      return 1;
    }
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = model->getModelFrame();
    obj.id = "probe_box";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = { x1 - x0, y1 - y0, z1 - z0 };
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5 * (x0 + x1);
    pose.position.y = 0.5 * (y0 + y1);
    pose.position.z = 0.5 * (z0 + z1);
    pose.orientation.w = 1.0;
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene->processCollisionObjectMsg(obj);
    printf("obstacle box x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]\n", x0, x1, y0, y1, z0, z1);
  }

  auto report = [&](const std::string& name, moveit::core::RobotState& state) {
    collision_detection::CollisionRequest creq;
    creq.contacts = true;
    creq.max_contacts = 40;
    creq.max_contacts_per_pair = 2;
    collision_detection::CollisionResult cres;
    scene->checkCollision(creq, cres, state);
    const auto& t0 = state.getGlobalLinkTransform("tool0").translation();
    const auto& tt = state.getGlobalLinkTransform("tool_tip").translation();
    printf("%-24s %-8s (%.3f, %.3f, %.3f)   (%.3f, %.3f, %.3f)\n", name.c_str(),
           cres.collision ? "YES" : "no", t0.x(), t0.y(), t0.z(), tt.x(), tt.y(), tt.z());
    for (const auto& entry : cres.contacts)
      for (const auto& c : entry.second)
        printf("    %-18s <-> %-18s depth=%.4f at (%.3f, %.3f, %.3f)\n",
               entry.first.first.c_str(), entry.first.second.c_str(), c.depth, c.pos.x(),
               c.pos.y(), c.pos.z());
  };

  // The actual SRDF named state that handleGoHome uses.
  {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    if (state.setToDefaultValues(jmg, "home"))
    {
      state.update();
      report("SRDF home (manipulator)", state);
    }
    else
    {
      printf("SRDF has no 'home' state for group manipulator\n");
    }
  }

  // Optional arbitrary state: -p state:="j1 j2 j3 j4 j5 j6 rail"
  const std::string state_arg = node->declare_parameter<std::string>("state", "");
  if (!state_arg.empty())
  {
    double q[7];
    if (std::sscanf(state_arg.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &q[0], &q[1], &q[2],
                    &q[3], &q[4], &q[5], &q[6]) != 7)
    {
      std::cerr << "state needs 7 numbers (j1 j2 j3 j4 j5 j6 rail)\n";
      return 1;
    }
    static const char* names[7] = { "j1", "j2", "j3", "j4", "j5", "j6", "rail_joint" };
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (int i = 0; i < 7; ++i)
      if (model->hasJointModel(names[i]))
        state.setVariablePosition(names[i], q[i]);
    state.update();
    report("probe state", state);
  }

  struct Cand
  {
    std::string name;
    std::vector<double> q;
  };
  // GP12-proportioned candidates: torch down-forward, tip in the r~0.7-1.0 m
  // / z~0.4-0.8 m sweet zone (zeros pose has tool0 straight up at 0.895/1.264)
  const std::vector<Cand> cands = {
      { "old_home 0.5/-1.0/0.6", { 0, 0.5, -1.0, 0, 0.6, 0 } },
      { "h1  0.3/-0.6/0.4", { 0, 0.3, -0.6, 0, 0.4, 0 } },
      { "h2  0.5/-0.8/0.6", { 0, 0.5, -0.8, 0, 0.6, 0 } },
      { "h3  0.6/-0.9/0.7", { 0, 0.6, -0.9, 0, 0.7, 0 } },
      { "h4  0.4/-0.7/0.5", { 0, 0.4, -0.7, 0, 0.5, 0 } },
      { "h5  0.7/-1.1/0.8", { 0, 0.7, -1.1, 0, 0.8, 0 } },
      { "h6  0.2/-0.5/0.3", { 0, 0.2, -0.5, 0, 0.3, 0 } },
      { "h7  0.8/-1.2/0.9", { 0, 0.8, -1.2, 0, 0.9, 0 } },
      { "zeros", { 0, 0, 0, 0, 0, 0 } },
      // 7th-axis-era candidates: clear a workpiece parked at x[1.0,1.3]
      // z[0.6,0.71] in front of the rail. Only the low, forward home collides.
      { "p1  0.1/-0.5/0.3", { 0, 0.1, -0.5, 0, 0.3, 0 } },
      { "p2  0.0/-0.5/0.3", { 0, 0.0, -0.5, 0, 0.3, 0 } },
      { "p3  0.1/-0.4/0.3", { 0, 0.1, -0.4, 0, 0.3, 0 } },
      { "p4  0.2/-0.4/0.3", { 0, 0.2, -0.4, 0, 0.3, 0 } },
      { "p5  0.0/-0.3/0.1", { 0, 0.0, -0.3, 0, 0.1, 0 } },
      { "p6  0.1/-0.3/0.0", { 0, 0.1, -0.3, 0, 0.0, 0 } },
      { "p7  0.0/-0.2/0.0", { 0, 0.0, -0.2, 0, 0.0, 0 } },
      { "p8  0.2/-0.3/0.2", { 0, 0.2, -0.3, 0, 0.2, 0 } },
      { "p9  0.3/-0.3/0.2", { 0, 0.3, -0.3, 0, 0.2, 0 } },
      { "p10 0.3/-0.2/0.0", { 0, 0.3, -0.2, 0, 0.0, 0 } },
  };

  printf("%-24s %-8s %-28s %s\n", "candidate", "collide", "tool0 xyz", "tool_tip xyz");
  const moveit::core::JointModelGroup* arm = model->getJointModelGroup("arm");
  for (const auto& c : cands)
  {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    // Candidates are 6 arm values; set them on the arm group and pin the rail
    // (setting a 6-vector on the 7-variable composite group would corrupt it).
    state.setJointGroupPositions(arm, c.q);
    if (model->hasJointModel("rail_joint"))
      state.setVariablePosition("rail_joint", 0.0);
    state.update();
    report(c.name, state);
  }

  rclcpp::shutdown();
  return 0;
}
