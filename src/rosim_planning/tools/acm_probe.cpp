/// Scan every link pair for self-collision with an EMPTY allowed-collision
/// matrix (zeros + candidate homes + N random states) and print an
/// Adjacent/Always/Sometimes/Never table plus paste-ready SRDF
/// disable_collisions lines. The SRDF pair list MUST come from measurement:
/// a missing always-touching pair silently invalidates every KDL IK seed
/// (known project trap). Run with the same parameters as home_probe:
///   ros2 run rosim_planning acm_probe --ros-args -p robot_description:=... \
///     -p robot_description_semantic:=... [-p samples:=3000]

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("acm_probe");
  int samples = 3000;
  node->declare_parameter<int>("samples", samples);
  node->get_parameter("samples", samples);

  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoader loader(node, opts);
  auto model = loader.getModel();
  if (!model)
  {
    std::cerr << "robot model load failed\n";
    return 1;
  }

  // adjacency across one joint, and links that carry collision geometry
  std::map<std::string, std::string> parent_of;
  std::vector<std::string> links;
  for (const moveit::core::LinkModel* l : model->getLinkModels())
  {
    if (const moveit::core::LinkModel* p = l->getParentJointModel()->getParentLinkModel())
      parent_of[l->getName()] = p->getName();
    if (!l->getShapes().empty())
      links.push_back(l->getName());
  }
  std::sort(links.begin(), links.end());

  std::map<std::pair<std::string, std::string>, int> contacts;
  int tested = 0;

  collision_detection::CollisionEnvFCL cenv(model, 0.0, 1.0);
  const collision_detection::AllowedCollisionMatrix acm;  // empty: every pair checked
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  auto record = [&]() {
    state.update();
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 128;
    req.max_contacts_per_pair = 1;
    collision_detection::CollisionResult res;
    cenv.checkSelfCollision(req, res, state, acm);
    ++tested;
    for (const auto& [id, _] : res.contacts)
    {
      const auto key = std::make_pair(std::min(id.first, id.second), std::max(id.first, id.second));
      ++contacts[key];
    }
  };

  auto set_q = [&](const std::vector<double>& q) {
    state.setJointGroupPositions(model->getJointModelGroup("manipulator"), q);
  };

  // fixed poses: zeros + candidate homes (same family as home_probe)
  const std::vector<std::vector<double>> fixed = {
      { 0, 0, 0, 0, 0, 0 },
      { 0, 0.5, -0.9, 0, 0.5, 0 },
      { 0, 0.6, -1.0, 0, 0.6, 0 },
      { 0, 0.9, 1.2, 0, 0.6, 0 },
      { 0, 0.45, -1.2, 0, 0.9, 0 },
      { 0, -1.2, 0.9, 0, 1.0, 0 },
  };
  for (const auto& q : fixed)
  {
    set_q(q);
    record();
  }

  // zeros FK reference for the later home search
  set_q(fixed[0]);
  state.update();
  const auto& t0 = state.getGlobalLinkTransform("tool0").translation();
  const auto& tt = state.getGlobalLinkTransform("tool_tip").translation();
  printf("zeros FK: tool0 (%.3f, %.3f, %.3f)  tool_tip (%.3f, %.3f, %.3f)\n\n",
         t0.x(), t0.y(), t0.z(), tt.x(), tt.y(), tt.z());

  for (int i = 0; i < samples; ++i)
  {
    state.setToRandomPositions();
    record();
  }

  printf("%-26s %-9s %6s  %s\n", "pair", "adjacent", "hit/of", "class");
  std::vector<std::string> srdf;
  for (size_t a = 0; a < links.size(); ++a)
    for (size_t b = a + 1; b < links.size(); ++b)
    {
      const std::string& la = links[a];
      const std::string& lb = links[b];
      const bool adjacent = (parent_of.count(la) && parent_of[la] == lb) ||
                            (parent_of.count(lb) && parent_of[lb] == la);
      const int hit = contacts.count({ la, lb }) ? contacts[{ la, lb }] : 0;
      std::string cls = adjacent ? "ADJACENT" : (hit == 0 ? "NEVER" : (hit >= tested ? "ALWAYS" : "SOMETIMES"));
      printf("%-26s %-9s %4d/%-4d  %s\n", (la + " / " + lb).c_str(), adjacent ? "yes" : "no", hit, tested,
             cls.c_str());
      if (cls != "SOMETIMES")
        srdf.push_back("    <disable_collisions link1=\"" + la + "\" link2=\"" + lb + "\" reason=\"" +
                       (adjacent ? "Adjacent" : (hit == 0 ? "Never" : "Always")) + "\"/>");
    }

  printf("\npaste into robot.srdf:\n");
  for (const auto& line : srdf)
    printf("%s\n", line.c_str());
  rclcpp::shutdown();
  return 0;
}
