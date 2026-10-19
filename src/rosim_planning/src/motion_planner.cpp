// MotionPlannerFactory: a string-keyed registry of planner implementations.
// "ompl" is registered by this library; "rl" is registered when librosim_rl.so
// is dlopen'ed by the planning node at startup.

#include "rosim_planning/motion_planner.h"

#include <mutex>
#include <stdexcept>
#include <vector>

namespace rosim::planning
{

std::map<std::string, MotionPlannerFactory::Creator>& MotionPlannerFactory::registry()
{
  static std::map<std::string, Creator> registry;
  return registry;
}

void MotionPlannerFactory::registerCreator(const std::string& type, Creator creator)
{
  registry()[type] = std::move(creator);
}

MotionPlannerPtr MotionPlannerFactory::create(const std::string& type,
                                              const rclcpp::Node::SharedPtr& node,
                                              const moveit::core::RobotModelPtr& model)
{
  const auto& reg = registry();
  const auto it = reg.find(type);
  if (it == reg.end())
  {
    std::string msg = "unknown planner type '" + type + "', registered: ";
    for (const auto& [k, _] : reg)
      msg += k + " ";
    throw std::runtime_error(msg);
  }
  return it->second(node, model);
}

std::vector<std::string> MotionPlannerFactory::registeredTypes()
{
  std::vector<std::string> types;
  for (const auto& [k, _] : registry())
    types.push_back(k);
  return types;
}

}  // namespace rosim::planning
