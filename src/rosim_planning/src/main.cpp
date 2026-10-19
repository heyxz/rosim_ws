#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rosim_planning/planning_node.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Planning can take seconds and the action server runs its goal in a worker
  // thread while services must stay responsive -> multithreaded executor.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<rosim::planning::PlanningNode>();
  node->init();  // model loading needs shared_from_this -> after construction
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
