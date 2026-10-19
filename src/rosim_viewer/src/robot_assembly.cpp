// Legacy viewer robot display: builds link actors from the robot_description
// parameter and applies incoming /joint_states as FK.

#include "rosim_viewer/robot_assembly.h"

namespace rosim::viewer
{

RobotAssembly::RobotAssembly(const rclcpp::Node::SharedPtr& node, QObject* parent)
  : QObject(parent)
{
  try
  {
    robot_model_loader::RobotModelLoader::Options opts("robot_description");
    opts.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoader loader(node, opts);
    model_ = const_cast<moveit::core::RobotModelPtr&>(loader.getModel());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rosim.viewer.robot"),
                 "robot model load failed: %s", ex.what());
    return;
  }
  if (!model_)
    return;

  state_ = std::make_shared<moveit::core::RobotState>(model_);
  state_->setToDefaultValues();
  state_->updateLinkTransforms();

  actors_ = std::make_unique<rosim::core::RobotLinkActors>();
  if (!actors_->init(model_))
    return;

  state_->update();
  actors_->updateFromState(*state_);
  valid_ = true;
}

void RobotAssembly::addToScene(vtkRenderer* renderer)
{
  if (valid_)
    actors_->addToRenderer(renderer);
}

void RobotAssembly::onJointState(sensor_msgs::msg::JointState state)
{
  if (!valid_ || state.name.size() != state.position.size())
    return;
  try
  {
    state_->setVariablePositions(state.name, state.position);
    state_->updateLinkTransforms();
    actors_->updateFromState(*state_);
  }
  catch (const std::exception&)
  {
    // partial joint sets from other publishers — ignore quietly
  }
}

}  // namespace rosim::viewer
