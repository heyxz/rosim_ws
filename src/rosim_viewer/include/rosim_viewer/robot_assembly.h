#pragma once

#include <memory>

#include <QObject>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vtkRenderer.h>

#include "rosim_core/robot_link_actors.h"

namespace rosim::viewer
{

/// \brief Robot display: loads the model from the node's robot_description
/// parameter, renders link actors, applies joint states (FK animation).
class RobotAssembly : public QObject
{
  Q_OBJECT

public:
  /// \param node node that carries the robot_description parameter
  /// (viewer needs no SRDF/kinematics — FK only).
  explicit RobotAssembly(const rclcpp::Node::SharedPtr& node, QObject* parent = nullptr);

  bool valid() const { return valid_; }
  void addToScene(vtkRenderer* renderer);
  const moveit::core::RobotModelPtr& model() const { return model_; }

public slots:
  void onJointState(sensor_msgs::msg::JointState state);

private:
  moveit::core::RobotModelPtr model_;
  moveit::core::RobotStatePtr state_;
  std::unique_ptr<rosim::core::RobotLinkActors> actors_;
  bool valid_ = false;
};

}  // namespace rosim::viewer
