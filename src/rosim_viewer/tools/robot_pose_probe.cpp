/// Headless check of the robot display transform path.
/// 1) FK sanity: print each link's global transform at home.
/// 2) VTK semantics: replicate updateFromState exactly (DeepCopy row-major ->
///    SetUserMatrix) on a bare vtkActor and print the EFFECTIVE GetMatrix().
/// If actor matrices come out ~identity while FK is spread out, the bug is the
/// UserMatrix path; if both are correct, the bug is viewer wiring/timing.
///
///   ros2 run rosim_viewer robot_pose_probe --ros-args --params-file <yaml>

#include <cstdio>

#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robot_pose_probe");

  robot_model_loader::RobotModelLoader::Options opts("robot_description");
  opts.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoader loader(node, opts);
  auto model = loader.getModel();
  if (!model)
  {
    std::cerr << "robot model load failed\n";
    return 1;
  }

  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  const auto* group = model->getJointModelGroup("manipulator");
  if (group)
    state.setToDefaultValues(group, "home");
  state.updateLinkTransforms();

  printf("%-12s %s\n", "link", "expected translation          vs stored/actor matrix translation");
  for (const auto* link : model->getLinkModels())
  {
    if (link->getShapes().empty())
      continue;
    const Eigen::Isometry3d tf = state.getGlobalLinkTransform(link->getName());

    // replicate the FIXED RobotLinkActors::updateFromState (element-wise
    // SetElement; the old flat[16]+DeepCopy path transposed the translation
    // into the bottom row and piled every link at the origin)
    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        matrix->SetElement(r, c, tf.linear()(r, c));
    matrix->SetElement(0, 3, tf.translation().x());
    matrix->SetElement(1, 3, tf.translation().y());
    matrix->SetElement(2, 3, tf.translation().z());
    matrix->SetElement(3, 3, 1.0);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetUserMatrix(matrix);
    const double m_after_setuser[3] = { actor->GetMatrix()->GetElement(0, 3),
                                        actor->GetMatrix()->GetElement(1, 3),
                                        actor->GetMatrix()->GetElement(2, 3) };
    const double user_readback[3] = { actor->GetUserMatrix() ? actor->GetUserMatrix()->GetElement(0, 3) : -999,
                                      actor->GetUserMatrix() ? actor->GetUserMatrix()->GetElement(1, 3) : -999,
                                      actor->GetUserMatrix() ? actor->GetUserMatrix()->GetElement(2, 3) : -999 };

    // variant B: SetUserTransform(vtkTransform built from the same matrix)
    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(matrix);
    auto actor_b = vtkSmartPointer<vtkActor>::New();
    actor_b->SetUserTransform(transform);
    const double m_after_transform[3] = { actor_b->GetMatrix()->GetElement(0, 3),
                                          actor_b->GetMatrix()->GetElement(1, 3),
                                          actor_b->GetMatrix()->GetElement(2, 3) };

    printf("%-12s expected (%6.3f,%6.3f,%6.3f) | SetUserMatrix: GetMatrix=(%6.3f,%6.3f,%6.3f) "
           "stored=(%6.3f,%6.3f,%6.3f) | SetUserTransform: GetMatrix=(%6.3f,%6.3f,%6.3f)\n",
           link->getName().c_str(), tf.translation().x(), tf.translation().y(), tf.translation().z(),
           m_after_setuser[0], m_after_setuser[1], m_after_setuser[2], user_readback[0], user_readback[1],
           user_readback[2], m_after_transform[0], m_after_transform[1], m_after_transform[2]);
  }
  rclcpp::shutdown();
  return 0;
}
