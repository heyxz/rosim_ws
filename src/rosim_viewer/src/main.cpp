#include <QApplication>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rosim_viewer/main_window.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // MUST happen before QApplication is constructed (VTK/Qt GL context).
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

  QApplication app(argc, argv);

  // queued cross-thread signal payloads need metatype registration
  qRegisterMetaType<sensor_msgs::msg::JointState>("sensor_msgs::msg::JointState");

  // rclcpp's signal handler intercepts SIGINT/SIGTERM and only invalidates
  // the rclcpp context — the Qt event loop keeps running and the process
  // never exits (launch teardown then has to SIGKILL us). Poll the context
  // instead and route it into a clean Qt quit: exec() returns, ~MainWindow
  // joins the ROS spin thread, process exits, and the launch-side
  // OnProcessExit handler shuts the whole session down.
  QTimer ros_watch;
  QObject::connect(&ros_watch, &QTimer::timeout, [&]() {
    if (!rclcpp::ok())
      QApplication::quit();
  });
  ros_watch.start(200);

  rosim::viewer::MainWindow window;
  window.show();

  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
