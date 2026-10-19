// rosim_server entry point. Forces GLX swap behaviour before the first VTK
// context (avoids a compositor-induced ~1 s stall per frame), then spins the
// server node on a multithreaded executor.

#include <cstdlib>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rosim_server/server_node.h"

int main(int argc, char** argv)
{
  // Offscreen GLX rendering must not wait for a monitor vsync: on an idle
  // desktop the NVIDIA driver can drop the (composited) display to a ~1 Hz
  // low-power refresh, which turns every offscreen swap into a ~1 s stall and
  // the render loop collapses from 12 fps to 0.5 fps. Must be set before the
  // first GL context is created (i.e. before any VTK render), so do it first.
  setenv("__GL_SYNC_TO_VBLANK", "0", 1);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosim::server::ServerNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
