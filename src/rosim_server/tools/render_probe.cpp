// Dev tool: verify headless VTK offscreen rendering + JPEG capture on this
// machine, independent of the WebSocket server. Renders the sample workpiece
// and writes the frame to /tmp/rosim_frame.jpg.
//
//   ros2 run rosim_server render_probe [step_path]

#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_server/frame_renderer.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string path =
      (argc > 1) ? argv[1]
                 : "/home/xz/rosim_ws/src/rosim_description/meshes/workpiece_sample.step";

  try
  {
    const auto shape = rosim::core::OccGeometryService::loadShape(path);
    rosim::core::MeshData solid;
    rosim::core::OccGeometryService::tessellate(shape, 3e-4, 0.35, solid);

    const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    auto model = rosim::core::OccGeometryService::importModel(path, pose, "workpiece_1");

    rosim::server::FrameRenderer renderer(1280, 720);
    renderer.init(nullptr);  // no robot model for this probe
    renderer.addWorkpiece("workpiece_1", solid, model->edges, pose);
    renderer.setHighlightedEdge(0);

    const auto jpeg = renderer.captureJpeg(80);
    if (jpeg.empty())
    {
      std::cerr << "FAILED: captureJpeg returned no bytes\n";
      return 2;
    }

    std::ofstream out("/tmp/rosim_frame.jpg", std::ios::binary);
    out.write(reinterpret_cast<const char*>(jpeg.data()), jpeg.size());
    std::cout << "OK: " << jpeg.size() << " JPEG bytes, " << model->edges.size()
              << " edges, " << solid.triangles.size() / 3 << " triangles -> /tmp/rosim_frame.jpg\n";
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "FAILED: " << ex.what() << "\n";
    return 1;
  }
}
