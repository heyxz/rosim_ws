/// Loads a STEP/BREP workpiece through OccGeometryService and prints edge and
/// tessellation statistics. Usage: test_occ_step <model.step> [lin_deflection_mm]

#include <iostream>
#include <string>

#include "rosim_core/occ_geometry_service.h"

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: test_occ_step <model.step|model.brep> [lin_deflection_m]\n";
    return 1;
  }
  const double defl = (argc > 2) ? std::stod(argv[2]) : 1e-3;

  try
  {
    const auto model = rosim::core::OccGeometryService::importModel(
        argv[1], Eigen::Isometry3d::Identity(), "test");

    std::cout << "file: " << model->file_path << "\n";
    std::cout << "edges (>=1mm): " << model->edges.size() << "\n";
    for (const auto& e : model->edges)
    {
      const auto& p0 = e.polyline.front();
      std::cout << "  [" << e.index << "] len=" << e.length << " closed=" << e.closed
                << " pts=" << e.polyline.size()
                << " start=(" << p0.x() << ", " << p0.y() << ", " << p0.z() << ")\n";
    }

    rosim::core::MeshData mesh;
    rosim::core::OccGeometryService::tessellate(model->shape, defl, 0.5, mesh);
    std::cout << "tessellation(defl=" << defl << "): "
              << mesh.vertices.size() / 3 << " verts, "
              << mesh.triangles.size() / 3 << " tris\n";

    auto collision = rosim::core::OccGeometryService::toCollisionMesh(
        mesh, Eigen::Isometry3d::Identity());
    std::cout << "collision mesh: " << (collision ? "OK" : "EMPTY") << "\n";
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "ERROR: " << ex.what() << "\n";
    return 1;
  }
}
