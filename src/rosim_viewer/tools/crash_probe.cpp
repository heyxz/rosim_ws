/// Headless reproduction of the viewer's post-import display sequence:
/// loadShape -> tessellate(3e-4) -> buildSolidActor + buildEdgeActor per edge
/// -> offscreen render. Run under gdb to catch the SIGSEGV seen when importing
/// from the GUI.
///
///   ros2 run rosim_viewer crash_probe <path/to/workpiece.step>

#include <cstdio>
#include <string>
#include <vector>

#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_core/occ_to_vtk.h"

int main(int argc, char** argv)
{
  const std::string path = argc > 1 ? argv[1] :
      "/home/xz/rosim_ws/src/rosim_description/meshes/workpiece_sample.step";

  // --- exactly MainWindow::onImportFinished's local display load ---
  rosim::core::MeshData solid;
  const auto shape = rosim::core::OccGeometryService::loadShape(path);
  rosim::core::OccGeometryService::tessellate(shape, 3e-4, 0.35, solid);
  printf("tessellated: %zu verts, %zu tris\n", solid.vertices.size() / 3,
         solid.triangles.size() / 3);

  // edges: reuse extractEdges for realistic polylines (GUI gets them from the
  // service response; geometry identical)
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  auto model = rosim::core::OccGeometryService::importModel(path, pose, "probe");
  printf("edges: %zu\n", model->edges.size());

  vtkNew<vtkRenderer> renderer;
  auto solid_actor = rosim::core::buildSolidActor(solid);
  renderer->AddActor(solid_actor);
  for (const auto& e : model->edges)
  {
    auto actor = rosim::core::buildEdgeActor(e);
    renderer->AddActor(actor);
  }
  printf("actors added\n");

  vtkNew<vtkRenderWindow> window;
  window->SetOffScreenRendering(1);
  window->AddRenderer(renderer);
  window->Render();
  printf("rendered OK\n");
  return 0;
}
