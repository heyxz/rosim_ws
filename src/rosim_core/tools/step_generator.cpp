/// Generates the demo workpiece STEP: a base plate fused with a cylindrical
/// boss and a vertical rib, giving a mix of straight, arc and circular edges
/// for edge-following demos. Usage: rosim_step_generator <out.step>

#include <iostream>
#include <string>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include "rosim_core/occ_geometry_service.h"

int main(int argc, char** argv)
{
  const std::string out = (argc > 1) ? argv[1] : "workpiece_sample.step";

  // Base plate 300 x 160 x 20 mm, local origin at its corner.
  TopoDS_Shape plate = BRepPrimAPI_MakeBox(0.30, 0.16, 0.02).Shape();

  // Cylindrical boss on top of the plate (interesting circular edges).
  TopoDS_Shape boss = BRepPrimAPI_MakeCylinder(
      gp_Ax2(gp_Pnt(0.22, 0.08, 0.02), gp_Dir(0, 0, 1)), 0.035, 0.06).Shape();

  // Vertical rib for tall straight edges.
  TopoDS_Shape rib = BRepPrimAPI_MakeBox(gp_Pnt(0.02, 0.06, 0.02), 0.02, 0.04, 0.09).Shape();

  TopoDS_Shape workpiece = BRepAlgoAPI_Fuse(plate, boss);
  workpiece = BRepAlgoAPI_Fuse(workpiece, rib);

  STEPControl_Writer writer;
  writer.Transfer(workpiece, STEPControl_AsIs);
  const IFSelect_ReturnStatus status = writer.Write(out.c_str());
  if (status != IFSelect_RetDone)
  {
    std::cerr << "failed to write " << out << "\n";
    return 1;
  }

  // Sanity report through the same service the runtime uses.
  try
  {
    const auto model = rosim::core::OccGeometryService::importModel(
        out, Eigen::Isometry3d::Identity(), "sample");
    std::cout << "wrote " << out << ": " << model->edges.size() << " selectable edges\n";
    for (const auto& e : model->edges)
      std::cout << "  edge " << e.index << ": length=" << e.length
                << " m, closed=" << (e.closed ? "yes" : "no")
                << ", points=" << e.polyline.size() << "\n";
  }
  catch (const std::exception& ex)
  {
    std::cerr << "re-read failed: " << ex.what() << "\n";
    return 1;
  }
  return 0;
}
