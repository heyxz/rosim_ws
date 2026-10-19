/// MOTOMAN-GP12 CAD preparation for rosim: extracts the six joint axes from
/// the vendor STEP (7 solids = base + 6 links, millimeters, all-axes-zero
/// export pose) and exports per-link binary STLs in link-local frames
/// (meters). Usage:
///
///   gp12_prepare <gp12.stp>               probe: cylinder/axis table + checks
///   gp12_prepare <gp12.stp> export <dir> [lin_mm] [ang_rad]
///                                          probe + write base_link.stl,
///                                          link1..5.stl, flange.stl into <dir>
///                                          (tessellation defaults 1.0 mm / 0.3 rad)
///
/// The JointCal table below is the single source of truth shared by the STL
/// export and the URDF joint origins: W[k] is the anchor point of joint k's
/// axis in the CAD frame (mm). Link k's mesh is translated by -W[k]; the URDF
/// origin of joint k is W[k] - W[k-1] (all axes canonical, rpy 0).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include "rosim_core/occ_geometry_service.h"

namespace
{

constexpr const char* LINK_NAMES[7] = { "base_link", "link1", "link2",
                                        "link3", "link4", "link5", "flange" };
// Expected canonical axis of j1..j6 at the CAD zero pose (S,L,U,R,B,T).
constexpr const char* JOINT_NAMES[6] = { "j1", "j2", "j3", "j4", "j5", "j6" };
constexpr const char* EXPECT_DIR[6] = { "z", "y", "y", "x", "y", "x" };

/// Baked from probe output and cross-checked link-by-link against
/// ROS-Industrial motoman_gp12_support/urdf/gp12_macro.xacro (noetic-devel):
/// W anchors equal the ros-i frame origins, axes are the manufacturer's
/// positive directions (j3..j6 negative in CAD frame).
/// B evidence: s6 r50 journal @(795.0,3.0,1264.0) has z exactly on the R/T
/// line (spherical wrist, ros-i B origin is 0 0 0); wrist-centre max radial
/// 155+614+670.5=1439.5mm vs datasheet reach 1440mm; the s5 fork bores
/// r48.5 @(802.9,+/-84,1262.9) cannot mate with a r50 journal (interference)
/// and s6 spans y only [-64.5,69] so it never reaches the fork plates.
bool BAKED = true;
struct JointCal
{
  double w[3];
  double axis[3];
};
//                    j1(S)                      j2(L)                        j3(U)
JointCal CAL[6] = { { { 0, 0, 0 }, { 0, 0, 1 } },    { { 155, 0, 450 }, { 0, 1, 0 } }, { { 155, 0, 1064 }, { 0, -1, 0 } },
                    // j4(R)                        j5(B)                        j6(T)
                    { { 795, 0, 1264 }, { -1, 0, 0 } }, { { 795, 0, 1264 }, { 0, -1, 0 } }, { { 795, 0, 1264 }, { -1, 0, 0 } } };

struct CylinderHit
{
  int solid;         // 1-based
  double radius_mm;
  double area_mm2;
  gp_Pnt point;      // axis anchor (surface axis location)
  gp_Dir dir;
};

/// Nearest of +/-x,+/-y,+/-z; returns index 0..5 and the deviation in degrees.
int snapDir(const gp_Dir& d, double& dev_deg)
{
  static const double C[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 },
                                  { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
  int best = 0;
  double best_dot = -2.0;
  for (int i = 0; i < 6; ++i)
  {
    const double dot = d.X() * C[i][0] + d.Y() * C[i][1] + d.Z() * C[i][2];
    if (dot > best_dot)
    {
      best_dot = dot;
      best = i;
    }
  }
  dev_deg = std::acos(std::min(1.0, std::max(-1.0, best_dot))) * 180.0 / M_PI;
  return best;
}

const char* dirName(int i)
{
  static const char* N[6] = { "+x", "-x", "+y", "-y", "+z", "-z" };
  return N[i];
}

std::vector<TopoDS_Solid> collectSolids(const TopoDS_Shape& shape)
{
  std::vector<TopoDS_Solid> out;
  for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next())
    out.push_back(TopoDS::Solid(ex.Current()));
  return out;
}

/// All cylindrical faces of a solid above the size filter, sorted by area.
std::vector<CylinderHit> cylindersOf(const TopoDS_Solid& solid, int index,
                                     double min_r_mm, double min_area_mm2)
{
  std::vector<CylinderHit> hits;
  for (TopExp_Explorer ex(solid, TopAbs_FACE); ex.More(); ex.Next())
  {
    const TopoDS_Face face = TopoDS::Face(ex.Current());
    BRepAdaptor_Surface surf(face);
    if (surf.GetType() != GeomAbs_Cylinder)
      continue;
    const gp_Cylinder cyl = surf.Cylinder();
    if (cyl.Radius() < min_r_mm)
      continue;
    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    if (props.Mass() < min_area_mm2)
      continue;
    hits.push_back({ index, cyl.Radius(), props.Mass(), cyl.Axis().Location(),
                     cyl.Axis().Direction() });
  }
  std::sort(hits.begin(), hits.end(),
            [](const CylinderHit& a, const CylinderHit& b) { return a.area_mm2 > b.area_mm2; });
  return hits;
}

void probe(const std::string& file)
{
  const TopoDS_Shape shape = rosim::core::OccGeometryService::loadShape(file);
  const std::vector<TopoDS_Solid> solids = collectSolids(shape);
  std::cout << "solids: " << solids.size() << "\n";
  if (solids.size() != 7)
    std::cerr << "WARNING: expected 7 solids (base + 6 links)\n";

  // Per-solid bbox + volume (mass estimate input).
  for (size_t i = 0; i < solids.size(); ++i)
  {
    Bnd_Box bb;
    BRepBndLib::Add(solids[i], bb);
    double x0, y0, z0, x1, y1, z1;
    bb.Get(x0, y0, z0, x1, y1, z1);
    GProp_GProps props;
    BRepGProp::VolumeProperties(solids[i], props);
    std::cout << "solid " << (i + 1) << " (" << LINK_NAMES[i] << ") bbox[" << std::fixed
              << std::setprecision(1) << x0 << "," << y0 << "," << z0 << "]-[" << x1 << "," << y1
              << "," << z1 << "] vol=" << std::setprecision(0) << props.Mass() / 1000.0
              << " cm3 (~" << props.Mass() / 1000.0 * 7.85 * 0.35 << " kg shell-est)\n";
  }

  // Cylinder table per solid.
  std::vector<std::vector<CylinderHit>> per_solid;
  for (size_t i = 0; i < solids.size(); ++i)
    per_solid.push_back(cylindersOf(solids[i], static_cast<int>(i) + 1, 10.0, 500.0));

  std::cout << "\ncylinder faces (r>10mm, area>500mm2, by area):\n";
  for (const auto& hits : per_solid)
    for (const auto& h : hits)
    {
      double dev;
      const int s = snapDir(h.dir, dev);
      std::cout << "  s" << h.solid << " r=" << std::setw(5) << std::fixed << std::setprecision(1)
                << h.radius_mm << " A=" << std::setw(7) << std::setprecision(0) << h.area_mm2
                << " axis@(" << std::setprecision(1) << h.point.X() << "," << h.point.Y() << ","
                << h.point.Z() << ") " << dirName(s) << " dev=" << std::setprecision(2) << dev
                << "deg" << ((dev > 0.5) ? "  <-- NON-CANONICAL" : "") << "\n";
    }

  // Suggestions: joint k lives at the s(k)/s(k+1) interface with expected dir.
  std::cout << "\ninterface suggestions:\n";
  for (int k = 0; k < 6; ++k)
  {
    std::cout << "  " << JOINT_NAMES[k] << " (" << EXPECT_DIR[k] << "-dir, s" << (k + 1) << "/s"
              << (k + 2) << "):";
    int shown = 0;
    for (const auto& hits : per_solid)
      for (const auto& h : hits)
      {
        if (shown >= 3)
          break;
        if (h.solid != k + 1 && h.solid != k + 2)
          continue;
        double dev;
        const std::string want = std::string(EXPECT_DIR[k]);
        const std::string got = dirName(snapDir(h.dir, dev));
        if (got != "+" + want && got != "-" + want)
          continue;
        std::cout << " [s" << h.solid << " r" << std::setprecision(0) << h.radius_mm << " @("
                  << std::setprecision(1) << h.point.X() << "," << h.point.Y() << "," << h.point.Z()
                  << ")]";
        ++shown;
      }
    std::cout << "\n";
  }
}

void calChecks()
{
  if (!BAKED)
  {
    std::cout << "\nJointCal not baked — skipping chain checks\n";
    return;
  }
  std::cout << "\nchain check (CAD mm): ";
  double sum = 0.0;
  for (int k = 0; k < 6; ++k)
  {
    const double dx = CAL[k].w[0] - (k ? CAL[k - 1].w[0] : 0.0);
    const double dy = CAL[k].w[1] - (k ? CAL[k - 1].w[1] : 0.0);
    const double dz = CAL[k].w[2] - (k ? CAL[k - 1].w[2] : 0.0);
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    std::cout << JOINT_NAMES[k] << "->" << (k ? JOINT_NAMES[k - 1] : "base") << "="
              << std::setprecision(1) << d << "  ";
  }
  // |S->L| + |L->U| + |U->R| against the datasheet reach span.
  const auto dist = [](int a, int b) {
    const double dx = CAL[a].w[0] - CAL[b].w[0], dy = CAL[a].w[1] - CAL[b].w[1],
                 dz = CAL[a].w[2] - CAL[b].w[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };
  std::cout << "\n|SL|=" << dist(0, 1) << " |LU|=" << dist(1, 2) << " |UR|=" << dist(2, 3)
            << " sum=" << (dist(0, 1) + dist(1, 2) + dist(2, 3))
            << " (reach spec 1440 +/- 50)\n";
  std::cout << "flange(T anchor)=(" << CAL[5].w[0] << "," << CAL[5].w[1] << "," << CAL[5].w[2]
            << ") -> tool0 ≈ +6mm along axis, tool_tip ≈ +86mm\n";
}

void writeBinaryStl(const std::string& path, const rosim::core::MeshData& mesh)
{
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot open " + path);
  const char header[80] = { 0 };
  f.write(header, 80);
  const uint32_t n = static_cast<uint32_t>(mesh.triangles.size() / 3);
  f.write(reinterpret_cast<const char*>(&n), 4);
  for (size_t t = 0; t < mesh.triangles.size(); t += 3)
  {
    const Eigen::Vector3d a(mesh.vertices[3 * mesh.triangles[t]], mesh.vertices[3 * mesh.triangles[t] + 1],
                            mesh.vertices[3 * mesh.triangles[t] + 2]);
    const Eigen::Vector3d b(mesh.vertices[3 * mesh.triangles[t + 1]],
                            mesh.vertices[3 * mesh.triangles[t + 1] + 1],
                            mesh.vertices[3 * mesh.triangles[t + 1] + 2]);
    const Eigen::Vector3d c(mesh.vertices[3 * mesh.triangles[t + 2]],
                            mesh.vertices[3 * mesh.triangles[t + 2] + 1],
                            mesh.vertices[3 * mesh.triangles[t + 2] + 2]);
    Eigen::Vector3d nrm = (b - a).cross(c - a);
    if (nrm.norm() > 1e-12)
      nrm.normalize();
    const float tri[12] = { static_cast<float>(nrm.x()), static_cast<float>(nrm.y()),
                            static_cast<float>(nrm.z()), static_cast<float>(a.x()),
                            static_cast<float>(a.y()), static_cast<float>(a.z()),
                            static_cast<float>(b.x()), static_cast<float>(b.y()),
                            static_cast<float>(b.z()), static_cast<float>(c.x()),
                            static_cast<float>(c.y()), static_cast<float>(c.z()) };
    const uint16_t zero = 0;
    f.write(reinterpret_cast<const char*>(tri), 48);
    f.write(reinterpret_cast<const char*>(&zero), 2);
  }
}

void exportStls(const std::string& file, const std::string& dir, double lin_mm,
                double ang_rad)
{
  if (!BAKED)
    throw std::runtime_error("JointCal not baked — run probe first and fill CAL[]");
  const TopoDS_Shape shape = rosim::core::OccGeometryService::loadShape(file);
  const std::vector<TopoDS_Solid> solids = collectSolids(shape);
  if (solids.size() != 7)
    throw std::runtime_error("expected 7 solids");

  size_t total_tris = 0;
  for (size_t i = 0; i < solids.size(); ++i)
  {
    // Link i (0-based) lives in the frame of joint i (0-based; base = world).
    const double w[3] = { i ? CAL[i - 1].w[0] : 0.0, i ? CAL[i - 1].w[1] : 0.0,
                          i ? CAL[i - 1].w[2] : 0.0 };
    rosim::core::MeshData mesh;
    rosim::core::OccGeometryService::tessellate(solids[i], lin_mm, ang_rad, mesh);
    for (size_t v = 0; v + 2 < mesh.vertices.size(); v += 3)
    {
      mesh.vertices[v] = (mesh.vertices[v] - w[0]) * 1e-3;
      mesh.vertices[v + 1] = (mesh.vertices[v + 1] - w[1]) * 1e-3;
      mesh.vertices[v + 2] = (mesh.vertices[v + 2] - w[2]) * 1e-3;
    }
    const std::string path = dir + "/" + LINK_NAMES[i] + ".stl";
    writeBinaryStl(path, mesh);
    total_tris += mesh.triangles.size() / 3;
    double vmax = 0.0;
    for (const double c : mesh.vertices)
      vmax = std::max(vmax, std::abs(c));
    std::cout << "wrote " << path << ": " << mesh.triangles.size() / 3 << " tris, |coord|max="
              << std::setprecision(4) << vmax << " m\n";
  }
  std::cout << "total triangles: " << total_tris << " (deflection " << lin_mm << " mm / "
            << ang_rad << " rad)\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: gp12_prepare <gp12.stp> [export <dir> [lin_mm] [ang_rad]]\n";
    return 1;
  }
  try
  {
    probe(argv[1]);
    calChecks();
    if (argc >= 4 && std::string(argv[2]) == "export")
    {
      const double lin_mm = (argc >= 5) ? std::stod(argv[4]) : 1.0;
      const double ang_rad = (argc >= 6) ? std::stod(argv[5]) : 0.3;
      exportStls(argv[1], argv[3], lin_mm, ang_rad);
    }
  }
  catch (const std::exception& ex)
  {
    std::cerr << "error: " << ex.what() << "\n";
    return 1;
  }
  return 0;
}
