// OCC geometry service: STEP/BREP loading, stable edge enumeration,
// arc-length edge discretization and tessellation. Shared by the planning node
// and the server so both agree on edge indices and display geometry.

#include "rosim_core/occ_geometry_service.h"

#include <algorithm>
#include <stdexcept>
#include <cmath>

#include <BRep_Builder.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>


namespace rosim::core
{

namespace
{
std::string lowerExt(const std::string& path)
{
  auto dot = path.find_last_of('.');
  std::string ext = (dot == std::string::npos) ? "" : path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  return ext;
}

Eigen::Vector3d toEigen(const gp_Pnt& p)
{
  return Eigen::Vector3d(p.X(), p.Y(), p.Z());
}
}  // namespace

TopoDS_Shape OccGeometryService::loadShape(const std::string& file_path)
{
  const std::string ext = lowerExt(file_path);

  if (ext == "step" || ext == "stp")
  {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(file_path.c_str());
    if (status != IFSelect_RetDone)
      throw std::runtime_error("STEP read failed: " + file_path);
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull())
      throw std::runtime_error("STEP contained no shape: " + file_path);
    return shape;
  }
  if (ext == "brep")
  {
    TopoDS_Shape shape;
    BRep_Builder builder;
    if (!BRepTools::Read(shape, file_path.c_str(), builder))
      throw std::runtime_error("BREP read failed: " + file_path);
    return shape;
  }
  throw std::runtime_error("Unsupported model format '" + ext + "' (use .step/.stp/.brep): " + file_path);
}

std::vector<TopoDS_Edge> OccGeometryService::topologicalEdges(const TopoDS_Shape& shape)
{
  std::vector<TopoDS_Edge> edges;
  TopExp_Explorer explorer(shape, TopAbs_EDGE);
  for (; explorer.More(); explorer.Next())
    edges.push_back(TopoDS::Edge(explorer.Current()));
  return edges;
}

double OccGeometryService::edgeLength(const TopoDS_Edge& edge)
{
  BRepAdaptor_Curve curve(edge);
  return GCPnts_AbscissaPoint::Length(curve);
}

bool OccGeometryService::edgeIsClosed(const TopoDS_Edge& edge)
{
  try
  {
    BRepAdaptor_Curve curve(edge);
    if (!curve.IsPeriodic())
      return false;
    return std::abs((curve.LastParameter() - curve.FirstParameter()) - curve.Period()) <
           1e-6 * curve.Period();
  }
  catch (const Standard_Failure&)
  {
    return false;
  }
}

std::vector<Eigen::Vector3d> OccGeometryService::discretizeEdge(const TopoDS_Edge& edge,
                                                                double max_spacing,
                                                                double deflection)
{
  std::vector<Eigen::Vector3d> points;
  BRepAdaptor_Curve curve(edge);

  const double length = GCPnts_AbscissaPoint::Length(curve);
  if (length < 1e-9)
    return points;

  // Arc-length-uniform sampling, bounded by curvature deflection: take the
  // denser of UniformAbscissa / QuasiUniformDeflection, merged in parameter.
  GCPnts_UniformAbscissa uniform(curve, std::max(max_spacing, 1e-5));
  GCPnts_QuasiUniformDeflection defl(curve, std::max(deflection, 1e-6));

  std::vector<double> params;
  params.reserve(256);
  if (uniform.IsDone())
    for (int i = 1; i <= uniform.NbPoints(); ++i)
      params.push_back(uniform.Parameter(i));
  if (defl.IsDone())
    for (int i = 1; i <= defl.NbPoints(); ++i)
      params.push_back(defl.Parameter(i));

  if (params.empty())
  {
    params.push_back(curve.FirstParameter());
    params.push_back(curve.LastParameter());
  }

  std::sort(params.begin(), params.end());
  params.erase(std::unique(params.begin(), params.end(),
                           [](double a, double b) { return std::abs(a - b) < 1e-9; }),
               params.end());

  // Guarantee endpoints are present.
  if (params.front() - curve.FirstParameter() > 1e-9)
    params.insert(params.begin(), curve.FirstParameter());
  if (curve.LastParameter() - params.back() > 1e-9)
    params.push_back(curve.LastParameter());

  points.reserve(params.size());
  for (double p : params)
    points.push_back(toEigen(curve.Value(p)));
  return points;
}

std::shared_ptr<WorkpieceModel> OccGeometryService::importModel(const std::string& file_path,
                                                                const Eigen::Isometry3d& world_pose,
                                                                const std::string& model_id)
{
  auto model = std::make_shared<WorkpieceModel>();
  model->model_id = model_id;
  model->file_path = file_path;
  model->world_pose = world_pose;
  model->shape = loadShape(file_path);

  uint32_t next_index = 0;
  for (const auto& edge : topologicalEdges(model->shape))
  {
    if (BRep_Tool::Degenerated(edge))
      continue;
    const double len = edgeLength(edge);
    if (len < 1e-3)  // skip sub-millimeter edges (seams, artifacts)
      continue;

    EdgeData data;
    data.index = next_index++;
    data.length = len;
    data.closed = edgeIsClosed(edge);
    auto pts = discretizeEdge(edge, std::min(5e-3, len / 8.0));
    if (pts.size() < 2)
      continue;
    data.polyline.reserve(pts.size());
    for (const auto& p : pts)
      data.polyline.push_back(world_pose * p);
    model->edges.push_back(std::move(data));
  }
  return model;
}

void OccGeometryService::tessellate(const TopoDS_Shape& shape, double lin_deflection,
                                    double angular_deflection, MeshData& out)
{
  out.vertices.clear();
  out.triangles.clear();

  BRepMesh_IncrementalMesh mesher(shape, lin_deflection, false, angular_deflection, true);
  (void)mesher;

  TopExp_Explorer face_it(shape, TopAbs_FACE);
  for (; face_it.More(); face_it.Next())
  {
    const TopoDS_Face& face = TopoDS::Face(face_it.Current());
    TopLoc_Location location;
    const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull())
      continue;

    const uint32_t vertex_offset = static_cast<uint32_t>(out.vertices.size() / 3);
    const int nb_nodes = triangulation->NbNodes();
    for (int i = 1; i <= nb_nodes; ++i)
    {
      gp_Pnt node = triangulation->Node(i);
      if (!location.IsIdentity())
        node.Transform(location.Transformation());
      out.vertices.push_back(node.X());
      out.vertices.push_back(node.Y());
      out.vertices.push_back(node.Z());
    }

    // BRep_Tool::Triangulation returns the mesh in the face's own parametric
    // orientation. On a face flagged REVERSED (the normal case for half of a
    // solid's skin), those triangles wind inward, so a front-face-culling
    // renderer (three.js) drops them and the part shows black holes. Flip the
    // winding for reversed faces to get a consistently outward shell.
    const bool reversed = (face.Orientation() == TopAbs_REVERSED);
    const int nb_triangles = triangulation->NbTriangles();
    for (int i = 1; i <= nb_triangles; ++i)
    {
      const Poly_Triangle& tri = triangulation->Triangle(i);
      // Skip degenerate triangles (repeated indices).
      if (tri.Value(1) == tri.Value(2) || tri.Value(2) == tri.Value(3) ||
          tri.Value(1) == tri.Value(3))
        continue;
      out.triangles.push_back(vertex_offset + tri.Value(1) - 1);
      out.triangles.push_back(vertex_offset + (reversed ? tri.Value(3) : tri.Value(2)) - 1);
      out.triangles.push_back(vertex_offset + (reversed ? tri.Value(2) : tri.Value(3)) - 1);
    }
  }
}

shapes::ShapePtr OccGeometryService::toCollisionMesh(const MeshData& mesh,
                                                     const Eigen::Isometry3d& world_pose)
{
  if (mesh.vertices.empty() || mesh.triangles.empty())
    return nullptr;

  const unsigned int vcount = static_cast<unsigned int>(mesh.vertices.size() / 3);
  const unsigned int tcount = static_cast<unsigned int>(mesh.triangles.size() / 3);
  auto* smesh = new shapes::Mesh(vcount, tcount);

  for (size_t i = 0; i < vcount; ++i)
  {
    Eigen::Vector3d p = world_pose * Eigen::Vector3d(mesh.vertices[3 * i],
                                                     mesh.vertices[3 * i + 1],
                                                     mesh.vertices[3 * i + 2]);
    smesh->vertices[3 * i] = p.x();
    smesh->vertices[3 * i + 1] = p.y();
    smesh->vertices[3 * i + 2] = p.z();
  }
  for (size_t i = 0; i < mesh.triangles.size(); ++i)
    smesh->triangles[i] = mesh.triangles[i];

  smesh->computeTriangleNormals();
  smesh->computeVertexNormals();
  return shapes::ShapePtr(smesh);
}

gp_Trsf OccGeometryService::eigenToTrsf(const Eigen::Isometry3d& pose)
{
  gp_Trsf trsf;
  const Eigen::Matrix3d& rot = pose.linear();
  const Eigen::Vector3d& trans = pose.translation();
  // gp_Mat is row-major constructed from 9 values x1..x3, y1..y3, z1..z3.
  gp_Mat rot_mat(rot(0, 0), rot(0, 1), rot(0, 2),
                 rot(1, 0), rot(1, 1), rot(1, 2),
                 rot(2, 0), rot(2, 1), rot(2, 2));
  trsf.SetValues(rot_mat.Value(1, 1), rot_mat.Value(1, 2), rot_mat.Value(1, 3), trans.x(),
                 rot_mat.Value(2, 1), rot_mat.Value(2, 2), rot_mat.Value(2, 3), trans.y(),
                 rot_mat.Value(3, 1), rot_mat.Value(3, 2), rot_mat.Value(3, 3), trans.z());
  return trsf;
}

}  // namespace rosim::core
