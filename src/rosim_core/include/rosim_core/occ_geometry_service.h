#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Trsf.hxx>

#include <geometric_shapes/shapes.h>

namespace rosim::core
{

/// Simple triangle soup (vertices flattened x,y,z; triangles flattened i,j,k).
struct MeshData
{
  std::vector<double> vertices;
  std::vector<uint32_t> triangles;
};

/// One selectable edge, discretized in the WORLD frame.
struct EdgeData
{
  uint32_t index = 0;    ///< TopExp_Explorer enumeration order (stable per file+lib)
  double length = 0.0;   ///< meters
  bool closed = false;
  std::vector<Eigen::Vector3d> polyline;  ///< world frame, ordered along the curve
};

/// An imported workpiece: shape in its local frame + edges in world frame.
struct WorkpieceModel
{
  std::string model_id;
  std::string file_path;
  Eigen::Isometry3d world_pose = Eigen::Isometry3d::Identity();
  TopoDS_Shape shape;                ///< local frame
  std::vector<EdgeData> edges;       ///< world frame
};

/// \brief Stateless OCC helpers shared by the viewer and the planning node.
///
/// Edge index consistency: both processes load the same file through this
/// service, and TopExp_Explorer enumerates TopAbs_EDGE in the same order,
/// so index N is the same physical edge everywhere.
class OccGeometryService
{
public:
  /// Load .step/.stp/.brep by extension. Throws std::runtime_error on failure.
  static TopoDS_Shape loadShape(const std::string& file_path);

  /// Full import: load + extract edges at the given world pose.
  /// Skips degenerate and tiny (<1mm) edges.
  static std::shared_ptr<WorkpieceModel> importModel(const std::string& file_path,
                                                     const Eigen::Isometry3d& world_pose,
                                                     const std::string& model_id);

  /// Enumerate TopAbs_EDGE sub-shapes in stable order.
  static std::vector<TopoDS_Edge> topologicalEdges(const TopoDS_Shape& shape);

  /// Discretize an edge into roughly arc-length-uniform points (deflection-bounded).
  /// Returns points in the shape's local frame.
  static std::vector<Eigen::Vector3d> discretizeEdge(const TopoDS_Edge& edge,
                                                     double max_spacing,
                                                     double deflection = 5e-4);

  /// Edge arc length in meters.
  static double edgeLength(const TopoDS_Edge& edge);

  /// Whether the edge curve is periodic and spans a full period (e.g. circle).
  static bool edgeIsClosed(const TopoDS_Edge& edge);

  /// Tessellate the shape (BRepMesh) into a triangle soup, local frame.
  static void tessellate(const TopoDS_Shape& shape, double lin_deflection,
                         double angular_deflection, MeshData& out);

  /// Build a moveit/geometric_shapes collision mesh from a tessellation,
  /// placed at world_pose. Empty shape ptr if the mesh is empty.
  static shapes::ShapePtr toCollisionMesh(const MeshData& mesh,
                                          const Eigen::Isometry3d& world_pose);

  /// Eigen -> OCC transform helper.
  static gp_Trsf eigenToTrsf(const Eigen::Isometry3d& pose);
};

}  // namespace rosim::core
