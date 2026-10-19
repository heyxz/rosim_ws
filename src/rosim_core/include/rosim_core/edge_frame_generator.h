#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

namespace rosim::core
{

/// \brief Turns an edge polyline into a sequence of tool frames (TCP poses).
///
/// Orientation conventions (mirror rosim_interfaces/action/FollowEdge.action):
///  - TANGENT (default): tool +Z = edge tangent; X/Y from a parallel-transport
///    style frame seeded by a world reference vector. No Frenet degeneracy on
///    straight edges, no sign flips, C0-continuous orientation.
///  - FIXED: constant orientation for the whole edge.
///  - NORMAL: reserved (V2): X from averaged adjacent-face normals.
class EdgeFrameGenerator
{
public:
  static constexpr uint8_t ORIENTATION_TANGENT = 0;
  static constexpr uint8_t ORIENTATION_FIXED = 1;
  static constexpr uint8_t ORIENTATION_NORMAL = 2;

  struct Config
  {
    uint8_t orientation_mode = ORIENTATION_TANGENT;
    Eigen::Quaterniond fixed_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d ref_up{ 0.0, 0.0, 1.0 };  ///< world reference for the transport frame
    double tangent_tilt = 0.0;  ///< TANGENT mode: rotate tool Z from the tangent toward
                                ///< the frame up-axis [rad] — welding lead angle. A
                                ///< straight torch with 0 tilt rides exactly on the edge
                                ///< line, so its body sweeps the workpiece on flat-top
                                ///< edges; tilting lifts the body off the surface.
  };

  /// \param polyline  world-frame points along the edge (>= 2)
  /// \return TCP poses (tool-tip frame: +Z along tangent) per polyline point.
  ///         Empty on invalid input.
  static std::vector<Eigen::Isometry3d> generate(const std::vector<Eigen::Vector3d>& polyline,
                                                 const Config& config);

  /// Tangent at point i (central difference, one-sided at the ends).
  static Eigen::Vector3d tangentAt(const std::vector<Eigen::Vector3d>& polyline, size_t i);

  /// Build one frame from a position + tangent using the reference vector.
  static Eigen::Isometry3d frameAt(const Eigen::Vector3d& position,
                                   const Eigen::Vector3d& tangent,
                                   const Eigen::Vector3d& ref_up);
};

}  // namespace rosim::core
