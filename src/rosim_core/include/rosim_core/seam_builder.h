#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "rosim_core/occ_geometry_service.h"

namespace rosim::core
{

/// One source edge inside a seam, in travel order.
struct SeamPart
{
  uint32_t edge_index = 0;
  bool reversed = false;  ///< that edge was walked end -> start
};

/// Result of merging edges into one seam.
struct SeamBuildResult
{
  bool ok = false;
  std::string message;
  std::vector<Eigen::Vector3d> polyline;  ///< merged, world frame, travel order
  std::vector<SeamPart> parts;            ///< ordered source edges
  double length = 0.0;
  bool closed = false;
};

/// Merge endpoint-connected edges into a single continuous seam.
///
/// The rules are deliberately strict: a wrong merge silently produces a wrong
/// weld path, so ambiguous input is rejected with a reason instead of guessed.
///  - every requested edge must exist and be listed once;
///  - a closed edge (full circle) can only be a seam of its own;
///  - endpoints closer than `endpoint_tolerance` are the same topological node;
///  - no node may carry more than two incident edges (a branch has no order);
///  - the selection must be one connected chain (exactly two end nodes) or one
///    closed loop (no end nodes).
SeamBuildResult buildSeam(const std::vector<EdgeData>& edges,
                          const std::vector<uint32_t>& indices,
                          double endpoint_tolerance = 5e-4);

/// True if a and b are within tol (used for joint de-duplication too).
bool pointsCoincide(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol);

}  // namespace rosim::core
