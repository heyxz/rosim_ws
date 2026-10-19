#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace rosim::core
{

/// How to get through a sharp corner without stopping the arm.
///
/// A seam polyline has a tangent discontinuity at every corner. Traversing it at
/// non-zero TCP speed therefore requires either leaving the seam by a small
/// amount (arc blending, like FANUC CNT / KUKA C_DIS / MoveIt-Pilz
/// `blend_radius`) or rotating the wrist in place at the corner. This policy
/// expresses that trade-off as data so it can be tuned per workpiece/process.
struct CornerPolicy
{
  /// Corners shallower than this are treated as straight (no processing).
  double angle_threshold_deg = 8.0;
  /// Maximum allowed distance from the seam at a corner [m]. Drives the arc
  /// radius: r = dev / (1/cos(turn/2) - 1).
  double max_deviation = 5e-4;
  /// If the available segment length forces the radius below this, the corner
  /// cannot be blended and is handled by an in-place wrist rotation instead.
  // A fillet is preferred over an in-place pivot: the weld torch keeps moving
  // through the corner (slowing down is handled by the seam speed profile), and
  // the cartesian solver now uses an ABSOLUTE per-step jump limit, so even the
  // tight 1.2 mm fillet a 0.5 mm deviation budget yields for a 90 deg corner is
  // traversable. Only when the neighbouring segments leave no room for any arc
  // does the corner degrade to a pivot.
  double min_radius = 5e-5;
  /// Allow the in-place-rotation fallback. When false such a corner is simply
  /// left sharp (and will likely truncate the Cartesian run).
  bool pivot_fallback = true;
  /// Target distance between arc samples [m].
  double sample_step = 2e-3;
  /// Spread the orientation change over +/- this distance along the path [m].
  /// The arc itself is only a few millimetres long, so without a window the
  /// wrist still has to turn the whole corner angle in one or two steps.
  double orient_window = 0.02;
  /// Radians per sample when rotating in place.
  double pivot_step_rad = 0.10;
};

/// What happened at one corner (reported before execution starts).
struct CornerInfo
{
  std::size_t vertex = 0;              ///< index into the input polyline
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double turn_deg = 0.0;               ///< direction change at the corner
  double radius = 0.0;                 ///< arc radius used (0 for pivot)
  double deviation = 0.0;              ///< distance from the original vertex
  bool pivot = false;                  ///< handled by in-place rotation
};

struct CornerPlan
{
  /// Waypoints (same frame convention as the input) with corners replaced by
  /// arc samples (or repeated positions for pivots).
  std::vector<Eigen::Isometry3d> frames;
  std::vector<CornerInfo> corners;
  std::size_t pivot_count = 0;
  /// Human-readable summary of anything the operator should know before the
  /// move starts (e.g. "corner 3 needs pivot: no room for a 0.5 mm blend").
  std::string summary;
};

/// Replace sharp corners in `polyline` with tangent-continuous arcs, rotating
/// the tool orientation over the same window. `frames` must be one per polyline
/// point (as returned by EdgeFrameGenerator).
CornerPlan blendCorners(const std::vector<Eigen::Vector3d>& polyline,
                        const std::vector<Eigen::Isometry3d>& frames,
                        const CornerPolicy& policy);

}  // namespace rosim::core
