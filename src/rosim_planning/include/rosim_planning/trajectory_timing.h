#pragma once

#include <rclcpp/logger.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace rosim::planning
{

/// \brief Put a trajectory on an executable, uniform time grid.
///
/// The joint trajectory controller samples at 100 Hz and interpolates with cubic
/// splines, which needs two things from the message:
///
///   * no segment shorter than one control period (4.5 ms gaps cannot be
///     followed and the spline spikes on them), and
///   * a uniform grid with consistent joint velocities. With an irregular grid
///     (0.02 s next to 0.4 s) the spline overshoots and the arm visibly
///     oscillates; with empty velocities it assumes zero at every waypoint and
///     every segment becomes a small excursion.
///
/// This applies all three: clamp short segments to at least 2x the control
/// period, resample onto a uniform grid, then fill velocities by central
/// difference. The path itself is unchanged - only the timing and the tangents.
///
/// \param rt       trajectory to modify in place
/// \param model    robot model owning the trajectory's group
/// \param logger   logger used for the summary lines
/// \param period   uniform grid period in seconds (default 50 ms = 20 Hz)
void regularizeTiming(robot_trajectory::RobotTrajectory& rt,
                      const moveit::core::RobotModelConstPtr& model,
                      const rclcpp::Logger& logger,
                      double period = 0.05);

}  // namespace rosim::planning
