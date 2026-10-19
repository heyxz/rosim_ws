#include "rosim_planning/trajectory_timing.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/logging.hpp>

namespace rosim::planning
{

namespace
{
constexpr double kMinSegment = 0.02;  // s, 2x the controller's 100 Hz period
}  // namespace

void regularizeTiming(robot_trajectory::RobotTrajectory& rt,
                      const moveit::core::RobotModelConstPtr& model,
                      const rclcpp::Logger& logger,
                      double period)
{
  const moveit::core::JointModelGroup* group = rt.getGroup();
  if (!group || !model || rt.size() < 3)
    return;

  // 1) No segment shorter than the controller period can be followed.
  std::size_t clamped = 0;
  for (std::size_t i = 1; i < rt.size(); ++i)
  {
    if (rt.getWayPointDurationFromPrevious(i) < kMinSegment)
    {
      rt.setWayPointDurationFromPrevious(i, kMinSegment);
      ++clamped;
    }
  }
  if (clamped > 0)
    RCLCPP_INFO(logger, "timing: clamped %zu segments shorter than 20 ms", clamped);

  // 2) Uniform time grid.
  const double total = rt.getWayPointDurationFromStart(rt.size() - 1);
  if (total <= 2.0 * period)
    return;

  const auto& vnames = group->getVariableNames();
  auto resampled = std::make_unique<robot_trajectory::RobotTrajectory>(model, group->getName());
  std::size_t k = 0;
  for (double t = 0.0; t <= total + 1e-9; t += period)
  {
    while (k + 2 < rt.size() && rt.getWayPointDurationFromStart(k + 1) < t)
      ++k;
    const double t0 = rt.getWayPointDurationFromStart(k);
    const double t1 = rt.getWayPointDurationFromStart(k + 1);
    const double alpha =
        t1 > t0 + 1e-9 ? std::min(1.0, std::max(0.0, (t - t0) / (t1 - t0))) : 1.0;
    moveit::core::RobotState st = *rt.getWayPointPtr(k);
    for (const std::string& vn : vnames)
    {
      const double a = rt.getWayPoint(k).getVariablePosition(vn);
      const double b = rt.getWayPoint(std::min(k + 1, rt.size() - 1)).getVariablePosition(vn);
      st.setVariablePosition(vn, a + alpha * (b - a));
    }
    st.update();
    resampled->addSuffixWayPoint(st, resampled->size() == 0 ? 0.0 : period);
  }
  if (resampled->size() < 3)
    return;
  const std::size_t before = rt.size();
  rt = std::move(*resampled);
  RCLCPP_INFO(logger, "timing: resampled %zu -> %zu waypoints on a uniform %.0f Hz grid", before,
              rt.size(), 1.0 / period);

  // 3) Velocities: central difference, zero at both ends. The controller's
  //    spline reads these as the tangents at each waypoint.
  const std::size_t n = rt.size();
  auto pos = [&](std::size_t i, const std::string& vn) {
    return rt.getWayPoint(i).getVariablePosition(vn);
  };
  for (std::size_t i = 0; i < n; ++i)
  {
    auto& wp = *rt.getWayPointPtr(i);
    const std::size_t i0 = i > 0 ? i - 1 : 0;
    const std::size_t i1 = i + 1 < n ? i + 1 : n - 1;
    const double span = rt.getWayPointDurationFromStart(i1) - rt.getWayPointDurationFromStart(i0);
    for (const std::string& vn : vnames)
    {
      const double v = (i == 0 || i + 1 == n || span < 1e-6)
                           ? 0.0
                           : (pos(i1, vn) - pos(i0, vn)) / span;
      wp.setVariableVelocity(vn, v);
    }
    wp.update();
  }
}

}  // namespace rosim::planning
