// Corner handling for merged seams.
//
// A seam polyline is C0 at its corners: the tangent jumps, so traversing the
// corner at speed is impossible without either (a) leaving the seam by a small
// amount or (b) rotating the wrist in place while the TCP stands still.
// This module implements both, driven by CornerPolicy, and reports every
// decision up front so the operator can be warned before the move starts.

#include "rosim_core/corner_planner.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rosim::core
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond quatOf(const Eigen::Isometry3d& m)
{
  return Eigen::Quaterniond(m.linear());
}

std::string fmt(const Eigen::Vector3d& p)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(3);
  os << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")";
  return os.str();
}
}  // namespace

CornerPlan blendCorners(const std::vector<Eigen::Vector3d>& polyline,
                        const std::vector<Eigen::Isometry3d>& frames,
                        const CornerPolicy& policy)
{
  CornerPlan out;
  if (polyline.size() < 3 || polyline.size() != frames.size())
  {
    out.frames = frames;
    return out;
  }

  const std::size_t n = polyline.size();
  const double thr = policy.angle_threshold_deg * kPi / 180.0;
  const double dev_budget = std::max(policy.max_deviation, 0.0);

  // ---- pass 1: where are the corners? -------------------------------------
  // A corner is a *discontinuity*, not merely a bend: a finely discretized
  // circle turns ~8 deg at every vertex, which would otherwise be detected as
  // a corner at every sample. Compare each vertex against the local background
  // curvature instead of using an absolute threshold alone.
  std::vector<double> turn(n, 0.0);  // direction change at vertex i [rad]
  for (std::size_t i = 1; i + 1 < n; ++i)
  {
    const Eigen::Vector3d a = polyline[i] - polyline[i - 1];
    const Eigen::Vector3d b = polyline[i + 1] - polyline[i];
    const double la = a.norm(), lb = b.norm();
    if (la < 1e-9 || lb < 1e-9)
      continue;
    const double c = std::max(-1.0, std::min(1.0, (a / la).dot(b / lb)));
    turn[i] = std::acos(c);
  }
  std::vector<bool> is_corner(n, false);
  constexpr int kBackground = 5;
  for (std::size_t i = 1; i + 1 < n; ++i)
  {
    if (turn[i] <= thr)
      continue;
    double bg = 0.0;
    int cnt = 0;
    for (int d = -kBackground; d <= kBackground; ++d)
    {
      if (d == 0)
        continue;
      const long j = static_cast<long>(i) + d;
      if (j < 1 || j >= static_cast<long>(n) - 1)
        continue;
      bg += turn[static_cast<std::size_t>(j)];
      ++cnt;
    }
    bg = cnt > 0 ? bg / cnt : 0.0;
    // at least 2.5x the local curvature (arcs have a flat background)
    if (turn[i] > 2.5 * (bg + 1e-4))
      is_corner[i] = true;
  }

  // Two corners separated by a very short segment cannot both be blended: the
  // arcs and their orientation references would overlap. Skip the later one.
  std::vector<bool> usable = is_corner;
  for (std::size_t i = 2; i + 1 < n; ++i)
    if (usable[i] && usable[i - 1])
      usable[i] = false;

  // ---- pass 2: build the output ------------------------------------------
  std::ostringstream notes;
  std::vector<std::pair<std::size_t, std::size_t>> arc_ranges;  // per blended corner
  out.frames.reserve(n + 16);
  out.frames.push_back(frames.front());

  for (std::size_t i = 1; i + 1 < n; ++i)
  {
    if (!is_corner[i])
    {
      out.frames.push_back(frames[i]);
      continue;
    }

    if (!usable[i])
    {
      out.frames.push_back(frames[i]);
      CornerInfo info;
      info.vertex = i;
      info.position = polyline[i];
      info.turn_deg = turn[i] * 180.0 / kPi;
      info.pivot = false;
      out.corners.push_back(info);
      notes << "\n  corner #" << i << " " << fmt(polyline[i]) << ": "
            << info.turn_deg << " deg skipped (too close to the previous corner)";
      continue;
    }

    const double phi = turn[i];
    const double half = 0.5 * phi;
    const double cos_half = std::cos(half);
    const double tan_half = std::tan(half);
    // arc of radius r deviates from the vertex by r * (sec(half) - 1)
    const double sec_minus_1 = 1.0 / std::max(cos_half, 1e-6) - 1.0;
    const double la = (polyline[i] - polyline[i - 1]).norm();
    const double lb = (polyline[i + 1] - polyline[i]).norm();
    const double max_t = 0.45 * std::min(la, lb);

    double r = sec_minus_1 > 1e-9 ? dev_budget / sec_minus_1 : 0.0;
    if (tan_half > 1e-9)
      r = std::min(r, max_t / tan_half);
    else
      r = 0.0;

    CornerInfo info;
    info.vertex = i;
    info.position = polyline[i];
    info.turn_deg = phi * 180.0 / kPi;

    const Eigen::Quaterniond q_in = quatOf(frames[i - 1]);
    const Eigen::Quaterniond q_out = quatOf(frames[i + 1]);

    if (r < policy.min_radius)
    {
      // Not enough room to leave the seam: rotate in place. The TCP holds the
      // corner point while the wrist turns; the arm never comes to rest.
      info.pivot = true;
      info.radius = 0.0;
      info.deviation = 0.0;
      ++out.pivot_count;
      notes << "\n  corner #" << i << " " << fmt(polyline[i]) << ": "
            << info.turn_deg << " deg has no room for a " << (dev_budget * 1000.0)
            << " mm blend -> pivot (TCP holds, wrist rotates)";
      if (policy.pivot_fallback)
      {
        const int steps = std::max(
            3, static_cast<int>(std::ceil(phi / std::max(policy.pivot_step_rad, 1e-3))));
        for (int s = 1; s <= steps; ++s)
        {
          Eigen::Isometry3d f = frames[i];
          f.linear() = q_in.slerp(static_cast<double>(s) / steps, q_out).toRotationMatrix();
          out.frames.push_back(f);
        }
      }
      else
      {
        out.frames.push_back(frames[i]);
      }
      out.corners.push_back(info);
      continue;
    }

    // ---- tangent-continuous arc -------------------------------------------
    const Eigen::Vector3d u = (polyline[i] - polyline[i - 1]).normalized();
    const Eigen::Vector3d v = (polyline[i + 1] - polyline[i]).normalized();
    const double t = r * tan_half;
    const Eigen::Vector3d t1 = polyline[i] - u * t;
    const Eigen::Vector3d t2 = polyline[i] + v * t;
    const Eigen::Vector3d bis = (v - u).normalized();
    const Eigen::Vector3d center = polyline[i] + bis * (r / cos_half);
    Eigen::Vector3d axis = u.cross(v);
    axis = axis.norm() > 1e-9 ? axis.normalized() : Eigen::Vector3d::UnitZ();

    const double arc_len = r * phi;
    const int steps = std::max(
        2, static_cast<int>(std::ceil(arc_len / std::max(policy.sample_step, 1e-4))));
    const std::size_t out_begin = out.frames.size();
    for (int s = 0; s <= steps; ++s)
    {
      const double a = static_cast<double>(s) / steps;
      const Eigen::AngleAxisd rot(a * phi, axis);
      Eigen::Isometry3d f = frames[i];
      f.translation() = center + rot * (t1 - center);
      f.linear() = q_in.slerp(a, q_out).toRotationMatrix();
      out.frames.push_back(f);
    }
    (void)t2;

    info.radius = r;
    info.deviation = r * sec_minus_1;
    info.pivot = false;
    out.corners.push_back(info);
    arc_ranges.push_back({ out_begin, out.frames.size() - 1 });
    notes << "\n  corner #" << i << " " << fmt(polyline[i]) << ": " << info.turn_deg
          << " deg, r=" << (r * 1000.0) << " mm, off-seam " << (info.deviation * 1000.0)
          << " mm";
  }

  out.frames.push_back(frames.back());

  // ---- pass 3: spread the orientation change over a window ----------------
  // The arc itself is only a few millimetres long (it is bounded by the 0.5 mm
  // deviation budget), so slerping across the arc alone still leaves a large
  // per-step rotation. Re-distribute the rotation over +/- orient_window along
  // the resulting path: position stays on the seam, the wrist turns gradually.
  if (policy.orient_window > 1e-6 && !arc_ranges.empty())
  {
    for (const auto& range : arc_ranges)
    {
      std::size_t l = range.first;
      std::size_t r2 = range.second;
      const double W = policy.orient_window;
      double acc = 0.0;
      while (l > 0)
      {
        acc += (out.frames[l].translation() - out.frames[l - 1].translation()).norm();
        if (acc > W)
          break;
        --l;
      }
      acc = 0.0;
      while (r2 + 1 < out.frames.size())
      {
        acc += (out.frames[r2 + 1].translation() - out.frames[r2].translation()).norm();
        if (acc > W)
          break;
        ++r2;
      }
      if (r2 <= l + 1)
        continue;

      const Eigen::Quaterniond q0(quatOf(out.frames[l]));
      const Eigen::Quaterniond q1(quatOf(out.frames[r2]));
      std::vector<double> cum(r2 - l + 1, 0.0);
      for (std::size_t i = l + 1; i <= r2; ++i)
        cum[i - l] = cum[i - l - 1] +
                     (out.frames[i].translation() - out.frames[i - 1].translation()).norm();
      const double total = cum.back();
      if (total < 1e-9)
        continue;
      for (std::size_t i = l + 1; i < r2; ++i)
      {
        const double a = cum[i - l] / total;
        out.frames[i].linear() = q0.slerp(a, q1).toRotationMatrix();
      }
    }
  }

  std::ostringstream summary;
  if (!out.corners.empty())
  {
    summary << "corner plan: " << out.corners.size() << " corner(s)";
    if (out.pivot_count)
      summary << ", " << out.pivot_count << " need pivot";
    summary << notes.str();
  }
  out.summary = summary.str();
  return out;
}

}  // namespace rosim::core
