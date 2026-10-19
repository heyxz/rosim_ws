/// Corner-blending probe: load a workpiece, merge the given edges, generate the
/// seam frames, then compare the per-step orientation change before and after
/// blendCorners(). Used to verify that corners are actually smoothed (and that
/// the off-seam deviation stays inside the budget) without running the planner.
///
/// usage: corner_probe <model.step> --edges 1,17 [--pose x,y,z] [--dev mm]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include "rosim_core/corner_planner.h"
#include "rosim_core/edge_frame_generator.h"
#include "rosim_core/occ_geometry_service.h"
#include "rosim_core/seam_builder.h"

using namespace rosim::core;

namespace
{
double maxStepDeg(const std::vector<Eigen::Isometry3d>& f)
{
  double worst = 0.0;
  for (std::size_t i = 1; i < f.size(); ++i)
  {
    const Eigen::Quaterniond a(f[i - 1].linear()), b(f[i].linear());
    const double ang = 2.0 * std::acos(std::min(1.0, std::fabs(a.dot(b))));
    worst = std::max(worst, ang * 180.0 / M_PI);
  }
  return worst;
}
double maxStepMm(const std::vector<Eigen::Isometry3d>& f)
{
  double worst = 0.0;
  for (std::size_t i = 1; i < f.size(); ++i)
    worst = std::max(worst, (f[i].translation() - f[i - 1].translation()).norm() * 1000.0);
  return worst;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: corner_probe <model.step> --edges 1,17 [--pose x,y,z] [--dev mm]\n";
    return 1;
  }
  Eigen::Vector3d pose(0, 0, 0);
  std::string edges_arg;
  double dev_mm = 0.5;
  double window_mm = 20.0;
  for (int i = 2; i + 1 < argc; i += 2)
  {
    const std::string k = argv[i];
    if (k == "--pose")
      std::sscanf(argv[i + 1], "%lf,%lf,%lf", &pose.x(), &pose.y(), &pose.z());
    else if (k == "--edges")
      edges_arg = argv[i + 1];
    else if (k == "--dev")
      dev_mm = std::stod(argv[i + 1]);
    else if (k == "--window")
      window_mm = std::stod(argv[i + 1]);
  }
  if (edges_arg.empty())
  {
    std::cerr << "--edges is required\n";
    return 1;
  }

  std::vector<uint32_t> idx;
  std::string cur;
  for (char c : edges_arg + ",")
  {
    if (c == ',')
    {
      if (!cur.empty())
        idx.push_back(static_cast<uint32_t>(std::stoul(cur)));
      cur.clear();
    }
    else
      cur.push_back(c);
  }

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = pose;
  auto model = OccGeometryService::importModel(argv[1], T, "probe");
  const auto seam = buildSeam(model->edges, idx);
  if (!seam.ok)
  {
    std::cout << "merge failed: " << seam.message << "\n";
    return 2;
  }

  EdgeFrameGenerator::Config cfg;
  cfg.orientation_mode = EdgeFrameGenerator::ORIENTATION_TANGENT;
  cfg.tangent_tilt = 0.6;
  const auto frames = EdgeFrameGenerator::generate(seam.polyline, cfg);

  CornerPolicy policy;
  policy.max_deviation = dev_mm * 1e-3;
  policy.orient_window = window_mm * 1e-3;
  const auto plan = blendCorners(seam.polyline, frames, policy);

  std::printf("seam: %.3f m, %zu points -> %zu after blending\n", seam.length,
              frames.size(), plan.frames.size());
  std::printf("max per-step rotation: %.2f deg -> %.2f deg\n", maxStepDeg(frames),
              maxStepDeg(plan.frames));
  std::printf("max per-step translation: %.1f mm -> %.1f mm\n", maxStepMm(frames),
              maxStepMm(plan.frames));
  std::printf("corners: %zu (pivot %zu)\n", plan.corners.size(), plan.pivot_count);
  for (const auto& c : plan.corners)
    std::printf("  #%zu (%.3f,%.3f,%.3f) turn %.1f deg  r=%.3f mm  off-seam %.3f mm%s\n",
                c.vertex, c.position.x(), c.position.y(), c.position.z(), c.turn_deg,
                c.radius * 1000.0, c.deviation * 1000.0, c.pivot ? "  [pivot]" : "");
  if (!plan.summary.empty())
    std::printf("%s\n", plan.summary.c_str());
  return 0;
}
