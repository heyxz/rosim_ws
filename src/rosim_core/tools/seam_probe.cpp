/// Seam-merging probe. Loads a STEP/BREP, discovers endpoint-connected chains
/// and runs them through rosim::core::buildSeam, printing what merged and what
/// was rejected (and why). Used to validate the merge rules against real parts
/// before wiring them into the planner.
///
/// usage: seam_probe <model.step> [--pose x,y,z] [--tol 0.0005] [--edges 4,17,22]

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "rosim_core/occ_geometry_service.h"
#include "rosim_core/seam_builder.h"

using rosim::core::EdgeData;
using rosim::core::OccGeometryService;
using rosim::core::buildSeam;

namespace
{

struct EndpointIndex
{
  // endpoint -> (edge index, which end)
  std::vector<std::pair<Eigen::Vector3d, int>> pts;
  std::vector<uint32_t> owner;

  int find(const Eigen::Vector3d& p, double tol) const
  {
    for (std::size_t i = 0; i < pts.size(); ++i)
      if ((pts[i].first - p).norm() <= tol)
        return static_cast<int>(i);
    return -1;
  }
};

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: seam_probe <model.step> [--pose x,y,z] [--tol m] [--edges 4,17,22]\n";
    return 1;
  }
  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  double tol = 5e-4;
  std::string edge_list;
  for (int i = 2; i + 1 < argc; i += 2)
  {
    const std::string k = argv[i];
    if (k == "--pose")
    {
      std::sscanf(argv[i + 1], "%lf,%lf,%lf", &pose.x(), &pose.y(), &pose.z());
    }
    else if (k == "--tol")
    {
      tol = std::stod(argv[i + 1]);
    }
    else if (k == "--edges")
    {
      edge_list = argv[i + 1];
    }
  }

  std::shared_ptr<rosim::core::WorkpieceModel> model_;
  try
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = pose;
    model_ = OccGeometryService::importModel(argv[1], T, "probe");
  }
  catch (const std::exception& e)
  {
    std::cerr << "import failed: " << e.what() << "\n";
    return 1;
  }
  std::cout << "edges: " << model_->edges.size() << "  tol=" << tol * 1000.0 << " mm\n";

  // ---- explicit edge list -------------------------------------------------
  if (!edge_list.empty())
  {
    std::vector<uint32_t> idx;
    std::string cur;
    for (char c : edge_list + ",")
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
    const auto r = buildSeam(model_->edges, idx, tol);
    std::cout << (r.ok ? "OK  " : "FAIL") << "  " << r.message << "\n";
    if (r.ok)
    {
      std::cout << "  length=" << r.length << " m  closed=" << r.closed
                << "  points=" << r.polyline.size() << "\n  parts:";
      for (const auto& p : r.parts)
        std::cout << " #" << p.edge_index << (p.reversed ? "(rev)" : "");
      std::cout << "\n";
    }
    return r.ok ? 0 : 2;
  }

  // ---- topology census ----------------------------------------------------
  std::vector<std::pair<Eigen::Vector3d, int>> nodes;  // position, degree
  auto node_id = [&](const Eigen::Vector3d& p) {
    for (std::size_t i = 0; i < nodes.size(); ++i)
      if ((nodes[i].first - p).norm() <= tol)
      {
        nodes[i].second++;
        return static_cast<int>(i);
      }
    nodes.push_back({ p, 1 });
    return static_cast<int>(nodes.size()) - 1;
  };
  std::vector<std::pair<int, int>> ends;
  for (const auto& e : model_->edges)
    ends.emplace_back(node_id(e.polyline.front()), node_id(e.polyline.back()));

  std::map<int, int> hist;
  for (const auto& n : nodes)
    hist[n.second]++;
  std::cout << "\n--- endpoint degree histogram (all " << model_->edges.size()
            << " edges) ---\n";
  for (const auto& [deg, cnt] : hist)
    std::cout << "  degree " << deg << ": " << cnt << " nodes\n";

  // ---- every adjacent pair ------------------------------------------------
  int pair_ok = 0, pair_bad = 0;
  std::map<std::string, int> reasons;
  std::vector<std::pair<uint32_t, uint32_t>> good_pairs;
  for (std::size_t i = 0; i < model_->edges.size(); ++i)
    for (std::size_t j = i + 1; j < model_->edges.size(); ++j)
    {
      const auto& a = model_->edges[i];
      const auto& b = model_->edges[j];
      const bool touch = (a.polyline.front() - b.polyline.front()).norm() <= tol ||
                         (a.polyline.front() - b.polyline.back()).norm() <= tol ||
                         (a.polyline.back() - b.polyline.front()).norm() <= tol ||
                         (a.polyline.back() - b.polyline.back()).norm() <= tol;
      if (!touch)
        continue;
      const auto r = buildSeam(model_->edges, { a.index, b.index }, tol);
      if (r.ok)
      {
        ++pair_ok;
        good_pairs.emplace_back(a.index, b.index);
      }
      else
      {
        ++pair_bad;
        reasons[r.message.substr(0, 40)]++;
      }
    }
  std::cout << "\n--- adjacent edge pairs ---\n  mergeable: " << pair_ok
            << "   rejected: " << pair_bad << "\n";
  for (std::size_t k = 0; k < std::min<std::size_t>(good_pairs.size(), 8); ++k)
  {
    const auto r = buildSeam(model_->edges, { good_pairs[k].first, good_pairs[k].second }, tol);
    std::cout << "  #" << good_pairs[k].first << " + #" << good_pairs[k].second
              << " -> len " << r.length << " m, points " << r.polyline.size() << "\n";
  }
  for (const auto& [msg, n] : reasons)
    std::cout << "  reject reason (" << n << "x): " << msg << "\n";

  // ---- 3-edge paths (a chain through one intermediate node) --------------
  std::map<uint32_t, std::vector<uint32_t>> adj3;
  for (std::size_t i = 0; i < model_->edges.size(); ++i)
    for (std::size_t j = i + 1; j < model_->edges.size(); ++j)
    {
      const auto& a = model_->edges[i];
      const auto& b = model_->edges[j];
      const bool touch = (a.polyline.front() - b.polyline.front()).norm() <= tol ||
                         (a.polyline.front() - b.polyline.back()).norm() <= tol ||
                         (a.polyline.back() - b.polyline.front()).norm() <= tol ||
                         (a.polyline.back() - b.polyline.back()).norm() <= tol;
      if (touch)
      {
        adj3[a.index].push_back(b.index);
        adj3[b.index].push_back(a.index);
      }
    }

  int chains = 0, merged = 0, loops = 0, rejected = 0;
  std::map<std::string, int> chain_reasons;
  uint32_t shown = 0;
  std::cout << "\n--- 3-edge paths ---\n";
  for (const auto& e : model_->edges)
    for (const uint32_t b : adj3[e.index])
      for (const uint32_t c : adj3[b])
      {
        if (c == e.index || c <= e.index)
          continue;  // each unordered triple once
        ++chains;
        const auto r = buildSeam(model_->edges, { e.index, b, c }, tol);
        if (r.ok)
        {
          ++merged;
          if (r.closed)
            ++loops;
          if (shown < 5)
          {
            std::cout << "  #" << e.index << " #" << b << " #" << c
                      << " -> OK len=" << r.length << " m closed=" << r.closed
                      << " parts";
            for (const auto& pt : r.parts)
              std::cout << " #" << pt.edge_index << (pt.reversed ? "(rev)" : "");
            std::cout << "\n";
            ++shown;
          }
        }
        else
        {
          ++rejected;
          chain_reasons[r.message.substr(0, 46)]++;
        }
      }
  std::cout << "  mergeable: " << merged << "   rejected: " << rejected << "\n";
  for (const auto& [msg, n] : chain_reasons)
    std::cout << "  reject reason (" << n << "x): " << msg << "\n";

  std::cout << "\nsummary: " << chains << " triples, " << merged << " merged ("
            << loops << " loops), " << rejected << " rejected\n";

  // deliberate negative case: two edges that do not touch
  for (std::size_t i = 0; i < model_->edges.size(); ++i)
    for (std::size_t j = i + 1; j < model_->edges.size(); ++j)
    {
      const auto& a = model_->edges[i];
      const auto& b = model_->edges[j];
      const bool touch = (a.polyline.front() - b.polyline.front()).norm() <= tol ||
                         (a.polyline.front() - b.polyline.back()).norm() <= tol ||
                         (a.polyline.back() - b.polyline.front()).norm() <= tol ||
                         (a.polyline.back() - b.polyline.back()).norm() <= tol;
      if (!touch)
      {
        const auto r = buildSeam(model_->edges, { a.index, b.index }, tol);
        std::cout << "\nnegative case #" << a.index << " + #" << b.index << ": "
                  << (r.ok ? "UNEXPECTED OK" : "rejected as expected (" + r.message + ")") << "\n";
        i = model_->edges.size();  // stop after the first
        break;
      }
    }

  std::cout << "\nsummary: " << chains << " chains, " << merged << " merged ("
            << loops << " loops), " << rejected << " rejected\n";
  return 0;
}
