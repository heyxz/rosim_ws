// Seam merging: turn a set of endpoint-connected edges into one continuous
// polyline that the edge-following pipeline can plan as a single weld.
//
// Pure geometry: no ROS, no planning scene. The caller owns the EdgeData list
// (planning node cache or a probe tool).

#include "rosim_core/seam_builder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <queue>
#include <sstream>

namespace rosim::core
{

bool pointsCoincide(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol)
{
  return (a - b).norm() <= tol;
}

namespace
{

std::string fmtPoint(const Eigen::Vector3d& p)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(3);
  os << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")";
  return os.str();
}

using CellKey = std::array<long long, 3>;

CellKey cellOf(const Eigen::Vector3d& p, double tol)
{
  return { static_cast<long long>(std::floor(p.x() / tol)),
           static_cast<long long>(std::floor(p.y() / tol)),
           static_cast<long long>(std::floor(p.z() / tol)) };
}

}  // namespace

SeamBuildResult buildSeam(const std::vector<EdgeData>& edges,
                          const std::vector<uint32_t>& indices,
                          double endpoint_tolerance)
{
  SeamBuildResult out;
  if (indices.empty())
  {
    out.message = "no edges selected";
    return out;
  }
  const double tol = endpoint_tolerance > 0.0 ? endpoint_tolerance : 5e-4;

  // De-duplicate while keeping the caller's order (it defines nothing about the
  // final travel order, but stable input helps error messages).
  std::vector<uint32_t> wanted;
  for (const uint32_t i : indices)
    if (std::find(wanted.begin(), wanted.end(), i) == wanted.end())
      wanted.push_back(i);

  std::vector<const EdgeData*> sel;
  sel.reserve(wanted.size());
  for (const uint32_t i : wanted)
  {
    const auto it = std::find_if(edges.begin(), edges.end(),
                                 [i](const EdgeData& e) { return e.index == i; });
    if (it == edges.end())
    {
      out.message = "edge #" + std::to_string(i) + " not found in this model";
      return out;
    }
    if (it->polyline.size() < 2)
    {
      out.message = "edge #" + std::to_string(i) + " has fewer than 2 points";
      return out;
    }
    sel.push_back(&*it);
  }
  const std::size_t n = sel.size();

  // Single edge: a valid seam on its own (including a full circle).
  if (n == 1)
  {
    out.ok = true;
    out.parts.push_back({ sel[0]->index, false });
    out.polyline = sel[0]->polyline;
    out.length = sel[0]->length;
    out.closed = sel[0]->closed;
    out.message = "single edge seam";
    return out;
  }

  for (const EdgeData* e : sel)
  {
    if (e->closed)
    {
      out.message = "closed edge #" + std::to_string(e->index) +
                    " cannot be merged with other edges";
      return out;
    }
  }

  // ---- endpoint clustering -------------------------------------------------
  std::map<CellKey, std::vector<int>> grid;
  std::vector<Eigen::Vector3d> node_pos;
  std::vector<std::array<int, 2>> edge_nodes(n);

  auto node_for = [&](const Eigen::Vector3d& p) {
    const CellKey k = cellOf(p, tol);
    for (long long dx = -1; dx <= 1; ++dx)
      for (long long dy = -1; dy <= 1; ++dy)
        for (long long dz = -1; dz <= 1; ++dz)
        {
          const auto it = grid.find({ k[0] + dx, k[1] + dy, k[2] + dz });
          if (it == grid.end())
            continue;
          for (const int node : it->second)
            if (pointsCoincide(node_pos[node], p, tol))
              return node;
        }
    const int node = static_cast<int>(node_pos.size());
    node_pos.push_back(p);
    grid[k].push_back(node);
    return node;
  };

  for (std::size_t i = 0; i < n; ++i)
  {
    edge_nodes[i] = { node_for(sel[i]->polyline.front()), node_for(sel[i]->polyline.back()) };
    if (edge_nodes[i][0] == edge_nodes[i][1])
    {
      out.message = "edge #" + std::to_string(sel[i]->index) +
                    " starts and ends at the same point (tolerance too large?)";
      return out;
    }
  }

  // ---- topology checks -----------------------------------------------------
  std::vector<std::vector<std::pair<int, int>>> incident(node_pos.size());  // (edge, end)
  for (std::size_t i = 0; i < n; ++i)
    for (int k = 0; k < 2; ++k)
      incident[edge_nodes[i][k]].push_back({ static_cast<int>(i), k });

  for (std::size_t v = 0; v < incident.size(); ++v)
  {
    if (incident[v].size() > 2)
    {
      out.message = "branch point " + fmtPoint(node_pos[v]) + " joins " +
                    std::to_string(incident[v].size()) + " edges; a seam must be a simple chain";
      return out;
    }
  }

  // connectivity
  std::vector<int> comp(n, -1);
  int comps = 0;
  for (std::size_t s = 0; s < n; ++s)
  {
    if (comp[s] >= 0)
      continue;
    std::queue<int> q;
    q.push(static_cast<int>(s));
    comp[s] = comps;
    while (!q.empty())
    {
      const int e = q.front();
      q.pop();
      for (int k = 0; k < 2; ++k)
        for (const auto& [e2, k2] : incident[edge_nodes[e][k]])
        {
          (void)k2;
          if (comp[e2] < 0)
          {
            comp[e2] = comps;
            q.push(e2);
          }
        }
    }
    ++comps;
  }
  if (comps > 1)
  {
    out.message = "selected edges form " + std::to_string(comps) +
                  " disconnected groups; select edges that share endpoints";
    return out;
  }

  std::vector<int> ends;
  for (std::size_t v = 0; v < incident.size(); ++v)
    if (incident[v].size() == 1)
      ends.push_back(static_cast<int>(v));
  const bool loop = ends.empty();
  if (!loop && ends.size() != 2)
  {
    out.message = "unexpected topology: " + std::to_string(ends.size()) + " open ends";
    return out;
  }

  // ---- walk the chain ------------------------------------------------------
  std::vector<bool> used(n, false);
  std::vector<SeamPart> parts;
  std::vector<Eigen::Vector3d> poly;

  const int start_node = loop ? edge_nodes[0][0] : ends[0];
  int cur = start_node;
  for (std::size_t guard = 0; guard <= n; ++guard)
  {
    int pick = -1;
    int pick_end = -1;
    for (const auto& [e, k] : incident[cur])
      if (!used[e])
      {
        pick = e;
        pick_end = k;
        break;
      }
    if (pick < 0)
      break;

    const auto& src = sel[pick]->polyline;
    const bool reversed = (pick_end == 1);  // that edge ends here -> walk backwards
    parts.push_back({ sel[pick]->index, reversed });

    if (!reversed)
    {
      for (const auto& p : src)
        if (poly.empty() || !pointsCoincide(poly.back(), p, tol))
          poly.push_back(p);
    }
    else
    {
      for (auto it = src.rbegin(); it != src.rend(); ++it)
        if (poly.empty() || !pointsCoincide(poly.back(), *it, tol))
          poly.push_back(*it);
    }

    used[pick] = true;
    cur = edge_nodes[pick][1 - pick_end];
    if (loop && cur == start_node)
      break;
  }

  if (std::find(used.begin(), used.end(), false) != used.end())
  {
    out.message = "internal error: chain walk left unused edges";
    return out;
  }

  double length = 0.0;
  for (std::size_t i = 1; i < poly.size(); ++i)
    length += (poly[i] - poly[i - 1]).norm();
  if (loop && poly.size() > 1)
    length += (poly.front() - poly.back()).norm();  // closing gap

  out.ok = true;
  out.polyline = std::move(poly);
  out.parts = std::move(parts);
  out.length = length;
  out.closed = loop;
  out.message = "merged " + std::to_string(n) + " edges";
  return out;
}

}  // namespace rosim::core
