#pragma once

// Minimal fork-join helper for the IK / feasibility scans.
//
// Why not OpenMP or TBB: rosim_planning is a plain ament library and the only
// requirement here is "run N independent, CPU-bound evaluations concurrently".
// std::thread is enough, has no extra dependency, and keeps the build flags
// untouched.
//
// Nesting: a parallelFor() called from inside a worker degrades to a serial
// loop. toolpathFeasible() (parallel over waypoints) calls solveIkMultiSeed()
// (parallel over seeds); without this guard the thread count would multiply.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <functional>
#include <thread>
#include <vector>

namespace rosim::planning
{

/// Thread-local nesting depth marker shared by all parallelFor() calls.
inline bool& inParallelRegion()
{
  static thread_local bool in_region = false;
  return in_region;
}

/// Upper bound on worker threads: leave headroom for the executor, the
/// controller manager and the render thread instead of saturating every core.
inline unsigned defaultParallelThreads()
{
  const unsigned hw = std::thread::hardware_concurrency();
  if (hw == 0)
    return 1;
  return std::min(hw > 4 ? hw - 2 : hw, 12u);  // 16 threads -> 12
}

/// Run body(i) for every i in [0, n) using up to `threads` threads.
/// The calling thread participates, so threads == 1 is a plain serial loop.
inline void parallelFor(std::size_t n, unsigned threads, const std::function<void(std::size_t)>& body)
{
  if (n == 0)
    return;
  if (n == 1 || threads <= 1 || inParallelRegion())
  {
    for (std::size_t i = 0; i < n; ++i)
      body(i);
    return;
  }

  const unsigned nthreads = static_cast<unsigned>(std::min<std::size_t>(n, threads));
  std::atomic<std::size_t> next{ 0 };
  std::vector<std::thread> workers;
  workers.reserve(nthreads - 1);

  inParallelRegion() = true;
  for (unsigned t = 0; t + 1 < nthreads; ++t)
  {
    workers.emplace_back([&]() {
      inParallelRegion() = true;  // nested calls from here on stay serial
      for (;;)
      {
        const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= n)
          break;
        body(i);
      }
    });
  }
  for (;;)
  {
    const std::size_t i = next.fetch_add(1, std::memory_order_relaxed);
    if (i >= n)
      break;
    body(i);
  }
  for (auto& w : workers)
    w.join();
  inParallelRegion() = false;
}

}  // namespace rosim::planning
