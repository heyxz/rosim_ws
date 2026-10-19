// Edge-following pipeline. Turns an edge polyline into TCP frames, allocates a
// rail station, finds a collision-free approach (multi-seed IK + OMPL), walks
// the seam with Cartesian interpolation, then retreats to free space and
// time-parameterizes the merged trajectory. Pure computation: no ROS topics —
// PlanningNode owns the action/service interfaces and execution.

#include "rosim_planning/edge_follower.h"

#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

#include <unordered_map>

#include "rosim_core/corner_planner.h"
#include "rosim_planning/parallel.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>

#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "rosim_interfaces/action/follow_edge.hpp"

namespace rosim::planning
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("rosim.edge_follower");

using ErrorCode = rosim_interfaces::action::FollowEdge_Result;

/// Deduplicate consecutive identical states when merging trajectory segments.
bool sameGroupState(const moveit::core::RobotState& a, const moveit::core::RobotState& b,
                    const moveit::core::JointModelGroup* group)
{
  std::vector<double> ja, jb;
  a.copyJointGroupPositions(group, ja);
  b.copyJointGroupPositions(group, jb);
  for (size_t i = 0; i < ja.size(); ++i)
    if (std::abs(ja[i] - jb[i]) > 1e-9)
      return false;
  return true;
}
}  // namespace

EdgeFollower::EdgeFollower(rclcpp::Node::SharedPtr node, moveit::core::RobotModelPtr model,
                           MotionPlannerPtr planner)
  : node_(std::move(node)), model_(std::move(model)), planner_(std::move(planner))
{
  // constant tool0 -> tool_tip offset from the model (fixed joint)
  moveit::core::RobotState scratch(model_);
  scratch.setToDefaultValues();
  scratch.updateLinkTransforms();
  const Eigen::Isometry3d& tf0 = scratch.getGlobalLinkTransform(tip_link_);
  const Eigen::Isometry3d& tf_tip = scratch.getGlobalLinkTransform("tool_tip");
  tool0_to_tip_ = tf0.inverse() * tf_tip;

  config_.consistency_limits =
      std::vector<double>(model_->getJointModelGroup(arm_group_)->getActiveJointModels().size(),
                          0.08);
}

bool EdgeFollower::hasRail() const
{
  const moveit::core::JointModel* rail = model_->getJointModel(rail_joint_);
  return rail && rail->getType() == moveit::core::JointModel::PRISMATIC &&
         rail->getVariableCount() == 1;
}

unsigned EdgeFollower::threads() const
{
  if (config_.parallel_threads > 0)
    return static_cast<unsigned>(config_.parallel_threads);
  return defaultParallelThreads();
}

void EdgeFollower::setRailStation(moveit::core::RobotState& state, double station) const
{
  if (!hasRail())
    return;
  const auto& bounds = model_->getJointModel(rail_joint_)->getVariableBounds();
  if (!bounds.empty() && bounds.front().position_bounded_)
    station = std::min(std::max(station, bounds.front().min_position_),
                       bounds.front().max_position_);
  state.setVariablePosition(rail_joint_, station);
  state.update();
}

double EdgeFollower::railStationFor(const Eigen::Vector3d& target,
                                    const moveit::core::RobotState& state) const
{
  if (!hasRail())
    return 0.0;

  const auto& bounds = model_->getJointModel(rail_joint_)->getVariableBounds();
  double lo = -1e9, hi = 1e9;
  if (!bounds.empty() && bounds.front().position_bounded_)
  {
    lo = bounds.front().min_position_;
    hi = bounds.front().max_position_;
  }

  // The carriage only moves along X, so any Y/Z offset of the seam consumes
  // part of the arm's comfortable reach. Pick the station that leaves the
  // wrist centre ~0.62 m from the rail axis (>= 0.20 m of forward lever).
  const double base_z = state.getGlobalLinkTransform("base_link").translation().z();
  const double dy = target.y();
  const double dz = target.z() - base_z;
  constexpr double kComfortReach = 0.62;
  const double forward = std::sqrt(std::max(
      0.20 * 0.20, kComfortReach * kComfortReach - dy * dy - dz * dz));
  return std::min(std::max(target.x() - forward, lo), hi);
}

bool EdgeFollower::toolpathFeasible(const std::vector<Eigen::Isometry3d>& poses,
                                    const moveit::core::RobotState& seed,
                                    const planning_scene::PlanningSceneConstPtr& scene)
{
  if (poses.empty())
    return false;
  // Each waypoint is independent, so the vote fans out across cores. The
  // reduced 12-seed budget is unchanged: same seeds, same verdict, just
  // evaluated concurrently. First failure cancels the rest.
  std::atomic<bool> failed{ false };
  parallelFor(poses.size(), threads(), [&](std::size_t i) {
    if (failed.load(std::memory_order_relaxed))
      return;
    moveit::core::RobotStatePtr sol;
    if (!solveIkMultiSeed(poses[i], seed, scene, sol, 12))
      failed.store(true, std::memory_order_relaxed);
  });
  return !failed.load();
}

bool EdgeFollower::solveIkMultiSeed(const Eigen::Isometry3d& tool0_pose,
                                    const moveit::core::RobotState& seed,
                                    const planning_scene::PlanningSceneConstPtr& scene,
                                    moveit::core::RobotStatePtr& solution,
                                    int max_attempts)
{
  // IK solves the 6-DOF arm; the rail is held at the seed's station.
  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(arm_group_);
  const bool rail = hasRail();
  const double seed_rail = rail ? seed.getVariablePosition(rail_joint_) : 0.0;

  auto validity = [&scene](moveit::core::RobotState* state,
                           const moveit::core::JointModelGroup* /*g*/,
                           const double* /*values*/) {
    state->updateCollisionBodyTransforms();
    return !scene->isStateColliding(*state);
  };

  // Cheap path: one consistency-limited KDL solve straight from the live seed.
  // It targets the same thing the walk does (stay on the seed's branch) for a
  // single solver call, so it is tried before the ladder when configured.
  auto try_consistency = [&]() -> moveit::core::RobotStatePtr {
    moveit::core::RobotState candidate(seed);
    const std::vector<double> loose(group->getActiveJointModels().size(), 0.8);
    if (!candidate.setFromIK(group, tool0_pose, tip_link_, loose, config_.ik_timeout * 3.0,
                             validity))
      return nullptr;
    candidate.updateCollisionBodyTransforms();
    if (scene->isStateColliding(candidate))
      return nullptr;
    return std::make_shared<moveit::core::RobotState>(candidate);
  };

  if (config_.cheap_solve_first)
  {
    if (auto s = try_consistency())
    {
      solution = s;
      RCLCPP_INFO(LOGGER, "IK: consistency-limited solve from current state");
      return true;
    }
  }

  // Fallback path: walk the seed state's TCP to the target through
  // interpolated intermediate poses (continuation method). Adaptive step: on
  // IK failure halve the step and retry, on success grow it again — gets
  // through intermediate poses KDL cannot solve in one jump. Per-step
  // consistency limits keep the solution on the seed's branch (no elbow wrap,
  // which an unconstrained long-timeout solve allows). Budgeted per execute().
  const bool may_walk = config_.walk_attempts <= 0 || walk_budget_.fetch_sub(1) > 0;
  if (may_walk)
  {
    const Eigen::Isometry3d from = seed.getGlobalLinkTransform(tip_link_);
    moveit::core::RobotState walk(seed);
    const Eigen::Quaterniond q0(from.linear()), q1(tool0_pose.linear());
    const std::vector<double> step_consistency(group->getActiveJointModels().size(), 0.35);
    double t = 0.0, dt = 0.1;
    int failures = 0, solves = 0;
    bool ok = true;
    while (t < 1.0 && ok)
    {
      const double next = std::min(1.0, t + dt);
      Eigen::Isometry3d mid = Eigen::Isometry3d::Identity();
      mid.translation() = (1.0 - next) * from.translation() + next * tool0_pose.translation();
      mid.linear() = q0.slerp(next, q1).toRotationMatrix();
      // One IK budget per step, not four: KDL converges in well under a
      // millisecond here, so a longer timeout only makes *failing* steps (the
      // common case near the reach boundary) burn the clock before the walk
      // halves the step and retries. The retry ladder already provides the
      // robustness the extra timeout was meant to give.
      ok = walk.setFromIK(group, mid, tip_link_, step_consistency, config_.ik_timeout,
                          validity);
      ++solves;
      if (ok)
      {
        t = next;
        dt = std::min(dt * 1.5, 0.2);
      }
      else
      {
        dt *= 0.5;
        ++failures;
        if (dt < 0.005 || failures > 40 || solves > 400)
          break;
        ok = true;  // retry from the same state with a smaller step
      }
    }
    if (!ok || t < 1.0)
      RCLCPP_INFO(LOGGER,
                  "IK: adaptive walk stalled at t=%.2f after %d solves/%d failures (tip "
                  "(%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f))",
                  t, solves, failures, from.translation().x(), from.translation().y(),
                  from.translation().z(), tool0_pose.translation().x(),
                  tool0_pose.translation().y(), tool0_pose.translation().z());
    if (ok && t >= 1.0)
    {
      walk.updateCollisionBodyTransforms();
      if (!scene->isStateColliding(walk))
      {
        solution = std::make_shared<moveit::core::RobotState>(walk);
        RCLCPP_INFO(LOGGER, "IK: walk solve on-branch (%d solves, %d step failures)", solves,
                    failures);
        return true;
      }
      RCLCPP_INFO(LOGGER, "IK: walk solve collided, falling back to multi-seed");
    }
  }

  // Legacy ordering: the consistency attempt ran after the walk. Kept behind
  // the flag so the old behaviour stays one parameter away.
  if (!config_.cheap_solve_first)
  {
    if (auto s = try_consistency())
    {
      solution = s;
      RCLCPP_INFO(LOGGER, "IK: consistency-limited solve from current state");
      return true;
    }
  }

  // Seed candidates: given seed -> "home" -> aimed home -> random draws.
  //
  // "Aimed" seeds: KDL's Newton iteration barely rotates j1 away from its
  // seed value. A seed whose base points elsewhere forces the wrist to
  // contort around to the target (the "reachable but awkward posture"
  // symptom) and fails outright near the workspace boundary. Pointing j1 at
  // the target first makes the natural branch the easy one to find.
  std::vector<moveit::core::RobotStatePtr> seeds;
  seeds.push_back(std::make_shared<moveit::core::RobotState>(seed));

  moveit::core::RobotState home(model_);
  home.setToDefaultValues(group, "home");
  if (rail)
    home.setVariablePosition(rail_joint_, seed_rail);
  home.update();
  seeds.push_back(std::make_shared<moveit::core::RobotState>(home));

  const moveit::core::JointModel* base_joint = group->getActiveJointModels().front();
  double aim = std::atan2(tool0_pose.translation().y(), tool0_pose.translation().x());
  const auto& j1_bounds = base_joint->getVariableBounds();
  if (!j1_bounds.empty() && j1_bounds.front().position_bounded_)
    aim = std::min(std::max(aim, j1_bounds.front().min_position_),
                   j1_bounds.front().max_position_);
  if (base_joint->getType() == moveit::core::JointModel::REVOLUTE)
  {
    moveit::core::RobotState aimed(home);
    aimed.setJointPositions(base_joint, &aim);
    aimed.update();
    seeds.push_back(std::make_shared<moveit::core::RobotState>(aimed));
  }

  moveit::core::RobotState random_state(model_);
  random_state.setToDefaultValues();
  random_state.setToRandomPositions();
  const int attempts =
      max_attempts >= 0 ? max_attempts : config_.random_seed_attempts;
  // deterministic j1 jitter pattern around the aimed value: the base should
  // stay pointed near the target; random j2..j6 explore the arm branches
  static const double JIT[] = { 0.0, +0.2, -0.2, +0.4, -0.4 };
  for (int i = 0; i < attempts; ++i)
  {
    // keep drawing from ONE state so its lazily-created RNG advances — a fresh
    // state per draw would create a new time-seeded RNG each time and produce
    // identical "random" seeds within the same second
    random_state.setToRandomPositions(group);
    // the arm is randomized; keep the operating rail station fixed
    if (rail)
      random_state.setVariablePosition(rail_joint_, seed_rail);
    if (base_joint->getType() == moveit::core::JointModel::REVOLUTE)
    {
      double j1_val = aim + JIT[i % 5];
      if (!j1_bounds.empty() && j1_bounds.front().position_bounded_)
        j1_val = std::min(std::max(j1_val, j1_bounds.front().min_position_),
                          j1_bounds.front().max_position_);
      random_state.setJointPositions(base_joint, &j1_val);
    }
    seeds.push_back(std::make_shared<moveit::core::RobotState>(random_state));
  }

  // Try every seed; among the collision-free solutions keep the one closest
  // to EITHER the current state or "home" in joint space. Closest-to-current
  // alone anchors the pick to whatever branch the robot happens to be parked
  // on — if that is an awkward wrapped posture, the whole edge traversal
  // inherits it and stalls; the home reference lets the natural branch win.
  std::vector<double> seed_joints, home_joints;
  seed.copyJointGroupPositions(group, seed_joints);
  home.copyJointGroupPositions(group, home_joints);

  // Evaluate the seed list concurrently: seeds are independent RobotStates and
  // KDL builds its solver per searchPositionIK call, so this is safe. The
  // winner is chosen exactly as before — smallest joint-space distance to
  // seed-or-home, ties broken by the lowest seed index — so the resulting
  // posture is unchanged; only the wall time drops.
  struct SeedEval
  {
    bool ok = false;
    double dist = std::numeric_limits<double>::max();
    moveit::core::RobotStatePtr state;
  };
  std::vector<SeedEval> evals(seeds.size());
  std::atomic<bool> hit_zero{ false };

  parallelFor(seeds.size(), threads(), [&](std::size_t i) {
    if (hit_zero.load(std::memory_order_relaxed))
      return;  // an exact seed/home hit cannot be beaten
    moveit::core::RobotState candidate(*seeds[i]);
    if (!candidate.setFromIK(group, tool0_pose, tip_link_, std::vector<double>(),
                             config_.ik_timeout, validity))
      return;
    candidate.updateCollisionBodyTransforms();
    if (scene->isStateColliding(candidate))
      return;

    std::vector<double> cand_joints;
    candidate.copyJointGroupPositions(group, cand_joints);
    double dist_seed = 0.0, dist_home = 0.0;
    for (std::size_t j = 0; j < cand_joints.size(); ++j)
    {
      const double ds = cand_joints[j] - seed_joints[j];
      const double dh = cand_joints[j] - home_joints[j];
      dist_seed += ds * ds;
      dist_home += dh * dh;
    }
    SeedEval& e = evals[i];
    e.ok = true;
    // Prefer solutions that do not park j4/j6 right at the wrap (+/-pi): those
    // are legal but force the following Cartesian run across the discontinuity
    // (observed j4 = -3.48 rad truncating a seam at its first waypoint).
    double wrap_penalty = 0.0;
    for (const std::size_t k : { static_cast<std::size_t>(3), static_cast<std::size_t>(5) })
      if (k < cand_joints.size())
      {
        const double a = std::fabs(cand_joints[k]);
        if (a > 3.0)
          wrap_penalty += (a - 3.0) * (a - 3.0) * 10.0;
      }
    e.dist = std::min(dist_seed, dist_home) + wrap_penalty;
    e.state = std::make_shared<moveit::core::RobotState>(candidate);
    if (e.dist <= 1e-12)
      hit_zero.store(true, std::memory_order_relaxed);
    RCLCPP_DEBUG(LOGGER, "IK seed %zu ok: dist %.2f", i + 1, std::sqrt(e.dist));
  });

  std::size_t best_idx = seeds.size();
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < evals.size(); ++i)
    if (evals[i].ok && evals[i].dist < best_dist)  // strict: lowest index wins ties
    {
      best_dist = evals[i].dist;
      best_idx = i;
    }
  if (best_idx < seeds.size())
  {
    solution = evals[best_idx].state;
    RCLCPP_INFO(LOGGER, "IK: multi-seed best (joint dist %.2f, seed %zu/%zu)",
                std::sqrt(best_dist), best_idx + 1, seeds.size());
    return true;
  }
  return false;
}

bool EdgeFollower::solveIkWithRailAllocation(
    const Eigen::Isometry3d& tool0_pose, const moveit::core::RobotState& seed,
    const planning_scene::PlanningSceneConstPtr& scene,
    moveit::core::RobotStatePtr& solution)
{
  if (solveIkMultiSeed(tool0_pose, seed, scene, solution))
    return true;
  if (!hasRail())
    return false;

  moveit::core::RobotState base(seed);
  base.update();
  const double current = base.getVariablePosition(rail_joint_);
  const double desired = railStationFor(tool0_pose.translation(), base);

  const auto& bounds = model_->getJointModel(rail_joint_)->getVariableBounds();
  auto clamp_station = [&bounds](double v) {
    if (bounds.empty() || !bounds.front().position_bounded_)
      return v;
    return std::min(std::max(v, bounds.front().min_position_), bounds.front().max_position_);
  };

  std::vector<double> candidates;
  for (const double s : { desired, current - 0.25, current + 0.25, current - 0.5, current + 0.5 })
  {
    const double c = clamp_station(s);
    if (std::abs(c - current) < 1e-6)
      continue;
    if (std::find(candidates.begin(), candidates.end(), c) == candidates.end())
      candidates.push_back(c);
  }

  for (const double s : candidates)
  {
    moveit::core::RobotState trial(seed);
    setRailStation(trial, s);
    if (solveIkMultiSeed(tool0_pose, trial, scene, solution, 40))
    {
      RCLCPP_INFO(LOGGER, "pose IK solved after moving the rail to %.3f m", s);
      return true;
    }
  }
  return false;
}

double EdgeFollower::runCartesian(const moveit::core::RobotState& start,
                                  const std::vector<Eigen::Isometry3d>& tcp_waypoints,
                                  const planning_scene::PlanningSceneConstPtr& scene,
                                  std::vector<moveit::core::RobotStatePtr>& out_states)
{
  // Cartesian interpolation moves the 6-DOF arm; the carriage is fixed for
  // the duration of the chunk (relocations re-station it with OMPL).
  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(arm_group_);

  // Wrist joints 4 and 6 wrap at +/-pi; a Cartesian step that crosses the wrap
  // cannot be executed continuously, so such states are rejected outright
  // instead of letting the interpolation die halfway along the seam.
  constexpr double kWristWrap = 3.0;  // rad (~172 deg)
  const auto wrist_wrapped = [&group](const moveit::core::RobotState& st) {
    std::vector<double> q;
    st.copyJointGroupPositions(group, q);
    for (const std::size_t k : { static_cast<std::size_t>(3), static_cast<std::size_t>(5) })
      if (k < q.size() && std::fabs(q[k]) > kWristWrap)
        return true;
    return false;
  };

  // NOTE: wrapped wrist values are *legal* (the joints travel to +/-pi); only a
  // discontinuous jump across the wrap breaks the Cartesian run, and that is
  // what MoveIt's JumpThreshold is for. Rejecting |q| > 3.0 here truncated
  // otherwise-fine seams (edge 45 regressed to 0.73), so the wrap is only
  // diagnosed, never used as a hard validity test.
  auto validity = [&scene](moveit::core::RobotState* state,
                           const moveit::core::JointModelGroup* /*group*/,
                           const double* /*values*/) {
    state->updateCollisionBodyTransforms();
    return !scene->isStateColliding(*state);
  };

  moveit::core::RobotState work(start);
  if (wrist_wrapped(work))
    RCLCPP_WARN(LOGGER,
                "cartesian starts on a wrapped wrist branch (j4/j6 near +/-pi); "
                "the run will truncate early - approach IK should have avoided it");
  EigenSTL::vector_Isometry3d waypoints(tcp_waypoints.begin(), tcp_waypoints.end());

  const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &work, group, out_states, model_->getLinkModel(tip_link_), waypoints,
      /*global_reference_frame=*/true,
      moveit::core::MaxEEFStep(config_.eef_step_translation, config_.eef_step_rotation),
      moveit::core::JumpThreshold(config_.jump_threshold_factor), validity);

  if (fraction < 1.0 && !out_states.empty())
  {
    // Diagnose where the traversal stopped: joint values at the truncation
    // point and a verbose collision check of that exact state.
    std::vector<double> jq;
    out_states.back()->copyJointGroupPositions(group, jq);
    std::string js;
    char buf[64];
    for (double v : jq)
    {
      snprintf(buf, sizeof(buf), "%.3f ", v);
      js += buf;
    }
    RCLCPP_WARN(LOGGER, "cartesian truncated at fraction %.2f (waypoint %zu/%zu); joints: %s",
                fraction, out_states.size(), waypoints.size(), js.c_str());
    scene->isStateColliding(*out_states.back(), "", true);
    const Eigen::Isometry3d tip_at_end = out_states.back()->getGlobalLinkTransform(tip_link_);
    RCLCPP_WARN(LOGGER, "tip at truncation: (%.3f, %.3f, %.3f)", tip_at_end.translation().x(),
                tip_at_end.translation().y(), tip_at_end.translation().z());
  }
  return fraction;
}

// Welding needs a constant travel speed *along the seam*, while the robot still
// has to accelerate from rest somewhere. TOTG/Ruckig put those ramps inside the
// seam (the first/last centimetres), which shows up as "slow - fast - slow" on
// the seam. This re-times the seam span at a fixed speed and shifts the rest of
// the trajectory accordingly; the ramps then live on the approach/depart legs.
// Welding profile:
//   approach: rest -> rest (trapezoid)
//   seam:     0 -> v -> 0, constant v in between (ramps live *inside* the seam
//             but are only a few mm long; the two ends must be at rest because
//             the torch strikes / extinguishes the arc there)
//   depart:   rest -> rest (trapezoid)
// Returns the achieved seam speed [m/s] or 0 when it could not be applied.
double EdgeFollower::applySeamConstantSpeed(robot_trajectory::RobotTrajectory& rt,
                                            const std::vector<Eigen::Isometry3d>& seam_frames,
                                            double weld_speed, double dwell, double vel_scale,
                                            double acc_scale)
{
  if (seam_frames.size() < 2 || rt.size() < 5)
    return 0.0;

  std::vector<Eigen::Vector3d> tcp(rt.size());
  for (std::size_t i = 0; i < rt.size(); ++i)
    tcp[i] = rt.getWayPoint(i).getGlobalLinkTransform(tip_link_).translation();

  auto frameDist = [&](const Eigen::Vector3d& p) {
    double best = 1e9;
    for (const auto& f : seam_frames)
      best = std::min(best, (p - f.translation()).norm());
    return best;
  };
  long i0 = -1, i1 = -1;
  for (std::size_t i = 0; i < tcp.size(); ++i)
    if (frameDist(tcp[i]) < 2e-3)
    {
      if (i0 < 0)
        i0 = static_cast<long>(i);
      i1 = static_cast<long>(i);
    }
  if (i0 < 0 || i1 - i0 < 3)
    return 0.0;

  std::vector<double> len(tcp.size(), 0.0);
  for (std::size_t i = 1; i < tcp.size(); ++i)
    len[i] = len[i - 1] + (tcp[i] - tcp[i - 1]).norm();
  const double span = len[i1] - len[i0];
  if (span < 1e-4)
    return 0.0;

  // Target speed: explicit, else the median seam speed of the current profile.
  double v = weld_speed;
  if (v <= 0.0)
  {
    std::vector<double> ds;
    for (long i = i0 + 1; i <= i1; ++i)
      ds.push_back((tcp[i] - tcp[i - 1]).norm());
    std::sort(ds.begin(), ds.end());
    const double dt = rt.getAverageSegmentDuration();
    if (dt <= 0.0)
      return 0.0;
    v = ds[ds.size() / 2] / dt;
  }
  if (v <= 1e-6)
    return 0.0;

  // Ramp at the two seam ends: the TCP must be standing still exactly at the
  // strike/extinguish points, so the acceleration happens inside the seam over
  // a short, configurable distance.
  const double ramp = std::max(1e-3, std::min(0.02, 0.25 * span));
  const double a_ramp = v * v / (2.0 * ramp);
  double t_ramp = v / a_ramp;
  double cruise = span - 2.0 * ramp;
  if (cruise < 0.0)
  {
    // Seam shorter than the two ramps: triangular profile, lower peak speed.
    cruise = 0.0;
    const double v_peak = std::sqrt(a_ramp * span);  // keep the same accel
    v = std::min(v, v_peak);
    t_ramp = v / a_ramp;
  }
  const double seam_time = 2.0 * t_ramp + cruise / v;

  auto seamTimeAt = [&](double s) {
    if (s <= 0.0)
      return 0.0;
    if (s <= ramp)
      return std::sqrt(2.0 * s / a_ramp);
    if (s >= span - ramp && ramp > 0.0)
      return seam_time - std::sqrt(2.0 * std::max(0.0, span - s) / a_ramp);
    return t_ramp + (s - ramp) / v;
  };

  // Approach / depart: rest-to-rest trapezoids at a conservative speed.
  const double v_move = std::max(0.02, std::min(0.3, 2.0 * v));
  const double a_move = std::max(0.2, 4.0 * a_ramp);
  auto trapTime = [&](double s, double L) {
    if (L < 1e-6)
      return 0.0;
    double vv = v_move;
    double d = vv * vv / (2.0 * a_move);
    if (2.0 * d > L)
    {
      vv = std::sqrt(a_move * L);
      d = vv * vv / (2.0 * a_move);
    }
    const double T = (L - 2.0 * d) / vv + 2.0 * (vv / a_move);
    if (s <= 0.0)
      return 0.0;
    if (s <= d)
      return std::sqrt(2.0 * s / a_move);
    if (s >= L - d)
      return T - std::sqrt(2.0 * std::max(0.0, L - s) / a_move);
    return vv / (2.0 * a_move) + s / vv;
  };

  // Joint velocity guard: slow the seam down until every step is legal.
  const auto* group = rt.getGroup();
  const std::vector<std::string>& vars = group->getVariableNames();
  std::vector<Eigen::VectorXd> q(rt.size());
  for (std::size_t i = 0; i < rt.size(); ++i)
  {
    Eigen::VectorXd vq(vars.size());
    const moveit::core::RobotState& st = rt.getWayPoint(i);
    for (std::size_t j = 0; j < vars.size(); ++j)
      vq[j] = st.getVariablePosition(vars[j]);
    q[i] = vq;
  }
  auto overload = [&](double speed) {
    const double a_r = speed * speed / (2.0 * ramp);
    const double tr = speed / std::max(1e-9, a_r);
    for (long i = i0 + 1; i <= i1; ++i)
    {
      const double ds = len[i] - len[i - 1];
      double dt;
      if (len[i] - len[i0] <= ramp)
        dt = std::sqrt(2.0 * std::max(1e-9, len[i] - len[i0]) / a_r) -
             std::sqrt(2.0 * std::max(0.0, len[i - 1] - len[i0]) / a_r);
      else if (len[i] - len[i0] >= span - ramp)
        dt = std::sqrt(2.0 * std::max(0.0, span - (len[i - 1] - len[i0])) / a_r) -
             std::sqrt(2.0 * std::max(0.0, span - (len[i] - len[i0])) / a_r);
      else
        dt = ds / speed;
      (void)tr;
      if (dt <= 1e-9)
        continue;
      for (std::size_t j = 0; j < vars.size(); ++j)
      {
        const auto& b = model_->getVariableBounds(vars[j]);
        if (b.max_velocity_ > 1e-9 && std::fabs(q[i][j] - q[i - 1][j]) / dt > b.max_velocity_)
          return true;
      }
    }
    return false;
  };
  int guard = 0;
  while (overload(v) && guard++ < 25)
    v *= 0.9;
  if (guard >= 25)
    return 0.0;

  // Approach / depart are not welds: run them time-optimally (TOTG over that
  // slice of the trajectory, rest to rest). Only if TOTG fails do we fall back
  // to the conservative trapezoid.
  auto totgTimes = [&](std::size_t a, std::size_t b) {
    std::vector<double> ts;
    if (b <= a)
      return ts;
    robot_trajectory::RobotTrajectory sub(model_, rt.getGroup()->getName());
    for (std::size_t i = a; i <= b; ++i)
      sub.addSuffixWayPoint(rt.getWayPoint(i), 0.0);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(sub, vel_scale, acc_scale))
      return ts;
    ts.resize(b - a + 1);
    for (std::size_t i = 0; i < ts.size(); ++i)
      ts[i] = sub.getWayPointDurationFromStart(i);
    return ts;
  };

  std::vector<double> t(rt.size(), 0.0);
  const double L_pre = len[i0];
  const auto pre = totgTimes(0, static_cast<std::size_t>(i0));
  for (long i = 0; i <= i0; ++i)
    t[i] = pre.size() == static_cast<std::size_t>(i0) + 1 ? pre[static_cast<std::size_t>(i)]
                                                          : trapTime(len[i], L_pre);
  for (long i = i0 + 1; i <= i1; ++i)
    t[i] = t[i0] + seamTimeAt(len[i] - len[i0]);
  const double t_end_seam = t[i1];
  const double L_post = len.back() - len[i1];
  const auto post = totgTimes(static_cast<std::size_t>(i1), rt.size() - 1);
  for (std::size_t i = static_cast<std::size_t>(i1) + 1; i < rt.size(); ++i)
    t[i] = t_end_seam + (post.size() == rt.size() - static_cast<std::size_t>(i1)
                             ? post[i - static_cast<std::size_t>(i1)]
                             : trapTime(len[i] - len[i1], L_post));

  rt.setWayPointDurationFromPrevious(0, 0.0);
  for (std::size_t i = 1; i < rt.size(); ++i)
  {
    const double dt = t[i] - t[i - 1];
    rt.setWayPointDurationFromPrevious(i, dt > 1e-6 ? dt : 1e-6);
  }

  // Arc strike / extinguish: hold the torch still at both seam ends. A zero
  // velocity instant is not enough in practice - the process needs the TCP to
  // sit on the point while the arc establishes/extinguishes.
  if (dwell > 1e-3)
  {
    moveit::core::RobotState start_state = rt.getWayPoint(static_cast<std::size_t>(i0));
    rt.insertWayPoint(static_cast<std::size_t>(i0) + 1, start_state, dwell);
    // The start dwell shifted everything after i0 by one, so the seam end now
    // lives at i1 + 1 (it used to be i1).
    const std::size_t end_idx = static_cast<std::size_t>(i1) + 1;
    moveit::core::RobotState end_state = rt.getWayPoint(end_idx);
    rt.insertWayPoint(end_idx + 1, end_state, dwell);
  }
  return v;
}

EdgeFollowOutcome EdgeFollower::execute(const rosim::core::EdgeData& edge,
                                        const EdgeFollowParams& params,
                                        const moveit::core::RobotState& current,
                                        const planning_scene::PlanningSceneConstPtr& scene,
                                        const std::atomic<bool>& cancel,
                                        const FeedbackCb& feedback,
                                        const NoteCb& note)
{
  EdgeFollowOutcome outcome;
  const moveit::core::JointModelGroup* group = model_->getJointModelGroup(group_);

  // Defensive: callers may hand us a state straight from /joint_states with
  // dirty transforms; everything below reads global link transforms.
  moveit::core::RobotState start(current);
  start.update();

  // Give this run its walk budget. The walk is a sequential continuation
  // ladder: excellent at staying on the seed's IK branch, but it mostly fails
  // on near-boundary edges and then falls back to the (now parallel) seed
  // search — spending most of the planning time for nothing.
  walk_budget_.store(config_.walk_attempts > 0 ? config_.walk_attempts
                                               : std::numeric_limits<int>::max());

  // IK/probe seed for the allocated station. Kept separate from `start`,
  // which stays the *actual* state the trajectory must begin at: planning
  // from a re-stationed copy made the first trajectory waypoint disagree with
  // the real carriage position, so the controller snapped the rail there on
  // the first control cycle instead of driving it.
  moveit::core::RobotState ik_seed(start);
  bool traversed_reverse = false;  // which polyline tcp_frames was built from
  auto report_error = [&](uint8_t code, const std::string& msg) {
    outcome.error_code = code;
    outcome.message = msg;
    RCLCPP_ERROR(LOGGER, "edge follow failed: %s", msg.c_str());
    return std::move(outcome);
  };

  // ---------------- phase: resolving -------------------------------------
  if (feedback)
    feedback(0.05f, "resolving");
  if (cancel.load())
    return report_error(ErrorCode::ERROR_CANCELED, "canceled before planning");

  // 1. TCP frames along the edge (tool_tip convention: +Z along tangent).
  rosim::core::EdgeFrameGenerator::Config gen_cfg;
  gen_cfg.orientation_mode = params.orientation_mode;
  gen_cfg.fixed_orientation = params.fixed_orientation;
  gen_cfg.tangent_tilt = params.tangent_tilt;
  const double spacing = std::min(params.max_waypoint_spacing, edge.length / 4.0);

  rosim::core::EdgeData resampled;
  resampled.index = edge.index;
  resampled.closed = edge.closed;
  resampled.length = edge.length;
  {
    // resample from the polyline with uniform spacing (dense already, but cheap)
    double accumulated = 0.0;
    resampled.polyline.push_back(edge.polyline.front());
    for (size_t i = 1; i < edge.polyline.size(); ++i)
    {
      accumulated += (edge.polyline[i] - edge.polyline[i - 1]).norm();
      if (accumulated >= spacing)
      {
        resampled.polyline.push_back(edge.polyline[i]);
        accumulated = 0.0;
      }
    }
    if ((resampled.polyline.back() - edge.polyline.back()).norm() > 1e-6)
      resampled.polyline.push_back(edge.polyline.back());
  }

  // ---------------- 7th axis: allocate the rail station -------------------
  // The carriage is the coarse axis: bring the seam's centre into the arm's
  // comfortable working envelope before any IK is attempted. Edges longer
  // than one station can cover are handled by the relocation loop, which
  // re-stations the carriage and lets OMPL plan the 7-DOF transfer.
  if (hasRail())
  {
    Eigen::Vector3d mid = Eigen::Vector3d::Zero();
    for (const auto& p : resampled.polyline)
      mid += p;
    mid /= static_cast<double>(resampled.polyline.size());
    const double station = railStationFor(mid, start);
    setRailStation(ik_seed, station);
    RCLCPP_INFO(LOGGER, "rail station allocated: %.3f m (seam centre x=%.3f)", station,
                mid.x());
  }

  const double standoff = params.tool_offset;

  auto probeSet = [](const std::vector<Eigen::Isometry3d>& f) {
    std::vector<Eigen::Isometry3d> poses;
    if (f.size() >= 2)
    {
      poses.push_back(f.front());
      poses.push_back(f[f.size() / 2]);
      poses.push_back(f.back());
    }
    return poses;
  };

  // Frame generation + tilt-direction auto-flip: the lead angle rotates the
  // torch one fixed way in the edge frame. On edges where that leans the torch
  // INTO the solid (e.g. step-top edges) every waypoint collides; flipping the
  // sign leans it away instead. The vote samples edge frames only — approach
  // feasibility is handled below with its own fallback.
  auto buildFrames = [&](const std::vector<Eigen::Vector3d>& poly,
                         const rosim::core::EdgeFrameGenerator::Config& base) {
    auto frames = rosim::core::EdgeFrameGenerator::generate(poly, base);
    if (frames.size() >= 2 &&
        base.orientation_mode == rosim::core::EdgeFrameGenerator::ORIENTATION_TANGENT &&
        base.tangent_tilt != 0.0 && !toolpathFeasible(probeSet(frames), ik_seed, scene))
    {
      auto cfg = base;
      cfg.tangent_tilt = -cfg.tangent_tilt;
      auto flipped = rosim::core::EdgeFrameGenerator::generate(poly, cfg);
      if (flipped.size() >= 2 && toolpathFeasible(probeSet(flipped), ik_seed, scene))
      {
        RCLCPP_INFO(LOGGER, "tilt auto-flip: %.1f deg infeasible, using %.1f deg",
                    base.tangent_tilt * 180.0 / M_PI, cfg.tangent_tilt * 180.0 / M_PI);
        return flipped;
      }
      // else: keep the original frames; downstream reports the honest failure
    }
    return frames;
  };

  // Frames are built per requested direction further down: a forced direction
  // must not pay for the other direction's feasibility probes.
  std::vector<Eigen::Isometry3d> tcp_frames;

  // All IK / Cartesian targets below are for tip_link (the TCP, i.e. the
  // torch tip), so the TCP rides exactly on the edge; tool_offset retreats it
  // along the tool axis if a standoff is requested.

  // 2/3. approach with adaptive retreat. Near the reach boundary the feasible
  // set for a tilted torch orientation is a thin shell: the edge frames can be
  // comfortably reachable while a full 8cm retreat is not. Try the tool-axis
  // retreat first, then a world-up hover (how a real welder descends onto a
  // joint), shrinking the retreat geometrically down to ~1cm — a short escort
  // before arc-on is perfectly normal in welding.
  auto hoverPose = [](const Eigen::Isometry3d& frame, double back) {
    Eigen::Isometry3d h = frame;
    h.translation() = frame.translation() + Eigen::Vector3d(0, 0, back);
    return h;
  };

  // Seed budget per approach candidate: the first candidate explores the full
  // branch tree; once it fails, the subsequent retreat/hover variants are
  // variations on a theme and only need a handful of draws. On a hard edge the
  // retries outnumber the real attempt ~10:1, so this is where the wall time
  // goes if every candidate pays for 63 seeds.
  auto tryApproach = [&](const std::vector<Eigen::Isometry3d>& frames,
                         Eigen::Isometry3d& approach_out,
                         moveit::core::RobotStatePtr& state_out) {
    bool have = false;
    bool first_candidate = true;
    auto solve_candidate = [&](const Eigen::Isometry3d& pose) {
      const int attempts = first_candidate ? -1 : config_.seed_attempts_retry;
      first_candidate = false;
      return solveIkMultiSeed(pose, ik_seed, scene, state_out, attempts);
    };
    for (double d = params.approach_distance; d > 0.009 && !have; d *= 0.5)
    {
      const double back = d + standoff;
      approach_out = frames.front() * Eigen::Translation3d(0, 0, -back);
      if (solve_candidate(approach_out))
      {
        have = true;
      }
      else
      {
        const Eigen::Isometry3d hover = hoverPose(frames.front(), back);
        if (solve_candidate(hover))
        {
          approach_out = hover;
          have = true;
          RCLCPP_INFO(LOGGER, "approach: world-up hover at %.3f m", back);
        }
      }
      if (have && back < params.approach_distance + standoff - 1e-9)
        RCLCPP_INFO(LOGGER, "approach: retreat shortened to %.3f m (reach/collision boundary)",
                    back);
    }
    return have;
  };

  Eigen::Isometry3d approach;
  moveit::core::RobotStatePtr approach_state;
  bool have_approach = false;

  auto reversed_poly = [&]() {
    return std::vector<Eigen::Vector3d>(resampled.polyline.rbegin(),
                                        resampled.polyline.rend());
  };
  auto build_frames = [&](const std::vector<Eigen::Vector3d>& poly) {
    tcp_frames = buildFrames(poly, gen_cfg);
    return tcp_frames.size() >= 2;
  };

  if (params.direction == EdgeFollowParams::DIRECTION_FORWARD ||
      params.direction == EdgeFollowParams::DIRECTION_REVERSE)
  {
    // Operator picked the travel direction: honour it and never silently switch.
    const bool reverse = params.direction == EdgeFollowParams::DIRECTION_REVERSE;
    if (!build_frames(reverse ? reversed_poly() : resampled.polyline))
      return report_error(ErrorCode::ERROR_INVALID, "edge produced <2 frames");
    traversed_reverse = reverse;
    have_approach = tryApproach(tcp_frames, approach, approach_state);
    outcome.direction_used = reverse ? 1 : 0;
    RCLCPP_INFO(LOGGER, "direction forced: %s", reverse ? "reverse" : "forward");
  }
  else
  {
    if (!build_frames(resampled.polyline))
      return report_error(ErrorCode::ERROR_INVALID, "edge produced <2 frames");
    have_approach = tryApproach(tcp_frames, approach, approach_state);
    outcome.direction_used = 0;

    if (!have_approach)
    {
      // Edge direction is part of the posture, not just the order of travel: on
      // a vertical edge the torch follows the tangent, so the "up" direction
      // points the torch body into the table (wrist below the base plane, out of
      // reach) while the reversed tangent points it down and hangs the wrist in
      // free space. Rebuild the toolpath on the reversed polyline and accept it
      // if that approach solves; downhill is also the usual choice on vertical
      // seams.
      auto rev_frames = buildFrames(reversed_poly(), gen_cfg);
      if (rev_frames.size() >= 2 && tryApproach(rev_frames, approach, approach_state))
      {
        RCLCPP_INFO(LOGGER,
                    "approach unreachable along the edge direction; traversing the edge in reverse");
        tcp_frames = std::move(rev_frames);
        have_approach = true;
        outcome.direction_used = 1;
        traversed_reverse = true;
      }
    }
  }

  // Last resort before giving up: shift the carriage. The reachable shell moves
  // with the base, so a seam that is awkward at the allocated station (or sits
  // just outside the arm envelope) often becomes feasible 0.25-0.5 m away.
  if (!have_approach && hasRail())
  {
    const auto& rb = model_->getJointModel(rail_joint_)->getVariableBounds();
    const double base_station = ik_seed.getVariablePosition(rail_joint_);
    // A forced direction is not negotiable here either: only the chosen
    // polyline is retried at the new station.
    const bool allow_forward = params.direction != EdgeFollowParams::DIRECTION_REVERSE;
    const bool allow_reverse = params.direction != EdgeFollowParams::DIRECTION_FORWARD;
    for (const double off : { -0.25, 0.25, -0.5, 0.5, -0.75, 0.75 })
    {
      double station = base_station + off;
      if (!rb.empty() && rb.front().position_bounded_)
        station = std::min(std::max(station, rb.front().min_position_),
                           rb.front().max_position_);
      if (std::abs(station - base_station) < 1e-6)
        continue;
      setRailStation(ik_seed, station);

      if (allow_forward)
      {
        auto frames = buildFrames(resampled.polyline, gen_cfg);
        if (frames.size() >= 2 && tryApproach(frames, approach, approach_state))
        {
          tcp_frames = std::move(frames);
          have_approach = true;
          outcome.direction_used = 0;
          traversed_reverse = false;
        }
      }
      if (!have_approach && allow_reverse)
      {
        auto rev_frames = buildFrames(reversed_poly(), gen_cfg);
        if (rev_frames.size() >= 2 && tryApproach(rev_frames, approach, approach_state))
        {
          tcp_frames = std::move(rev_frames);
          have_approach = true;
          outcome.direction_used = 1;
          traversed_reverse = true;
        }
      }
      if (have_approach)
      {
        RCLCPP_INFO(LOGGER, "approach solved after re-stationing the rail to %.3f m", station);
        break;
      }
    }
  }

  if (!have_approach)
  {
    if (params.direction == EdgeFollowParams::DIRECTION_FORWARD)
      return report_error(ErrorCode::ERROR_IK,
                          "requested FORWARD direction is unreachable at this pose; "
                          "try AUTO or REVERSE");
    if (params.direction == EdgeFollowParams::DIRECTION_REVERSE)
      return report_error(ErrorCode::ERROR_IK,
                          "requested REVERSE direction is unreachable at this pose; "
                          "try AUTO or FORWARD");
    return report_error(
        ErrorCode::ERROR_IK,
        "no collision-free IK at approach pose (unreachable from either edge direction)");
  }

  // ---- corners -----------------------------------------------------------
  // A merged seam has tangent discontinuities where its source edges meet.
  // Replace each sharp corner with a tangent-continuous arc (bounded by
  // params.corner_max_deviation) and rotate the tool over the same window; if
  // the segments are too short, rotate in place instead. Everything is decided
  // here, before execution, so the operator can be warned up front.
  {
    rosim::core::CornerPolicy cp;
    cp.max_deviation =
        params.corner_max_deviation > 0.0 ? params.corner_max_deviation : 5e-4;
    cp.angle_threshold_deg =
        params.corner_threshold_deg > 0.0 ? params.corner_threshold_deg : 8.0;
    cp.orient_window = params.corner_orient_window > 0.0 ? params.corner_orient_window : 0.02;
    const auto& active_poly = traversed_reverse ? reversed_poly() : resampled.polyline;
    if (tcp_frames.size() == active_poly.size() && tcp_frames.size() >= 3)
    {
      auto cp_plan = rosim::core::blendCorners(active_poly, tcp_frames, cp);
      if (cp_plan.frames.size() >= 2)
        tcp_frames = std::move(cp_plan.frames);
      outcome.corner_count = cp_plan.corners.size();
      outcome.corner_pivot_count = cp_plan.pivot_count;
      if (!cp_plan.summary.empty())
      {
        RCLCPP_INFO(LOGGER, "%s", cp_plan.summary.c_str());
        if (note)
          note(cp_plan.summary);
      }
    }
  }

  outcome.toolpath = tcp_frames;

  Eigen::Isometry3d depart = tcp_frames.back() *
      Eigen::Translation3d(0, 0, params.approach_distance + standoff);

  // Depart: the welding path runs a little past the seam end along its
  // tangent. Past-the-end poses sit at the reach boundary and can graze the
  // workpiece corner by <1 mm — enough that the *executed* final state is
  // rejected as START_STATE_INVALID by the next OMPL request, which bricks
  // GoHome and the next follow. Search run-off lengths and the world-up hover,
  // keep the candidate with the largest clearance *whose straight insertion is
  // also collision-free*: a hover above a plate-bottom seam is itself in free
  // air but lies on the far side of the solid, so interpolating up to it drives
  // the torch through the part and stalls the cartesian run.
  {
    struct Cand
    {
      Eigen::Isometry3d pose;
      moveit::core::RobotStatePtr state;
      double clearance = -1.0;
    };
    Cand best;
    Cand best_runoff;
    best.pose = depart;

    // Sample the straight tip path from the seam end to a candidate. The IK
    // branch can change along it, so intermediate poses are solved too (seeded
    // from the candidate to stay on its branch). Orientation is identical at
    // both ends, hence a plain translation lerp.
    auto segmentClear = [&](const Eigen::Isometry3d& target,
                            const moveit::core::RobotState& seed) {
      for (const double t : { 0.1, 0.33, 0.66, 0.9 })
      {
        Eigen::Isometry3d mid = tcp_frames.back();
        mid.translation() =
            (1.0 - t) * tcp_frames.back().translation() + t * target.translation();
        moveit::core::RobotStatePtr s;
        if (!solveIkMultiSeed(mid, seed, scene, s) || scene->isStateColliding(*s))
          return false;
      }
      return true;
    };

    std::vector<double> dists = { params.approach_distance,
                                  params.approach_distance * 1.5,
                                  params.approach_distance * 2.0 };
    for (double d = params.approach_distance * 0.5; d > 0.009; d *= 0.5)
      dists.push_back(d);
    dists.push_back(0.01);

    int depart_candidate = 0;
    for (double d : dists)
    {
      const double adv = d + standoff;
      for (int kind = 0; kind < 2; ++kind)
      {
        const Eigen::Isometry3d pose =
            kind == 0 ? tcp_frames.back() * Eigen::Translation3d(0, 0, adv)
                      : hoverPose(tcp_frames.back(), adv);
        moveit::core::RobotStatePtr solved;
        // First candidate gets the full budget, the rest are variations.
        const int att = depart_candidate++ == 0 ? -1 : config_.seed_attempts_retry;
        if (!solveIkMultiSeed(pose, ik_seed, scene, solved, att))
          continue;
        const double clearance = scene->distanceToCollisionUnpadded(*solved);
        const Cand cand{ pose, solved, clearance };
        if (clearance > best.clearance)
          best = cand;
        if (kind == 0 && clearance > best_runoff.clearance)
          best_runoff = cand;
      }
      if (best.clearance >= 0.01)
        break;  // 1 cm of air is enough to absorb execution tolerances
    }

    Cand chosen;
    if (best.state && segmentClear(best.pose, *best.state))
      chosen = best;
    else if (best_runoff.state && segmentClear(best_runoff.pose, *best_runoff.state))
      chosen = best_runoff;

    if (chosen.state)
    {
      depart = chosen.pose;
      RCLCPP_INFO(LOGGER, "depart clearance %.1f mm at (%.3f, %.3f, %.3f) (segment validated)",
                  chosen.clearance * 1000.0, chosen.pose.translation().x(),
                  chosen.pose.translation().y(), chosen.pose.translation().z());
    }
    else if (best.state)
    {
      depart = tcp_frames.back();
      RCLCPP_WARN(LOGGER, "no depart candidate has a clear insertion; ending on the seam");
    }
    else
    {
      RCLCPP_WARN(LOGGER, "depart pose has no collision-free IK; cartesian will truncate");
    }
  }
  if (cancel.load())
    return report_error(ErrorCode::ERROR_CANCELED, "canceled after IK");

  // ---------------- phase: planning (OMPL free-space) ---------------------
  if (feedback)
    feedback(0.15f, "planning");

  // OMPL rejects in-collision/out-of-bounds start states with a bare -26; make
  // the reason visible before handing the problem to the planner.
  if (scene->isStateColliding(start, "", true))
    RCLCPP_WARN(LOGGER, "start state is in collision (see contacts above)");

  PlanRequest preq(start);
  approach_state->copyJointGroupPositions(group, preq.goal_joint_positions);
  preq.scene = scene;
  preq.group = group_;
  preq.allowed_planning_time = config_.allowed_planning_time;
  preq.velocity_scaling = params.velocity_scaling;
  preq.acceleration_scaling = params.acceleration_scaling;

  PlanResult preres;
  if (!planner_->plan(preq, preres) || !preres.success || !preres.trajectory)
    return report_error(ErrorCode::ERROR_OMPL, "OMPL pre-approach failed: " + preres.error);
  if (cancel.load())
    return report_error(ErrorCode::ERROR_CANCELED, "canceled after OMPL");

  // ---------------- phase: following (Cartesian along the edge) -----------
  if (feedback)
    feedback(0.45f, "following");

  const moveit::core::RobotState& ompl_end = preres.trajectory->getLastWayPoint();
  std::vector<Eigen::Isometry3d> waypoints;
  waypoints.push_back(approach);  // straight insertion from hover to edge start
  waypoints.insert(waypoints.end(), tcp_frames.begin(), tcp_frames.end());
  waypoints.push_back(depart);

  std::vector<moveit::core::RobotStatePtr> cart_states;
  double fraction = runCartesian(ompl_end, waypoints, scene, cart_states);
  outcome.cartesian_fraction = fraction;

  // L3 degradation: re-connect with OMPL to an alternate IK further along.
  int relocations = 0;
  while (fraction < config_.min_cartesian_fraction &&
         relocations < config_.max_relocations && !cancel.load())
  {
    ++relocations;
    RCLCPP_WARN(LOGGER, "cartesian path incomplete (fraction %.2f), relocation %d",
                fraction, relocations);

    const size_t reached = static_cast<size_t>(fraction * static_cast<double>(waypoints.size()));
    size_t resume = std::min(waypoints.size() - 1, reached + 3);

    // Re-station the carriage for the remaining waypoints: the next chunk can
    // lie beyond the current station's arm envelope. IK holds the rail at the
    // new value; OMPL then transfers the 7-DOF group (rail + arm) there, so
    // the carriage participates in the relocation instead of being dragged.
    moveit::core::RobotState resume_seed(*cart_states.back());
    resume_seed.update();
    if (hasRail())
      setRailStation(resume_seed,
                     railStationFor(waypoints[resume].translation(), resume_seed));

    moveit::core::RobotStatePtr resume_state;
    if (!solveIkMultiSeed(waypoints[resume], resume_seed, scene, resume_state))
      break;  // fall through with the partial fraction

    PlanRequest rreq(*cart_states.back());
    resume_state->copyJointGroupPositions(group, rreq.goal_joint_positions);
    rreq.scene = scene;
    rreq.group = group_;
    rreq.allowed_planning_time = config_.allowed_planning_time;
    rreq.velocity_scaling = params.velocity_scaling;
    rreq.acceleration_scaling = params.acceleration_scaling;

    PlanResult rres;
    if (!planner_->plan(rreq, rres) || !rres.success)
      break;

    // cartesian continues from the relocation target onward
    std::vector<Eigen::Isometry3d> remaining(waypoints.begin() + resume, waypoints.end());
    std::vector<moveit::core::RobotStatePtr> more;
    const double tail = runCartesian(rres.trajectory->getLastWayPoint(), remaining, scene, more);

    if (more.empty() || tail <= 0.0)
      break;

    // splice: [OMPL(reloc) + tail states]
    for (size_t i = 0; i < rres.trajectory->size(); ++i)
    {
      const auto& wp = rres.trajectory->getWayPointPtr(i);
      if (!cart_states.empty() && sameGroupState(*wp, *cart_states.back(), group))
        continue;
      cart_states.push_back(wp);
    }
    for (auto& s : more)
    {
      if (!cart_states.empty() && sameGroupState(*s, *cart_states.back(), group))
        continue;
      cart_states.push_back(s);
    }
    fraction = static_cast<double>(reached + tail * static_cast<double>(waypoints.size() - resume)) /
               static_cast<double>(waypoints.size());
    outcome.cartesian_fraction = fraction;
  }

  if (cancel.load())
    return report_error(ErrorCode::ERROR_CANCELED, "canceled during cartesian traversal");
  if (fraction < config_.min_cartesian_fraction)
    return report_error(ErrorCode::ERROR_CARTESIAN,
                        "edge not fully traversable, fraction " + std::to_string(fraction));

  // The Cartesian validity callback only rejects *interpenetrating* states.
  // FCL can report an exactly-touching pose as free space when the step is
  // generated and as a zero-depth contact a moment later, so a run can legally
  // end with the torch grazing a part corner. OMPL then rejects that touch as
  // START_STATE_INVALID (bare -26), which blocked GoHome and the next follow
  // until the operator jogged clear. Never hand the controller a grazing end
  // state: trim back to the last state with real air around the torch and
  // reconnect to a validated retreat pose (run-off along the torch axis, or a
  // hover perpendicular to it) so the cell is always left in free space.
  // A 1.5 mm band is enough to keep OMPL's start-state check off the
  // zero-depth knife edge. Welding legitimately runs within a few millimetres
  // of the part, so a larger margin would trim away most of the seam.
  constexpr double kEndClearance = 0.0015;
  size_t keep = cart_states.size();
  for (const double margin : { kEndClearance, 0.0005 })
  {
    while (keep > 0 && scene->distanceToCollisionUnpadded(*cart_states[keep - 1]) < margin)
      --keep;
    if (keep > 0)
      break;
  }
  if (keep == 0)
    return report_error(ErrorCode::ERROR_CARTESIAN,
                        "cartesian path never leaves contact with the scene");

  size_t trimmed = cart_states.size() - keep;
  cart_states.resize(keep);
  if (trimmed > 0)
  {
    cart_states.back()->update();
    const double clear = scene->distanceToCollisionUnpadded(*cart_states.back());
    RCLCPP_WARN(LOGGER,
                "end state grazed the scene; trimmed %zu cartesian states, ending %.1f mm clear",
                trimmed, clear * 1000.0);

    // Retreat targets, nearest first. +Z/-Z are the torch axis (run-off or
    // back-off along the seam in TANGENT mode); world up/down covers tilted
    // and fixed-orientation toolpaths.
    const Eigen::Isometry3d tip = cart_states.back()->getGlobalLinkTransform(tip_link_);
    std::vector<Eigen::Isometry3d> retreats;
    for (const double d : { 0.04, 0.08, 0.12, 0.18, 0.25 })
    {
      retreats.push_back(tip * Eigen::Translation3d(0, 0, d));
      retreats.push_back(tip * Eigen::Translation3d(0, 0, -d));
      Eigen::Isometry3d up = tip;
      up.translation().z() += d;
      retreats.push_back(up);
      Eigen::Isometry3d down = tip;
      down.translation().z() -= d;
      retreats.push_back(down);
    }

    int plans = 0;
    bool retreated = false;
    for (const auto& pose : retreats)
    {
      moveit::core::RobotStatePtr goal;
      if (!solveIkMultiSeed(pose, *cart_states.back(), scene, goal))
        continue;
      if (scene->distanceToCollisionUnpadded(*goal) < kEndClearance)
        continue;

      PlanRequest rreq(*cart_states.back());
      goal->copyJointGroupPositions(group, rreq.goal_joint_positions);
      rreq.scene = scene;
      rreq.group = group_;
      rreq.allowed_planning_time = config_.allowed_planning_time;
      rreq.velocity_scaling = params.velocity_scaling;
      rreq.acceleration_scaling = params.acceleration_scaling;

      PlanResult rres;
      if (++plans > 3)
        break;  // a short retreat should not need a second search
      if (!planner_->plan(rreq, rres) || !rres.success || !rres.trajectory)
        continue;

      for (size_t i = 0; i < rres.trajectory->size(); ++i)
      {
        const auto& wp = rres.trajectory->getWayPointPtr(i);
        if (sameGroupState(*wp, *cart_states.back(), group))
          continue;
        cart_states.push_back(wp);
      }
      retreated = true;
      RCLCPP_INFO(LOGGER, "search retreat planned to (%.3f, %.3f, %.3f)",
                  pose.translation().x(), pose.translation().y(), pose.translation().z());
      break;
    }
    if (!retreated)
      RCLCPP_WARN(LOGGER, "no retreat planned; ending at the trimmed state (%.1f mm clear)",
                  clear * 1000.0);
  }
  else
  {
    RCLCPP_INFO(LOGGER, "cartesian end clearance %.1f mm",
                scene->distanceToCollisionUnpadded(*cart_states.back()) * 1000.0);
  }

  // ---------------- merge + time parameterization -------------------------
  auto merged = std::make_unique<robot_trajectory::RobotTrajectory>(model_, group);
  for (size_t i = 0; i < preres.trajectory->size(); ++i)
    merged->addSuffixWayPoint(preres.trajectory->getWayPointPtr(i),
                              preres.trajectory->getWayPointDurationFromPrevious(i));
  for (const auto& s : cart_states)
  {
    if (merged->size() > 0 && sameGroupState(*s, merged->getLastWayPoint(), group))
      continue;
    merged->addSuffixWayPoint(s, 0.0);
  }
  if (merged->size() < 2)
    return report_error(ErrorCode::ERROR_CARTESIAN, "merged trajectory degenerate");

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(*merged, params.velocity_scaling, params.acceleration_scaling))
    return report_error(ErrorCode::ERROR_CARTESIAN, "time parameterization failed");

  if (params.scurve)
  {
    // TOTG gives a trapezoidal profile (jerk is infinite at the acceleration
    // switches); this pass re-times it with a jerk limit, i.e. a double-S
    // profile. Limits are keyed by *variable* name and are taken from the
    // robot model (scaled like the caller asked), falling back to jerk_limit.
    std::unordered_map<std::string, double> vlim, alim, jlim;
    for (const moveit::core::JointModel* jm : merged->getGroup()->getActiveJointModels())
    {
      const auto& bounds = jm->getVariableBounds();
      const auto& names = jm->getVariableNames();
      const bool prismatic = jm->getType() == moveit::core::JointModel::PRISMATIC;
      for (std::size_t i = 0; i < names.size() && i < bounds.size(); ++i)
      {
        vlim[names[i]] = bounds[i].max_velocity_ * params.velocity_scaling;
        alim[names[i]] = (bounds[i].max_acceleration_ > 1e-9 ? bounds[i].max_acceleration_ : 1.0) *
                         params.acceleration_scaling;
        jlim[names[i]] = bounds[i].max_jerk_ > 1e-9
                             ? bounds[i].max_jerk_
                             : (prismatic ? params.jerk_limit * 0.05 : params.jerk_limit);
      }
    }
    if (!trajectory_processing::RuckigSmoothing::applySmoothing(*merged, vlim, alim, jlim))
      RCLCPP_WARN(LOGGER, "Ruckig smoothing failed; keeping the TOTG profile");
  }

  if (params.seam_constant_speed)
  {
    const double v =
        applySeamConstantSpeed(*merged, tcp_frames, params.weld_speed, params.seam_dwell,
                               params.velocity_scaling, params.acceleration_scaling);
    if (v > 0.0)
      RCLCPP_INFO(LOGGER, "seam constant speed: %.1f mm/s", v * 1000.0);
    else
      RCLCPP_WARN(LOGGER, "seam constant speed could not be applied; keeping the profile");
  }

  outcome.trajectory = std::move(merged);
  outcome.success = true;
  outcome.error_code = ErrorCode::ERROR_NONE;
  outcome.message = "edge followed, fraction " + std::to_string(fraction) +
                    (trimmed > 0 ? " (grazing end trimmed, retreated to free space)" : "");
  if (feedback)
    feedback(1.0f, "done");
  return outcome;
}

}  // namespace rosim::planning
