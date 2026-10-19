#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "rosim_core/edge_frame_generator.h"
#include "rosim_core/occ_geometry_service.h"
#include "rosim_planning/motion_planner.h"

namespace rosim::planning
{

/// \brief Edge-following task parameters (mirror of FollowEdge.action goal).
struct EdgeFollowParams
{
  /// Travel direction along the edge polyline (values mirror FollowEdge.action).
  enum Direction : uint8_t
  {
    DIRECTION_AUTO = 0,     ///< forward first, reverse only if forward is infeasible
    DIRECTION_FORWARD = 1,  ///< force start at polyline.front()
    DIRECTION_REVERSE = 2,  ///< force start at polyline.back()
  };

  uint8_t direction = DIRECTION_AUTO;
  uint8_t orientation_mode = rosim::core::EdgeFrameGenerator::ORIENTATION_TANGENT;
  Eigen::Quaterniond fixed_orientation = Eigen::Quaterniond::Identity();
  double tangent_tilt = 0.0;         ///< TANGENT mode lead angle [rad] (lifts the tool
                                     ///< body off the workpiece surface)
  double tool_offset = 0.0;          ///< extra TCP retreat along -tool Z [m]
  double approach_distance = 0.08;   ///< approach/depart segment length [m]
  double max_waypoint_spacing = 0.005;
  double corner_max_deviation = 5e-4;   ///< allowed off-seam distance at a corner [m]
  double corner_threshold_deg = 8.0;    ///< corners shallower than this are ignored
  double corner_orient_window = 0.02;   ///< +/- window used to spread the turn [m]
  double velocity_scaling = 1.0;
  double acceleration_scaling = 1.0;
  bool scurve = false;          ///< true: jerk-limited (double-S) smoothing after TOTG
  double jerk_limit = 100.0;    ///< revolute jerk limit [rad/s^3] for the S-curve pass
  bool seam_constant_speed = false;  ///< keep the TCP speed constant *on the seam*
  double weld_speed = 0.0;            ///< target seam speed [m/s]; 0 = keep the profile's median
  double seam_dwell = 0.0;            ///< pause at the seam start/end [s] (arc strike/extinguish)
};

/// \brief Result of a full edge-follow pipeline run.
struct EdgeFollowOutcome
{
  bool success = false;
  uint8_t error_code = 0;   ///< FollowEdge action ERROR_* codes
  double cartesian_fraction = 0.0;
  std::string message;
  std::unique_ptr<robot_trajectory::RobotTrajectory> trajectory;  ///< full timed plan
  std::vector<Eigen::Isometry3d> toolpath;  ///< TCP frames along the edge (RL obs / viz)
  uint8_t direction_used = 255;  ///< 0 forward, 1 reverse, 255 none (mirrors the action)
  std::size_t corner_count = 0;       ///< corners detected on the seam
  std::size_t corner_pivot_count = 0; ///< corners handled by in-place rotation
};

/// \brief Orchestrates: edge sampling -> approach IK -> OMPL free-space plan ->
/// Cartesian edge traversal -> merge + TOTG. Pure computation, no ROS topics —
/// the PlanningNode drives it from an action worker thread.
class EdgeFollower
{
public:
  /// phase strings: resolving / planning / following / done
  using FeedbackCb = std::function<void(float progress, const std::string& phase)>;
  /// Out-of-band operator notice (corner plan, fallbacks). May be called during
  /// resolving, i.e. before the trajectory is published.
  using NoteCb = std::function<void(const std::string& note)>;

  struct Config
  {
    int random_seed_attempts = 60;       ///< random IK seeds before giving up (ik_probe shows
                                         ///< ~17 draws per branch; 60 covers most branches)
    double ik_timeout = 0.05;            ///< per-seed IK timeout [s]
    std::vector<double> consistency_limits;  ///< per-joint rad, along-edge continuity
    double eef_step_translation = 0.004;
    double eef_step_rotation = 0.05;
    double jump_threshold_factor = 2.5;  ///< tighter than the 4.0 default: a
                                         ///< loose factor lets the chained IK
                                         ///< leap branches mid-edge (observed
                                         ///< j1 swinging 165° backward) and
                                         ///< strand the traversal
    double min_cartesian_fraction = 0.95;
    int max_relocations = 3;             ///< L3 degradation retries
    double allowed_planning_time = 8.0;
    int parallel_threads = 0;            ///< >0: cap for the IK / feasibility fan-out.
                                         ///< 0 = auto (min(hw-2, 12)); 1 = serial (old behaviour)
    /// How many times *per execute()* the sequential continuation walk may run.
    /// The walk keeps IK on the seed's branch, but it is also the largest cost:
    /// on a typical edge it fails ~12 times and falls back to the seed search
    /// anyway. 1 = first call only (default); <=0 = every call (legacy).
    int walk_attempts = 1;
    /// Try the cheap consistency-limited solve *before* the walk. Same branch
    /// intent, one KDL call instead of an adaptive ladder.
    bool cheap_solve_first = true;
    /// Seed draws for *retry* candidates (second and later approach/depart
    /// variants, and the remaining rail stations). The first candidate still
    /// uses random_seed_attempts; the retries are variations on it.
    int seed_attempts_retry = 12;
  };

  EdgeFollower(rclcpp::Node::SharedPtr node, moveit::core::RobotModelPtr model,
               MotionPlannerPtr planner);

  void setConfig(const Config& config) { config_ = config; }

  /// \param edge        world-frame edge data (from OccGeometryService)
  /// \param current     current robot state (start)
  /// \param scene       planning scene containing the workpiece
  /// \param cancel      checked between phases; returns ERROR_CANCELED if set
  /// Re-time the seam portion of `rt` so the TCP travels at a constant speed.
  /// The acceleration/deceleration ramps are pushed onto the approach/depart
  /// segments. Returns the achieved speed (m/s) or 0 when it could not apply.
  double applySeamConstantSpeed(robot_trajectory::RobotTrajectory& rt,
                                const std::vector<Eigen::Isometry3d>& seam_frames,
                                double weld_speed, double dwell);

  EdgeFollowOutcome execute(const rosim::core::EdgeData& edge,
                            const EdgeFollowParams& params,
                            const moveit::core::RobotState& current,
                            const planning_scene::PlanningSceneConstPtr& scene,
                            const std::atomic<bool>& cancel,
                            const FeedbackCb& feedback = {},
                            const NoteCb& note = {});

private:
  /// Multi-seed IK for a tool0 pose with collision filtering (also public
  /// utility for one-shot pose services).
public:
  bool solveIkMultiSeed(const Eigen::Isometry3d& tool0_pose,
                        const moveit::core::RobotState& seed,
                        const planning_scene::PlanningSceneConstPtr& scene,
                        moveit::core::RobotStatePtr& solution,
                        int max_attempts = -1);

  /// Pose IK with 7th-axis assist: first tries at the seed's rail station,
  /// then re-stations the carriage in 0.25/0.5 m steps. Used by one-shot
  /// MoveToPose so a target outside the current station still plans.
  bool solveIkWithRailAllocation(const Eigen::Isometry3d& tool0_pose,
                                 const moveit::core::RobotState& seed,
                                 const planning_scene::PlanningSceneConstPtr& scene,
                                 moveit::core::RobotStatePtr& solution);

  /// Cartesian traversal through waypoints; returns achieved fraction.
  double runCartesian(const moveit::core::RobotState& start,
                      const std::vector<Eigen::Isometry3d>& tool0_waypoints,
                      const planning_scene::PlanningSceneConstPtr& scene,
                      std::vector<moveit::core::RobotStatePtr>& out_states);

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr model_;
  MotionPlannerPtr planner_;
  Config config_;
  std::string group_ = "manipulator";  // 7-DOF OMPL group (arm + rail)
  std::string arm_group_ = "arm";      // 6-DOF chain for IK / Cartesian
  std::string rail_joint_ = "rail_joint";
  std::string tip_link_ = "tool_tip";  // the TCP (torch wire end) — NOT tool0:
                                       // solving for the flange buries the torch
                                       // 0.10 m inside the workpiece
  Eigen::Isometry3d tool0_to_tip_ = Eigen::Isometry3d::Identity();
  /// Walk budget for the current execute() call (reset at its start).
  std::atomic<int> walk_budget_{ 0 };

private:
  /// Rail station (m) that puts `target` at a comfortable arm extension:
  /// the carriage travels along X, so the station is the target's X minus the
  /// forward lever left after the seam's Y/Z offset, clamped to the travel.
  double railStationFor(const Eigen::Vector3d& target,
                        const moveit::core::RobotState& state) const;

  /// Write the rail variable (no-op if the model has no rail joint) and update.
  void setRailStation(moveit::core::RobotState& state, double station) const;

  /// True if the model has a prismatic rail joint we can allocate.
  bool hasRail() const;

  /// Thread budget for the independent-seed / independent-waypoint fan-out.
  unsigned threads() const;

  /// Cheap feasibility probe: collision-free IK must exist at EVERY given
  /// pose (caller passes the sampled toolpath + its approach pose). Used to
  /// pick the tilt direction.
  bool toolpathFeasible(const std::vector<Eigen::Isometry3d>& poses,
                        const moveit::core::RobotState& seed,
                        const planning_scene::PlanningSceneConstPtr& scene);
};

}  // namespace rosim::planning
