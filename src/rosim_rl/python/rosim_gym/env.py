"""rosim gym environment skeleton.

Bridges the rosim simulation to a gym API for future RL training.

Today it provides:
  - joint state observation (/joint_states)
  - episode control via the FollowEdge action and ImportModel service

The action->rl_step plumbing lands together with the C++ RlPlanner stepping
loop (rosim_rl/src/rl_planner.cpp): expose ~/rl_reset, ~/rl_step,
~/rl_observation services on the planning node and forward them here.

Usage (skeleton smoke test, no torch required):
  python3 random_rollout.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rosim_interfaces.action import FollowEdge
from rosim_interfaces.srv import ImportModel
from sensor_msgs.msg import JointState


class RosimEnv:
    """Minimal RL environment view over the rosim ROS2 interfaces."""

    OBS_DIM = 22  # must match rosim::rl::RlPlanner::Spaces::OBS_DIM
    ACT_DIM = 6   # joint increments, +/-0.1 rad (Spaces::MAX_ACTION_RAD)

    def __init__(self, node: Node):
        self.node = node
        self.joint_names = []
        self.joint_positions = np.zeros(self.ACT_DIM)
        self._create_interfaces()

    def _create_interfaces(self):
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10)
        self.follow_client = ActionClient(self.node, FollowEdge, "/rosim_planning/follow_edge")
        self.import_client = self.node.create_client(ImportModel, "/rosim_planning/import_model")

    def _on_joint_state(self, msg):
        self.joint_names = list(msg.name)
        self.joint_positions = np.array(msg.position[: self.ACT_DIM])

    # ------------------------------------------------------------------ gym-ish
    def reset(self):
        """Start a new episode. V1: an episode is one FollowEdge goal."""
        # TODO(rl): call ~/rl_reset once the planning node exposes it.
        return self.observation()

    def observation(self) -> np.ndarray:
        """Partial observation (6 joints). The full 22-dim vector including
        goal/TCP/collision terms is computed C++-side in RlPlanner::observation
        (single source of truth) and will arrive via ~/rl_observation."""
        obs = np.zeros(self.OBS_DIM, dtype=np.float32)
        obs[: len(self.joint_positions)] = self.joint_positions
        return obs

    def step(self, action: np.ndarray):
        """Apply a 6-dim joint increment. Not wired until ~/rl_step exists."""
        raise NotImplementedError(
            "rl_step service not exposed yet — implement it next to "
            "RlPlanner::plan() stepping, then forward actions here"
        )

    # ------------------------------------------------------------------ helpers
    def import_workpiece(self, path: str, x=0.40, y=-0.15, z=0.30):
        """Import a STEP file; returns (ok, model_id, edge_count)."""
        self.import_client.wait_for_service(timeout_sec=5.0)
        req = ImportModel.Request()
        req.file_path = path
        req.world_pose.position.x = float(x)
        req.world_pose.position.y = float(y)
        req.world_pose.position.z = float(z)
        req.world_pose.orientation.w = 1.0
        future = self.import_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        res = future.result()
        return res.success, res.model_id, len(res.edges)

    def follow_edge(self, model_id: str, edge_index: int, **kwargs):
        """Send a FollowEdge goal; returns (success, cartesian_fraction)."""
        goal = FollowEdge.Goal()
        goal.model_id = model_id
        goal.edge_index = edge_index
        for key, value in kwargs.items():
            setattr(goal, key, value)
        self.follow_client.wait_for_server(timeout_sec=5.0)
        goal_future = self.follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=10.0)
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            return False, 0.0
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=180.0)
        result = result_future.result().result
        return result.success, result.cartesian_fraction
