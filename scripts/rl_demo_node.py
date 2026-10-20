#!/usr/bin/env python3
"""RL 路径规划演示节点（方案 B）。

加载 demo 训练出的 PPO 策略（ONNX + VecNormalize 统计），从机器人**当前构型**
出发跑一条 episode，把走过的关节路径发布成 JointTrajectory。调试服务端订阅
/rl_demo/joint_trajectory 之后会用现成的 TCP 轨迹管线把它画出来（按速度配色）。

观测/动作与训练代码完全一致（直接 import demo 的 env.py / robot.py），所以这里
不存在 sim2sim 的观测差异；差异只来自"真实场景 vs 训练用的球近似障碍"。

用法:
    ./rl_demo.sh                       # 一条 episode
    ./rl_demo.sh --episodes 5 --seed 3
    ./rl_demo.sh --from-home           # 忽略当前构型，从 Q_HOME 出发
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

DEMO_DIR = os.environ.get("RL_DEMO_DIR",
                          "/home/xz/rosim_ws/.scratch/rl_demo_ws/rosim_ws/rl_demo")
if DEMO_DIR not in sys.path:
    sys.path.insert(0, DEMO_DIR)

import robot  # noqa: E402  (demo 的运动学)
from env import ArmEnv, MAX_DQ  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]


class Policy:
    """ONNX 策略 + VecNormalize 统计（与 sb3 训练时一致）。"""

    def __init__(self, onnx_path: str, meta_path: str):
        import json
        import onnxruntime as ort

        meta = json.load(open(meta_path))
        self.mean = np.asarray(meta["obs_mean"], dtype=np.float32)
        self.var = np.asarray(meta["obs_var"], dtype=np.float32)
        self.clip = float(meta.get("clip_obs", 5.0))
        self.max_dq = float(meta.get("max_dq", MAX_DQ))
        self.eps = 1e-8

        self.sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.input = self.sess.get_inputs()[0].name
        self.eval_success = float(meta.get("eval_success", float("nan")))

    def act(self, obs: np.ndarray) -> np.ndarray:
        x = (np.asarray(obs, dtype=np.float32) - self.mean) / np.sqrt(self.var + self.eps)
        x = np.clip(x, -self.clip, self.clip).reshape(1, -1)
        a = self.sess.run(None, {self.input: x})[0].reshape(-1)
        return np.clip(a, -1.0, 1.0)


class RlDemoNode(Node):
    def __init__(self, args):
        super().__init__("rl_demo_node")
        self.args = args
        self.policy = Policy(args.onnx, args.meta)

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(JointTrajectory, args.topic, qos)

        self.current: np.ndarray | None = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.get_logger().info(
            f"policy 就绪 (eval_success={self.policy.eval_success:.1%}, max_dq={self.policy.max_dq})"
        )

    def on_joint_state(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(j in index for j in JOINTS):
            return
        self.current = np.array([msg.position[index[j]] for j in JOINTS], dtype=np.float64)

    def wait_for_state(self, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and self.current is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current is not None

    def run_episode(self, seed: int) -> dict:
        env = ArmEnv(horizon=self.args.horizon, seed=seed, difficulty=1.0)
        env.reset()

        start = robot.Q_HOME.copy()
        if not self.args.from_home and self.current is not None:
            # 当前构型离 home 太远时策略没见过这种起点，退回 home。
            if float(np.max(np.abs(self.current - robot.Q_HOME))) < self.args.start_tol:
                start = self.current.copy()
            else:
                self.get_logger().warn(
                    f"当前构型离 home 太远（max Δq={np.max(np.abs(self.current - robot.Q_HOME)):.2f} rad），"
                    f"从 home 出发"
                )
        env.q = np.clip(start, robot.LIMITS[:, 0], robot.LIMITS[:, 1])
        env.prev_dist = float(np.linalg.norm(robot.forward(env.q)[1] - env.target))
        env.steps = 0

        path = [env.q.copy()]
        min_clear = float("inf")
        reached = collided = False
        while True:
            obs = env._obs()
            action = self.policy.act(obs)
            env.step(action)  # env 内部会 clip 到 ±1 再乘 MAX_DQ
            path.append(env.q.copy())
            pts, tcp = robot.forward(env.q)
            clear = robot.min_clearance(pts, env.obstacles)
            min_clear = min(min_clear, clear)
            dist = float(np.linalg.norm(tcp - env.target))
            if dist < 0.04:
                reached = True
                break
            if clear < 0.0:
                collided = True
                break
            if env.steps >= self.args.horizon:
                break
        return {
            "path": np.array(path),
            "reached": reached,
            "collided": collided,
            "steps": len(path) - 1,
            "min_clearance": min_clear,
            "target": env.target,
            "start": start,
        }

    def publish_path(self, path: np.ndarray, spacing: float) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = JOINTS
        for i, q in enumerate(path):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.time_from_start.sec = int(i * spacing)
            pt.time_from_start.nanosec = int((i * spacing % 1.0) * 1e9)
            msg.points.append(pt)
        self.pub.publish(msg)
        self.get_logger().info(f"已发布 {len(msg.points)} 个路径点到 {self.args.topic}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default=os.path.join(DEMO_DIR, "out_sb3/policy.onnx"))
    ap.add_argument("--meta", default=os.path.join(DEMO_DIR, "out_sb3/policy.json"))
    ap.add_argument("--topic", default="/rl_demo/joint_trajectory")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--horizon", type=int, default=180)
    ap.add_argument("--spacing", type=float, default=0.1, help="路径点时间间隔 (s)")
    ap.add_argument("--from-home", action="store_true")
    ap.add_argument("--start-tol", type=float, default=1.0,
                    help="当前构型与 home 的最大允许关节差 (rad)")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = RlDemoNode(args)
    node.wait_for_state()

    ok = 0
    for k in range(args.episodes):
        result = node.run_episode(args.seed + k)
        node.get_logger().info(
            f"episode {k + 1}: steps={result['steps']} reached={result['reached']} "
            f"collided={result['collided']} min_clearance={result['min_clearance'] * 1000:.0f} mm"
        )
        node.publish_path(result["path"], args.spacing)
        ok += int(result["reached"] and not result["collided"])
        if k + 1 < args.episodes:
            time.sleep(1.0)

    node.get_logger().info(f"完成 {args.episodes} 条，成功 {ok} 条")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
