#!/usr/bin/env python3
"""RL 策略推理节点（方案 A 的 Python 半边）。

提供 /rl_plan 服务：给定起始关节和目标关节，跑一遍策略，返回关节路径。
观测构造直接复用训练代码（rl_a/kin.py），所以不存在 sim2sim 的观测差异；
碰撞判定在 C++ 侧用 MoveIt/FCL 逐点复检，这里只负责“策略怎么说”。

用法:
    ./scripts/rl_plan.sh
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

RL_A_DIR = os.environ.get("RL_A_DIR", "/home/xz/rosim_ws/rl_a")
if RL_A_DIR not in sys.path:
    sys.path.insert(0, RL_A_DIR)

import kin  # noqa: E402

import onnxruntime as ort  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosim_interfaces.srv import RlPlan  # noqa: E402

TOL = 0.10          # rad，与训练时一致
MAX_STEPS = 300
MAX_DQ = 0.10

# 训练时的工件模型（固定盒子 → 球阵）。C++ 侧会用真实 FCL 复检。
WORKPIECE_CENTRE = np.array([0.540, 0.450, 0.600])
WORKPIECE_HALF = np.array([0.110, 0.170, 0.080])
SPHERE_RADIUS = 0.05


class Policy:
    def __init__(self, out_dir: str):
        meta = json.load(open(os.path.join(out_dir, "policy.json")))
        self.mean = np.asarray(meta["obs_mean"], np.float32)
        self.var = np.asarray(meta["obs_var"], np.float32)
        self.clip = float(meta.get("clip_obs", 5.0))
        self.sess = ort.InferenceSession(os.path.join(out_dir, "policy.onnx"),
                                         providers=["CPUExecutionProvider"])
        self.input = self.sess.get_inputs()[0].name
        self.eval_success = float(meta.get("eval_success", float("nan")))

    def act(self, obs: np.ndarray) -> np.ndarray:
        x = (np.asarray(obs, np.float32) - self.mean) / np.sqrt(self.var + 1e-8)
        x = np.clip(x, -self.clip, self.clip).reshape(1, -1)
        return np.clip(self.sess.run(None, {self.input: x})[0].ravel(), -1.0, 1.0)


class RlPlanNode(Node):
    def __init__(self, out_dir: str):
        super().__init__("rl_plan_node")
        self.policy = Policy(out_dir)
        self.obstacles = kin.box_spheres(WORKPIECE_CENTRE, WORKPIECE_HALF, SPHERE_RADIUS)
        self.srv = self.create_service(RlPlan, "/rl_plan", self.handle_plan)
        self.get_logger().info(
            f"rl_plan 就绪：{out_dir}（eval_success={self.policy.eval_success:.1%}，"
            f"障碍球 {len(self.obstacles)} 个）"
        )

    def observation(self, q: np.ndarray, goal: np.ndarray) -> np.ndarray:
        pts, tcp = kin.forward(q)
        clearance = kin.min_clearance(pts, self.obstacles)
        return np.concatenate([
            q / 3.0, goal / 3.0, (goal - q) / 3.0, tcp,
            [float(np.clip(clearance, -0.2, 1.0))],
        ]).astype(np.float32)

    def _log_request(self, q, goal, reached, collided, steps):
        """把每次调用追加到 JSONL；RL_PLAN_LOG 未设置则不记录。"""
        path = os.environ.get("RL_PLAN_LOG")
        if not path:
            return
        try:
            with open(path, "a") as f:
                f.write(json.dumps({
                    "start": q.tolist(), "goal": goal.tolist(),
                    "reached": bool(reached), "collided": bool(collided),
                    "steps": int(steps),
                    "delta": float(np.linalg.norm(goal - q)),
                }) + "\n")
        except OSError as exc:
            self.get_logger().warn(f"日志写入失败: {exc}")

    def handle_plan(self, request, response):
        if len(request.start) != 6 or len(request.goal) != 6:
            response.success = False
            response.message = "start/goal 必须是 6 维"
            return response

        q = np.clip(np.asarray(request.start, float), kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        goal = np.clip(np.asarray(request.goal, float), kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        q_start = q.copy()

        path = [q.copy()]
        min_clearance = float("inf")
        reached = collided = False
        for step in range(MAX_STEPS):
            action = self.policy.act(self.observation(q, goal))
            q = np.clip(q + action * MAX_DQ, kin.LIMITS[:, 0], kin.LIMITS[:, 1])
            path.append(q.copy())

            pts, _ = kin.forward(q)
            clearance = kin.min_clearance(pts, self.obstacles)
            min_clearance = min(min_clearance, clearance)

            if np.linalg.norm(q - goal) < TOL:
                reached = True
                break
            if clearance < 0.0:
                collided = True
                break

        # 记录真实调用分布（方案 B 的训练数据来源）：start/goal 是调用方真正
        # 发过来的位姿，训练和部署同分布才能避免"训练 90%、真实 0%"。
        self._log_request(q_start, goal, reached, collided, len(path) - 1)

        response.success = bool(reached and not collided)
        response.path = np.concatenate(path).astype(float).tolist()
        response.min_clearance = float(min_clearance if np.isfinite(min_clearance) else 0.0)
        response.message = (
            f"{len(path) - 1} 步，{'到达' if reached else '未到达'}"
            f"{'，碰撞' if collided else ''}"
        )
        self.get_logger().info(f"rl_plan: {response.message}（最小间隙 {response.min_clearance * 1000:.0f} mm）")
        return response


def main() -> int:
    out_dir = os.environ.get("RL_OUT_DIR", os.path.join(RL_A_DIR, "out_a2"))  # RL_OUT_DIR 可指向别的策略目录
    rclpy.init()
    node = RlPlanNode(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
