#!/usr/bin/env python3
"""规划算法对比基准：同一批场景，跑不同规划器，出一张表。

用途是**学习和对比算法**，不是生产回归。三组场景：

  free     自由空间的目标（直线通常能过）          —— 看谁快
  long     大位移目标（1.6~2.5 rad）              —— 看 RL 门限与通用性
  blocked  TCP 直线必须穿过工件的目标              —— 看谁会绕障

起点固定为**机器人当前位姿**（plan_joint 就是从当前状态开始规划的），所以不需要
传送机器人；先摆好姿态再跑，就能复现你想研究的场景。

用法:
    ./scripts/rl_plan.sh &                  # rl/auto 需要策略节点
    python3 scripts/benchmark_suite.py
    python3 scripts/benchmark_suite.py --out bench.json
"""

from __future__ import annotations

import argparse
import json
import os
import statistics as st
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosim_interfaces.srv import PlanJoint, SetPlanner
from sensor_msgs.msg import JointState

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "rl_a"))

import kin  # noqa: E402
import real_scene  # noqa: E402

PLANNERS = ("auto", "line", "rl", "ompl")
GROUP_SIZE = {"free": 8, "long": 4, "blocked": 6}


class Bench(Node):
    def __init__(self):
        super().__init__("benchmark_suite")
        self.cur: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._js, qos_profile_sensor_data)
        self.set_planner = self.create_client(SetPlanner, "/rosim_planning/set_planner")
        self.plan = self.create_client(PlanJoint, "/rosim_planning/plan_joint")
        for c in (self.set_planner, self.plan):
            if not c.wait_for_service(timeout_sec=10.0):
                raise SystemExit(f"服务不可用: {c.srv_name}")

    def _js(self, msg):
        self.cur = dict(zip(msg.name, msg.position))

    def start(self) -> np.ndarray:
        t0 = time.time()
        while time.time() - t0 < 10.0 and "j1" not in self.cur:
            rclpy.spin_once(self, timeout_sec=0.1)
        return np.array([self.cur["rail_joint"]] + [self.cur[f"j{i}"] for i in range(1, 7)])

    def use(self, name: str) -> None:
        r = SetPlanner.Request()
        r.planner_type = name
        fut = self.set_planner.call_async(r)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

    def plan_to(self, goal: np.ndarray) -> tuple[bool, float, str]:
        req = PlanJoint.Request()
        req.goal = [float(v) for v in goal]
        t0 = time.time()
        fut = self.plan.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        dt = (time.time() - t0) * 1e3
        res = fut.result()
        if res is None:
            return False, dt, "no response"
        return bool(res.success), dt, res.message


def build_goals(start: np.ndarray, part, rng) -> dict[str, list[np.ndarray]]:
    """三组目标，都相对当前起点。"""
    goals = {"free": [], "long": [], "blocked": []}
    while len(goals["free"]) < GROUP_SIZE["free"]:
        g = np.clip(start + rng.uniform(-1.0, 1.0, 7) * [0.2, 1, 1, 1, 1, 1, 1],
                    kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        if 0.4 <= np.linalg.norm(g - start) <= 1.2:
            goals["free"].append(g)
    while len(goals["long"]) < GROUP_SIZE["long"]:
        g = np.clip(start + rng.uniform(-1, 1, 7) * [0.3, 1, 1, 1, 1, 1, 1],
                    kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        if 1.6 <= np.linalg.norm(g - start) <= 2.5:
            goals["long"].append(g)

    # blocked：TCP 直线穿过工件，且目标构型本身不碰撞
    obs = part.spheres(part.centre0, 0.0)
    p0 = kin.forward(start)[1]
    for _ in range(200000):
        if len(goals["blocked"]) >= GROUP_SIZE["blocked"]:
            break
        g = np.clip(start + rng.uniform(-1, 1, 7) * [0.2, 1, 1, 1, 1, 1, 1],
                    kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        if not (0.3 <= np.linalg.norm(g - start) <= 1.2):
            continue
        if kin.min_clearance(kin.forward(g)[0], obs) < 0.04:
            continue
        p1 = kin.forward(g)[1]
        seg = p1 - p0
        den = float(seg @ seg)
        t = 0.0 if den < 1e-12 else float(np.clip((part.centre0 - p0) @ seg / den, 0, 1))
        if float(np.linalg.norm(part.centre0 - (p0 + t * seg))) < part.radius + 0.05:
            goals["blocked"].append(g)
    return goals


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    rclpy.init()
    node = Bench()
    start = node.start()
    part = real_scene.RealWorkpiece(os.path.join(ROOT, "rl_a", "scene.json"))
    rng = np.random.default_rng(args.seed)
    goals = build_goals(start, part, rng)
    print(f"起点 rail={start[0]:.2f} arm={np.round(start[1:], 2)}")
    for k, v in goals.items():
        print(f"  {k:8s} {len(v)} 个目标")

    results: dict[str, dict] = {}
    for planner in PLANNERS:
        node.use(planner)
        rows = {}
        for group, gs in goals.items():
            ok, times, errs = 0, [], []
            for g in gs:
                success, dt, msg = node.plan_to(g)
                ok += int(success)
                if success:
                    times.append(dt)
                else:
                    errs.append(msg[:70])
            rows[group] = dict(ok=ok, n=len(gs), times=times, errs=errs[:2])
        results[planner] = rows
        print(f"\n[{planner}]")
        for group, row in rows.items():
            med = st.median(row["times"]) if row["times"] else float("nan")
            print(f"  {group:8s} {row['ok']}/{row['n']}  中位 {med:7.1f} ms"
                  + (f"   失败示例: {row['errs'][0]}" if row["errs"] else ""))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=1, ensure_ascii=False)
        print("\n已保存", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
