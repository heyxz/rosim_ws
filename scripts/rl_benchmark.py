#!/usr/bin/env python3
"""OMPL vs RL 规划器对比基准。

对同一批随机关节目标，分别用两个规划器调用 /rosim_planning/plan_joint，
统计成功率、耗时、路径长度（关节空间）和最大关节速度。

用法:
    ./scripts/rl_plan.sh &                 # RL 需要一个策略节点
    python3 scripts/rl_benchmark.py --trials 25
"""

from __future__ import annotations

import argparse
import statistics as st
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rosim_interfaces.srv import PlanJoint, SetPlanner

HOME = np.array([0.0, -0.6, 0.0, 0.0, 0.6, 0.0])
# URDF limits for the six arm joints
LIMITS = np.array([[-2.967, 2.967], [-1.571, 2.705], [-1.484, 2.618],
                   [-3.0, 3.0], [-2.618, 2.618], [-3.0, 3.0]])


def goal_dist(traj):
    """Total joint-space path length of a JointTrajectory."""
    pts = [np.array(p.positions[-6:]) for p in traj.points]
    if len(pts) < 2:
        return 0.0
    return float(sum(np.linalg.norm(b - a) for a, b in zip(pts, pts[1:])))


def max_vel(traj):
    v = 0.0
    for p in traj.points:
        if p.velocities:
            v = max(v, max(abs(x) for x in p.velocities[-6:]))
    return v


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=25)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--radius", type=float, default=0.7, help="目标关节偏移幅度 (rad)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("rl_benchmark")
    set_planner = node.create_client(SetPlanner, "/rosim_planning/set_planner")
    plan = node.create_client(PlanJoint, "/rosim_planning/plan_joint")
    for c in (set_planner, plan):
        if not c.wait_for_service(timeout_sec=5.0):
            print(f"服务不可用: {c.srv_name}")
            return 1

    rng = np.random.default_rng(args.seed)
    goals = []
    while len(goals) < args.trials:
        g = np.clip(HOME + rng.uniform(-args.radius, args.radius, 6), LIMITS[:, 0], LIMITS[:, 1])
        if np.linalg.norm(g - HOME) > 0.25:
            goals.append(g)

    results = {}
    for planner in ("ompl", "rl"):
        r = SetPlanner.Request()
        r.planner_type = planner
        fut = set_planner.call_async(r)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().success:
            print(f"切换规划器失败: {planner}")
            continue

        times, lengths, vels = [], [], []
        ok = 0
        for g in goals:
            req = PlanJoint.Request()
            req.goal = [float(x) for x in g]
            fut = plan.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
            res = fut.result() if fut.done() else None
            if res is None:
                continue
            if not res.success:
                continue
            ok += 1
            times.append(_ms(res.message))
            lengths.append(goal_dist(res.trajectory))
            vels.append(max_vel(res.trajectory))
        results[planner] = dict(ok=ok, n=len(goals), times=times, lengths=lengths, vels=vels)

    print(f"\n{'规划器':<8}{'成功':>10}{'中位耗时':>12}{'平均路径(rad)':>16}{'最大速度':>12}")
    for name, r in results.items():
        if not r["times"]:
            print(f"{name:<8}{r['ok']}/{r['n']:>4}      (无成功样本)")
            continue
        print(f"{name:<8}{r['ok']:>4}/{r['n']:<5}{st.median(r['times']):>9.1f} ms"
              f"{st.mean(r['lengths']):>14.2f}{max(r['vels']):>11.2f}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def _ms(message: str) -> float:
    """Extract the '... N ms' figure the plan_joint response carries."""
    import re
    m = re.search(r"([\d.]+) ms", message)
    return float(m.group(1)) if m else float("nan")


if __name__ == "__main__":
    raise SystemExit(main())
