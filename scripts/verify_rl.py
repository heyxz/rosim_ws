#!/usr/bin/env python3
"""验证今天的改动：RL（含兜底/门限）、直线规划器、以及路径直度。

对每种规划器用同一起点规划到关节目标，记录成功/耗时/点数，并用 kin 的 FK
把关节轨迹换算成 TCP，算「相对首尾直线的最大偏离」——直线规划器应该接近 0。

用法:
    python3 scripts/verify_rl.py            # 全部用例
    python3 scripts/verify_rl.py --only line
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import numpy as np

sys.path.insert(0, "/home/xz/rosim_ws/rl_a")
import kin  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from rosim_interfaces.srv import PlanJoint, SetPlanner  # noqa: E402

JOINTS7 = ["rail_joint", "j1", "j2", "j3", "j4", "j5", "j6"]


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_rl")
        self.current = None
        self.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        self.set_planner = self.create_client(SetPlanner, "/rosim_planning/set_planner")
        self.plan_joint = self.create_client(PlanJoint, "/rosim_planning/plan_joint")
        for c in (self.set_planner, self.plan_joint):
            if not c.wait_for_service(timeout_sec=8.0):
                raise RuntimeError(f"服务不可用: {c.srv_name}")

    def on_state(self, msg: JointState):
        index = {n: i for i, n in enumerate(msg.name)}
        if all(j in index for j in JOINTS7):
            self.current = np.array([msg.position[index[j]] for j in JOINTS7])

    def wait_state(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout and self.current is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def switch(self, planner: str) -> bool:
        req = SetPlanner.Request()
        req.planner_type = planner
        fut = self.set_planner.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        return bool(res and res.success)

    def plan(self, goal) -> tuple[bool, str, float, float, float]:
        """→ (success, message, wall_ms, tcp deviation from the chord [m], points)"""
        req = PlanJoint.Request()
        req.goal = [float(x) for x in goal]
        t0 = time.time()
        fut = self.plan_joint.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        wall = (time.time() - t0) * 1000.0
        res = fut.result()
        if res is None:
            return False, "(no response)", wall, float("nan"), 0
        if not res.success:
            return False, res.message, wall, float("nan"), 0

        # TCP straightness over the returned waypoints
        idx = {n: i for i, n in enumerate(res.trajectory.joint_names)}
        if not all(n in idx for n in JOINTS7):
            return True, res.message, wall, float("nan"), len(res.trajectory.points)
        qs = np.array([[p.positions[idx[n]] for n in JOINTS7] for p in res.trajectory.points])
        tcp = np.array([kin.forward(q)[1] for q in qs])
        chord = tcp[-1] - tcp[0]
        length = np.linalg.norm(chord)
        if length < 1e-6:
            dev = 0.0
        else:
            u = chord / length
            dev = float(np.max(np.linalg.norm((tcp - tcp[0]) - np.outer((tcp - tcp[0]) @ u, u), axis=1)))
        return True, res.message, wall, dev, len(res.trajectory.points)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    args = ap.parse_args()

    rclpy.init()
    v = Verifier()
    v.wait_state()
    if v.current is None:
        print("拿不到 /joint_states")
        return 1
    start = v.current.copy()
    print(f"起点: rail={start[0]:.2f}  臂={np.round(start[1:], 2)}")

    cases = [
        ("rl",   0.4, "小位移（焊缝接近量级）"),
        ("rl",   2.0, "大位移（应触发门限直接 OMPL）"),
        ("line", 0.3, "直线规划器"),
        ("ompl", 0.4, "OMPL 基线"),
    ]
    print(f"\n{'规划器':<6}{'Δq':>5}  {'结果':<46}{'耗时':>9}{'点数':>6}{'TCP偏离':>10}")
    for planner, dq, note in cases:
        if args.only and args.only != planner:
            continue
        if not v.switch(planner):
            print(f"{planner:<6}  切换失败")
            continue
        goal = start.copy()
        goal[2] += dq          # 动 j2，最容易看出差别
        ok, msg, wall, dev, pts = v.plan(goal)
        dev_s = "—" if np.isnan(dev) else f"{dev * 1000:.1f} mm"
        print(f"{planner:<6}{dq:>5.1f}  {msg[:44]:<46}{wall:>7.0f}ms{pts:>6}{dev_s:>10}")

    v.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
