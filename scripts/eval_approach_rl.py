#!/usr/bin/env python3
"""用真实接近目标评测 RL 策略：读 .scratch/approach_targets.jsonl，逐个调 /rl_plan。

数据由 EdgeFollower 在每次接近规划时采集（组顺序 [rail_joint, j1..j6]）。

用法（需要 rosim-rl-policy 在跑）：
    python3 scripts/eval_approach_rl.py
    python3 scripts/eval_approach_rl.py --gate 1.5     # 只看闸门会放行的那批
"""
from __future__ import annotations
import argparse, json, math, sys
import rclpy
from rclpy.node import Node
from rosim_interfaces.srv import RlPlan


class Eval(Node):
    def __init__(self):
        super().__init__("eval_approach_rl")
        self.cli = self.create_client(RlPlan, "/rl_plan")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            print("缺少 /rl_plan：先 systemctl --user start rosim-rl-policy"); sys.exit(1)

    def plan(self, start, goal):
        req = RlPlan.Request()
        req.start = [float(v) for v in start]   # JSON 里可能是 int，ROS 要求 double
        req.goal = [float(v) for v in goal]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        return fut.result()


def arm_delta(start, goal):
    """arm 位移 = j1..j6 的欧氏范数（导轨单列），与 C++ 侧 computeJointDelta 一致。"""
    arm = math.sqrt(sum((g - s) ** 2 for g, s in zip(goal[1:], start[1:])))
    rail = abs(goal[0] - start[0])
    return arm, rail


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--gate", type=float, default=None)
    ap.add_argument("--path", default=".scratch/approach_targets.jsonl"); a = ap.parse_args()
    try:
        rows = [json.loads(l) for l in open(a.path) if l.strip()]
    except FileNotFoundError:
        print(f"没有数据：{a.path}（先跑 workpiece_bench.sh 采集）"); return
    if a.gate is not None:
        rows = [r for r in rows if arm_delta(r["start"], r["goal"])[0] <= a.gate]

    rclpy.init(); n = Eval()
    print(f"{'arm位移':>8} {'rail':>6} {'到达':>4} {'碰撞':>4} {'步数':>5}  message")
    print("-" * 74)
    ok = coll = 0
    for r in rows:
        arm, rail = arm_delta(r["start"], r["goal"])
        res = n.plan(r["start"], r["goal"])
        if res is None:
            print(f"{arm:8.2f} {rail:6.2f}   请求超时"); continue
        steps = max(0, len(res.path) // 7 - 1)   # path 每点 7 个数
        collided = res.min_clearance <= 0.0
        ok += bool(res.success); coll += collided
        print(f"{arm:8.2f} {rail:6.2f} {str(res.success):>4} {str(collided):>4} {steps:5d}  {res.message[:34]}")
    print("-" * 74)
    print(f"样本 {len(rows)} 个：到达 {ok} ({100.0*ok/max(len(rows),1):.0f}%)，碰撞 {coll}")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
