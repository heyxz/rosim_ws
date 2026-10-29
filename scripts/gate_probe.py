#!/usr/bin/env python3
"""测量 H3 的 RL 闸门：先回 home，再按不同幅度生成**有效**目标，看闸门判定。

有效性用 OMPL 先验一遍（OMPL 13 ms，便宜）：OMPL 都解不出的目标算无效，跳过，
避免把"目标本身非法"误记成规划器失败。

用法（需要 rosim-bench 在跑）：
    python3 scripts/gate_probe.py --n 20
"""
from __future__ import annotations
import argparse, random, sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosim_interfaces.srv import PlanJoint, SetPlanner


class Probe(Node):
    def __init__(self):
        super().__init__("gate_probe")
        self.setp = self.create_client(SetPlanner, "/rosim_planning/set_planner")
        self.plan = self.create_client(PlanJoint, "/rosim_planning/plan_joint")
        self.home = self.create_client(Empty := __import__("std_srvs.srv", fromlist=["Empty"]).Empty,
                                       "/rosim_planning/go_home")
        for c in (self.setp, self.plan):
            if not c.wait_for_service(timeout_sec=20.0):
                print("服务未就绪：先 systemctl --user start rosim-bench"); sys.exit(1)
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, m):
        if self.q is None and len(m.position) >= 6:
            self.q = list(m.position[:6])

    def call(self, fut, timeout=90.0):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def spin_wait(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--planner", default="hybrid"); a = ap.parse_args()
    rclpy.init(); n = Probe()

    # 回 home 并等到关节状态稳定
    if n.home.wait_for_service(timeout_sec=5.0):
        n.call(n.home.call_async(n.home.srv_type.Request()))
    n.spin_wait(3.0)
    if n.q is None:
        print("拿不到 /joint_states"); sys.exit(1)
    base = list(n.q)
    print(f"home 关节: {[round(v,3) for v in base]}\n")

    rng = random.Random(11)
    print(f"{'目标幅度':>8} {'OMPL先验':>8} {'hybrid':>8} {'耗时':>7}  message")
    print("-" * 86)
    used = gate_hits = valid = 0
    for mag in [0.3, 0.5, 0.8, 1.0, 1.3, 1.5, 1.8, 2.0, 2.5, 3.0] * (a.n // 10 + 1):
        if used >= a.n:
            break
        goal = [max(-3.0, min(3.0, b + rng.uniform(-mag, mag))) for b in base]
        g = PlanJoint.Request(); g.goal = goal

        # 1) OMPL 先验：确认目标可达/无碰撞
        r = SetPlanner.Request(); r.planner_type = "ompl"; n.call(n.setp.call_async(r))
        ok_ompl = n.call(n.plan.call_async(g)).success
        if not ok_ompl:
            continue
        valid += 1

        # 2) hybrid：闸门是否让它走 RL
        r.planner_type = a.planner; n.call(n.setp.call_async(r))
        t0 = time.time(); res = n.call(n.plan.call_async(g)); dt = (time.time() - t0) * 1000
        msg = res.message
        # RL 分支会 2 s 左右（Python rollout），OMPL 几十 ms
        hit = "超出 RL 范围" in msg
        gate_hits += hit; used += 1
        print(f"{mag:8.1f} {'成功':>8} {str(res.success):>8} {dt:6.0f}ms  {msg[:52]}")

    print("-" * 86)
    print(f"有效目标 {valid} 个，闸门拦截（转 OMPL）{gate_hits} 个 = {100.0*gate_hits/max(valid,1):.0f}%")
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
