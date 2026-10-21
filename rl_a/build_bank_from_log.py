"""把真实调用日志变成训练样本库。

日志每一行是一次真实 /rl_plan 调用：(start, goal, reached, steps)。
直接拿这些 (start, goal) 对训练，训练分布就等于部署分布——比合成采样可靠。

    python3 build_bank_from_log.py --log ../.scratch/rl_requests.jsonl --out scene_bank.npz

样本会按"目标构型"去重（同一个目标只保留一次，起点取最近一次的），
再按 --jitter 对起点加噪扩充条数。
"""

from __future__ import annotations

import argparse
import json

import numpy as np

import kin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="../.scratch/rl_requests.jsonl")
    ap.add_argument("--out", default="scene_bank.npz")
    ap.add_argument("--jitter", type=float, default=0.06, help="起点加噪幅度 [rad]")
    ap.add_argument("--copies", type=int, default=12, help="每个目标扩充几份起点")
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.log) if l.strip()]
    if not rows:
        raise SystemExit("日志为空")

    # 按目标去重（保留 last 一条的起点）
    by_goal: dict[tuple, dict] = {}
    for r in rows:
        key = tuple(np.round(r["goal"], 3))
        by_goal[key] = r

    rng = np.random.default_rng(3)
    goals, starts, reached, deltas = [], [], [], []
    for r in by_goal.values():
        g = np.array(r["goal"])
        s = np.array(r["start"])
        for k in range(args.copies):
            starts.append(s if k == 0 else np.clip(
                s + rng.uniform(-args.jitter, args.jitter, 6),
                kin.LIMITS[:, 0], kin.LIMITS[:, 1]))
            goals.append(g)
            reached.append(bool(r["reached"]))
            deltas.append(float(np.linalg.norm(g - s)))

    # 工件位姿列：这些样本都在标称摆放上采集，所以位姿固定
    from real_scene import RealWorkpiece
    part = RealWorkpiece("/home/xz/rosim_ws/.scratch/scene.json")
    poses = np.tile([*part.centre0, 0.0], (len(goals), 1))

    np.savez(args.out, poses=poses, goals=np.array(goals), starts=np.array(starts),
             reached=np.array(reached), deltas=np.array(deltas))
    print(f"{len(goals)} 条样本（{len(by_goal)} 个不同目标），delta {min(deltas):.2f}~{max(deltas):.2f} rad")
    for g, d in zip(by_goal.values(), [np.linalg.norm(np.array(r["goal"]) - np.array(r["start"]))
                                       for r in by_goal.values()]):
        R, t = kin.fk_pose(np.array(g["goal"]))
        print(f"  目标 TCP {np.round(t,3)}  delta {d:.2f}  reached={g['reached']}")


if __name__ == "__main__":
    main()
