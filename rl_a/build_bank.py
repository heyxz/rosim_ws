"""预生成"工件位姿 → 接近关节构型"样本库。

背景：每回合都做一次 IK 要 140 ms，训练时相当于每步多 0.5 ms。
改成一次性生成 N 条可行样本存成 npz，各并行环境直接读，reset 变成查表。

    python3 build_bank.py --count 150
"""

from __future__ import annotations

import argparse
import time

import numpy as np

import kin
from env import WORKPIECE_CENTRE

# 标定值（见 scene_env.py）：真实接近 goal 的 TCP 位姿相对工件中心的偏移
R_REAL = np.array([[0.564643, 0.0, -0.825335],
                   [0.0, 1.0, 0.0],
                   [0.825335, 0.0, 0.564643]])
LOCAL = np.array([0.090, -0.290, -0.045])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=150)
    ap.add_argument("--out", default="scene_bank.npz")
    ap.add_argument("--xy", type=float, default=0.12)
    ap.add_argument("--z", type=float, default=0.04)
    ap.add_argument("--yaw", type=float, default=30.0)
    args = ap.parse_args()

    rng = np.random.default_rng(7)
    poses, goals = [], []
    t0 = time.time()
    tries = 0
    while len(goals) < args.count and tries < args.count * 6:
        tries += 1
        centre = WORKPIECE_CENTRE + rng.uniform(-1, 1, 3) * [args.xy, args.xy, args.z]
        yaw = np.deg2rad(rng.uniform(-args.yaw, args.yaw))
        c, s = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        p = Rz @ LOCAL + centre
        Rt = Rz @ R_REAL

        best_q, best_err = None, 9.0
        for _ in range(6):
            seed = np.clip(kin.Q_HOME + rng.uniform(-1.2, 1.2, 6),
                           kin.LIMITS[:, 0], kin.LIMITS[:, 1])
            q, err = kin.ik(Rt, p, seed, iters=120)
            if err < best_err:
                best_q, best_err = q, err
            if best_err < 4e-3:
                break
        if best_err < 4e-3:
            poses.append([*centre, yaw])
            goals.append(best_q)

    poses = np.array(poses)
    goals = np.array(goals)
    np.savez(args.out, poses=poses, goals=goals)
    print(f"{len(goals)} 条样本（试了 {tries} 次），耗时 {time.time()-t0:.0f}s -> {args.out}")


if __name__ == "__main__":
    main()
