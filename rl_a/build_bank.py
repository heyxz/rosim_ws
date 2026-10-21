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
from real_scene import RealWorkpiece

# 标定值（见 scene_env.py）：真实接近位姿相对真实工件质心的偏移
R_REAL = np.array([[0.564643, 0.0, -0.825335],
                   [0.0, 1.0, 0.0],
                   [0.825335, 0.0, 0.564643]])
LOCAL = np.array([-0.550, 0.080, -0.091])
PART = RealWorkpiece("/home/xz/rosim_ws/.scratch/scene.json")
# 标称解：真实调用返回的接近构型。工件只在小范围内移动，用它做 IK 种子
# 几乎必然收敛；纯随机种子基本找不到（实测 720 次里只成功 1 次）。
NOMINAL = np.load("nominal_goal.npy")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=150)
    ap.add_argument("--out", default="scene_bank.npz")
    ap.add_argument("--xy", type=float, default=0.03)
    ap.add_argument("--z", type=float, default=0.01)
    ap.add_argument("--yaw", type=float, default=8.0)
    args = ap.parse_args()

    rng = np.random.default_rng(7)
    poses, goals = [], []
    t0 = time.time()
    tries = 0
    while len(goals) < args.count and tries < args.count * 6:
        tries += 1
        centre = PART.centre0 + rng.uniform(-1, 1, 3) * [args.xy, args.xy, args.z]
        yaw = np.deg2rad(rng.uniform(-args.yaw, args.yaw))
        c, s = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        p = Rz @ LOCAL + centre
        Rt = Rz @ R_REAL

        # 标称接近姿态离工件只有 ~16 mm：工件一挪就必须重新求接近位姿。
        # 真实系统也是这么做的（多退让距离 + 多次 IK），这里照做：
        # 沿工具轴往外退 0 / 4 / 8 / 12 cm，取第一个不碰的可行解。
        z_axis = Rt[:, 2]
        ok_q = None
        for back in (0.0, 0.04, 0.08, 0.12):
            target = p + z_axis * back
            for k in range(3):
                seed = NOMINAL if k == 0 else np.clip(
                    NOMINAL + rng.uniform(-0.3, 0.3, 6), kin.LIMITS[:, 0], kin.LIMITS[:, 1])
                q, err = kin.ik(Rt, target, seed, iters=120)
                if err > 4e-3:
                    continue
                pts, _ = kin.forward(q)
                if kin.min_clearance(pts, PART.spheres(centre, yaw)) < 0.015:
                    continue
                ok_q = q
                break
            if ok_q is not None:
                break
        if ok_q is not None:
            poses.append([*centre, yaw])
            goals.append(ok_q)

    poses = np.array(poses)
    goals = np.array(goals)
    np.savez(args.out, poses=poses, goals=goals)
    print(f"{len(goals)} 条样本（试了 {tries} 次），耗时 {time.time()-t0:.0f}s -> {args.out}")


if __name__ == "__main__":
    main()
