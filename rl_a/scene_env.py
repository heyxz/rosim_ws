"""工件位置随机化 + 真实几何的接近环境。

两个关键点（都是踩坑换来的）：
1. 目标不是随机关节角，而是**焊枪在焊缝前的接近位姿**，随工件一起变换；
2. 障碍用 ``/planning_scene`` 导出的**真实网格**（FCL 用的同一份），不是猜的盒子。

接近位姿由真实调用标定：goal 的 TCP = (0.630, 0.160, 0.555)，
真实工件质心 = (1.180, 0.080, 0.646)，工具 z 轴 = (-0.825, 0, 0.565)。
"""

from __future__ import annotations

import numpy as np

import kin
from env import PlanEnv
from real_scene import RealWorkpiece

# 标定值（工件局部坐标，相对真实质心）
APPROACH_LOCAL = np.array([-0.550, 0.080, -0.091])
R_TOOL = np.array([[0.564643, 0.0, -0.825335],
                   [0.0, 1.0, 0.0],
                   [0.825335, 0.0, 0.564643]])


class WorkpieceEnv(PlanEnv):
    """工件位置/朝向随机的接近任务（真实几何）。"""

    # 实测：标称接近姿态下前臂离工件只有 16 mm，工件能动的范围本来就很小。
    # ±10 cm 会直接穿模（不是代码问题，是这台臂在这个姿态没有余量）。
    XY_RANGE = 0.03
    Z_RANGE = 0.01
    YAW_RANGE = np.deg2rad(8.0)

    def __init__(self, seed=None, difficulty=1.0, horizon=300,
                 bank="scene_bank.npz",
                 scene="/home/xz/rosim_ws/.scratch/scene.json"):
        super().__init__(seed=seed, difficulty=difficulty, horizon=horizon)
        self.part = RealWorkpiece(scene)
        self.centre0 = self.part.centre0
        data = np.load(bank)
        self._bank_poses = data["poses"]
        self._bank_goals = data["goals"]
        # 由真实调用日志生成的库会带 starts：直接用真实起点，分布才对
        self._bank_starts = data["starts"] if "starts" in data else None

    # ------------------------------------------------------------------ scene
    def _sample_pose(self):
        d = self.difficulty
        if self.rng.uniform() > d:
            return self.centre0.copy(), 0.0
        centre = self.centre0 + self.rng.uniform(-1, 1, 3) * \
            [self.XY_RANGE, self.XY_RANGE, self.Z_RANGE]
        yaw = self.rng.uniform(-1, 1) * self.YAW_RANGE
        return centre, float(yaw)

    def _bank_goal(self, centre, yaw):
        scale = np.array([self.XY_RANGE, self.XY_RANGE, self.Z_RANGE, self.YAW_RANGE])
        delta = (self._bank_poses - np.array([*centre, yaw])) / scale
        return self._bank_goals[int(np.argmin((delta ** 2).sum(axis=1)))].copy()

    def _bank_sample(self):
        """有 starts 的库：随机取一条真实 (start, goal) 对。"""
        i = int(self.rng.integers(len(self._bank_goals)))
        return self._bank_starts[i].copy(), self._bank_goals[i].copy()

    def approach_pose(self, centre, yaw):
        """接近位姿 (R, t)，随工件平移和旋转。"""
        c, s = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return Rz @ R_TOOL, Rz @ APPROACH_LOCAL + centre

    # ------------------------------------------------------------------ reset
    def reset(self) -> np.ndarray:
        for _ in range(20):
            centre, yaw = self._sample_pose()
            self.obstacles = self.part.spheres(centre, yaw)
            if self._bank_starts is not None:
                start, goal = self._bank_sample()
                self.goal = goal
                self.q = start
                self.steps = 0
                self.prev = float(np.linalg.norm(self.q - self.goal))
                return self._obs()

            goal = self._bank_goal(centre, yaw)
            pts, _ = kin.forward(goal)
            if kin.min_clearance(pts, self.obstacles) < 0.02:
                continue
            self.goal = goal

            # 起点：离目标 TCP 约 0.55 m 的自由构型（真实接近段约 0.6 m）
            _, goal_tcp = kin.forward(goal)
            best, best_score = None, 1e9
            for _ in range(8):
                q = self._sample_free(0.7)
                _, tcp = kin.forward(q)
                score = abs(float(np.linalg.norm(tcp - goal_tcp)) - 0.55)
                if score < best_score:
                    best, best_score = q, score
            if best is not None:
                self.q = best
                self.steps = 0
                self.prev = float(np.linalg.norm(self.q - self.goal))
                return self._obs()

        self.obstacles = self.part.spheres(self.centre0, 0.0)
        self.q = kin.Q_HOME.copy()
        self.goal = np.clip(kin.Q_HOME + 0.4, kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        self.steps = 0
        self.prev = float(np.linalg.norm(self.q - self.goal))
        return self._obs()
