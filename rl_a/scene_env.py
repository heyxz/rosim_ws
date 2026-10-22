"""工件位置随机化 + 真实几何的接近环境。

两个关键点（都是踩坑换来的）：
1. 目标不是随机关节角，而是**焊枪在焊缝前的接近位姿**，随工件一起变换；
2. 障碍用 ``/planning_scene`` 导出的**真实网格**（FCL 用的同一份），不是猜的盒子。

接近位姿由真实调用标定：goal 的 TCP = (0.630, 0.160, 0.555)，
真实工件质心 = (1.180, 0.080, 0.646)，工具 z 轴 = (-0.825, 0, 0.565)。
"""

from __future__ import annotations

import os

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
                 bank=None, bank_path=None, bank_ratio=0.5, blocked_ratio=0.0,
                 scene=os.path.join(os.path.dirname(__file__), "scene.json")):
        super().__init__(seed=seed, difficulty=difficulty, horizon=horizon)
        bank = bank_path or bank or os.path.join(os.path.dirname(__file__), "scene_bank.npz")
        # 直线被工件挡住的场景占比：不训练这类场景，策略只学会"直奔目标"
        # （实测：30 个挡路场景下 0/30 到达、一半以上直接撞上去）
        self.blocked_ratio = float(blocked_ratio)
        self._blocked_pool = None
        self._pool_obstacles = None
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

    def _ensure_blocked_pool(self, n: int = 60):
        """挡路场景池：优先读盘上的 blocked_bank.npz（构建一次要两分钟，8 个
        worker 各建一次太浪费）。"""
        if self._blocked_pool is not None:
            return
        cache = os.path.join(os.path.dirname(__file__), "blocked_bank.npz")
        if os.path.exists(cache):
            data = np.load(cache)
            self._blocked_pool = [(data["starts"][i], data["goals"][i])
                                  for i in range(len(data["starts"]))]
            self._pool_obstacles = data["obstacles"]
            self.obstacles = self._pool_obstacles
            return
        self._pool_obstacles = self.part.spheres(self.centre0, 0.0)
        self.obstacles = self._pool_obstacles      # _blocked_pair 依赖这个
        pool = []
        while len(pool) < n:
            pair = self._blocked_pair()
            if pair is not None:
                pool.append(pair)
        self._blocked_pool = pool

    def _blocked_pair(self):
        """两端无碰撞、但关节直线穿过工件的 (start, goal)。

        这是"必须绕行"的最小可学样本：直线规划器必撞，策略必须学会侧移。
        """
        import kin as _kin
        for _ in range(3000):
            q0 = np.clip(_kin.Q_HOME + self.rng.uniform(-1.0, 1.0, 7),
                         _kin.LIMITS[:, 0], _kin.LIMITS[:, 1])
            q1 = np.clip(_kin.Q_HOME + self.rng.uniform(-1.0, 1.0, 7),
                         _kin.LIMITS[:, 0], _kin.LIMITS[:, 1])
            d = float(np.linalg.norm(q1 - q0))
            if not (0.3 <= d <= 1.2):
                continue
            p0, p1 = _kin.forward(q0)[1], _kin.forward(q1)[1]
            c = self.centre0
            seg = p1 - p0
            den = float(seg @ seg)
            t = 0.0 if den < 1e-12 else float(np.clip((c - p0) @ seg / den, 0.0, 1.0))
            if float(np.linalg.norm(c - (p0 + t * seg))) > self.part.radius + 0.05:
                continue                      # 直线没挡路
            if _kin.min_clearance(_kin.forward(q0)[0], self.obstacles) < 0.03:
                continue
            if _kin.min_clearance(_kin.forward(q1)[0], self.obstacles) < 0.03:
                continue
            # 关节直线必须真的会撞（否则 line 能过，学不到东西）
            worst = min(_kin.min_clearance(_kin.forward(q0 + s * (q1 - q0))[0], self.obstacles)
                        for s in np.linspace(0.0, 1.0, 25))
            if worst > 0.0:
                continue
            return q0, q1
        return None

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
            # 挡路场景优先（占比 blocked_ratio）：不练这类样本，策略只学会直奔目标。
            # 现场采样命中率极低（几千次才有一条），所以预生成一个池子反复用。
            if self.blocked_ratio > 0.0 and self.rng.random() < self.blocked_ratio:
                self._ensure_blocked_pool()
                self.obstacles = self._pool_obstacles
                q0, q1 = self._blocked_pool[int(self.rng.integers(len(self._blocked_pool)))]
                self.q, self.goal = q0.copy(), q1.copy()
                self.steps = 0
                self.prev = float(np.linalg.norm(self.q - self.goal))
                return self._obs()

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
