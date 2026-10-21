"""RL 路径规划环境（方案 A）。

契约与项目 ``rosim_rl::RlPlanner`` 一致，这是 B 方案里最关键的修正：

    obs 22 = q/3 (6) + goal_q/3 (6) + (goal_q-q)/3 (6) + tcp (3) + clearance (1)
    act  6 = Δq, 单位 rad，|Δq| <= 0.1
    goal   关节构型（和 MoveIt 的规划请求一致），不是 TCP 点
    障碍   工件包围盒用球近似（训练侧），MoveIt/FCL 才是执行时的判定

为什么不像 demo 那样用 TCP 目标：项目里 approach/retreat 的规划请求给的是
关节目标；用关节空间建模，策略输出可以直接喂给 RobotTrajectory。
"""

from __future__ import annotations

import numpy as np

import kin

MAX_DQ = 0.1          # rad per step
TOL = 0.10            # rad, 认为到达
HORIZON = 300

# 工件包围盒（世界系）。x/y 覆盖焊缝两端 0.44..0.64 / 0.28..0.62，z 上下各 8 cm。
WORKPIECE_CENTRE = np.array([0.540, 0.450, 0.600])
WORKPIECE_HALF = np.array([0.110, 0.170, 0.080])
SPHERE_RADIUS = 0.05


class PlanEnv:
    def __init__(self, seed: int | None = None, difficulty: float = 1.0,
                 horizon: int = HORIZON):
        self.rng = np.random.default_rng(seed)
        self.horizon = horizon
        self.difficulty = float(np.clip(difficulty, 0.0, 1.0))
        self.q = kin.Q_HOME.copy()
        self.goal = kin.Q_HOME.copy()
        self.obstacles = np.zeros((0, 4))
        self.steps = 0
        self.prev = 0.0

    # ---------------------------------------------------------------- helpers
    def _spheres(self) -> np.ndarray:
        # 位姿随机化：位置 ±3 cm、尺寸 ±10%，让策略不要死记一个盒子
        jitter = self.rng.uniform(-0.03, 0.03, 3) * self.difficulty
        scale = 1.0 + self.rng.uniform(-0.1, 0.1) * self.difficulty
        centre = WORKPIECE_CENTRE + jitter
        half = WORKPIECE_HALF * scale
        radius = SPHERE_RADIUS * (1.0 + self.rng.uniform(-0.15, 0.15) * self.difficulty)
        return kin.box_spheres(centre, half, radius)

    def _obs(self) -> np.ndarray:
        pts, tcp = kin.forward(self.q)
        clearance = kin.min_clearance(pts, self.obstacles)
        parts = [
            self.q / 3.0,
            self.goal / 3.0,
            self._delta() / 3.0,
            tcp,
            [float(np.clip(clearance, -0.2, 1.0))],
        ]
        return np.concatenate(parts).astype(np.float32)

    @property
    def obs_dim(self) -> int:
        return 25

    @property
    def act_dim(self) -> int:
        return 7

    # 导轨和关节的量纲不同：1 m 导轨大致等价 2 rad 关节，距离里按这个折算
    DIST_WEIGHT = np.array([2.0, 1, 1, 1, 1, 1, 1])

    def _delta(self) -> np.ndarray:
        return self.goal - self.q

    def _dist(self) -> float:
        return float(np.linalg.norm(self._delta() * self.DIST_WEIGHT))

    # ------------------------------------------------------------------- gym
    def reset(self) -> np.ndarray:
        self.obstacles = self._spheres()

        # 起点：home 附近；目标：远离起点、且目标构型本身不碰撞
        self.q = self._sample_free(0.5)
        goal_spread = 0.50 + 0.50 * self.difficulty
        for _ in range(200):
            goal = self._sample_free(goal_spread)
            if float(np.linalg.norm((goal - self.q) * self.DIST_WEIGHT)) > 0.5:
                self.goal = goal
                break
        else:
            self.goal = np.clip(kin.Q_HOME + 0.8, kin.LIMITS[:, 0], kin.LIMITS[:, 1])

        self.steps = 0
        self.prev = self._dist()
        return self._obs()

    def _sample_free(self, spread: float) -> np.ndarray:
        for _ in range(200):
            scale = np.array([0.6, 1, 1, 1, 1, 1, 1])     # 导轨用米，幅度小一些
            q = kin.Q_HOME + self.rng.uniform(-spread, spread, 7) * scale * 0.5
            q = np.clip(q, kin.LIMITS[:, 0], kin.LIMITS[:, 1])
            pts, _ = kin.forward(q)
            if kin.min_clearance(pts, self.obstacles) > 0.05:
                return q
        return np.clip(kin.Q_HOME + self.rng.uniform(-0.2, 0.2, 7),
                       kin.LIMITS[:, 0], kin.LIMITS[:, 1])

    def step(self, action: np.ndarray):
        a = np.clip(action, -1.0, 1.0) * kin.MAX_STEP
        self.q = np.clip(self.q + a, kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        pts, tcp = kin.forward(self.q)
        dist = self._dist()
        clearance = kin.min_clearance(pts, self.obstacles)
        self.steps += 1

        collided = clearance < 0.0
        reached = dist < TOL

        # 与 demo 相同的奖励结构：进度为主、距离为辅、碰撞重罚。
        reward = -0.1 * dist + 2.0 * (self.prev - dist) - 0.01
        margin = 0.05
        if clearance < margin:
            reward -= 2.0 * (margin - max(clearance, 0.0)) / margin
        if collided:
            reward -= 10.0
        if reached:
            reward += 15.0
        self.prev = dist

        done = collided or reached or self.steps >= self.horizon
        info = {"dist": dist, "clearance": clearance, "collided": collided,
                "reached": reached}
        return self._obs(), float(reward), bool(done), info

    # --------------------------------------------------------------- rollout
    def rollout(self, policy, max_steps: int | None = None) -> dict:
        obs = self.reset()
        frames, min_clear = [self.q.copy()], float("inf")
        reached = collided = False
        for _ in range(max_steps or self.horizon):
            action = policy(obs)
            obs, _, done, info = self.step(action)
            frames.append(self.q.copy())
            min_clear = min(min_clear, info["clearance"])
            if info["reached"]:
                reached = True
            if info["collided"]:
                collided = True
            if done:
                break
        return {"path": np.array(frames), "reached": reached, "collided": collided,
                "steps": len(frames) - 1, "min_clearance": min_clear,
                "goal": self.goal.copy(), "obstacles": self.obstacles.copy()}
