"""工件位置随机化的接近环境。

和 ``env.PlanEnv`` 的差别只有 reset()：目标不再是"随机关节构型"，而是
**焊枪在焊缝起点上方 10 cm、朝下的接近位姿**，而且这个位姿随工件一起移动。

为什么这样改：真实调用里策略失败（0/3、0.89 rad 也到不了），因为训练目标是
随机关节角，部署目标是"贴着工件的接近位姿"，分布完全不同。

工件位姿随机化范围（可按需调）：
    x/y ±0.12 m、z ±0.04 m、绕 z 轴 ±30°
球近似仍然只用于训练；执行时的碰撞判定仍是 MoveIt/FCL。
"""

from __future__ import annotations

import numpy as np

import kin
from env import PlanEnv, WORKPIECE_CENTRE, WORKPIECE_HALF, SPHERE_RADIUS

# 接近位姿（工件局部坐标），由一次真实调用的 goal 标定得到：
#   真实 goal TCP = (0.63, 0.16, 0.555)，工具 z 轴 = (-0.825, 0, 0.565)
#   标称工件中心 = (0.540, 0.450, 0.600)
# 这样"平移/旋转工件"时，接近位姿跟着工件一起动，和真实单元里挪动工件的效果一致。
APPROACH_LOCAL = np.array([0.090, -0.290, -0.045])
_TOOL_Z = np.array([-0.825, 0.0, 0.565])


def _frame_from_z(z_axis):
    """构造一个 z 轴沿给定方向的旋转矩阵（x 轴任取正交方向）。"""
    z = z_axis / np.linalg.norm(z_axis)
    tmp = np.array([0.0, 1.0, 0.0]) if abs(z[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(tmp, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class WorkpieceEnv(PlanEnv):
    """工件位置/朝向随机的接近任务。"""

    # 随机化幅度：以 difficulty 缩放，0 时固定在标称位置
    XY_RANGE = 0.12
    Z_RANGE = 0.04
    YAW_RANGE = np.deg2rad(30.0)

    def __init__(self, seed=None, difficulty=1.0, horizon=300, bank="scene_bank.npz"):
        super().__init__(seed=seed, difficulty=difficulty, horizon=horizon)
        data = np.load(bank)
        self._bank_poses = data["poses"]   # [x, y, z, yaw]
        self._bank_goals = data["goals"]   # 对应的接近关节构型
        self._bank_var = self._bank_poses.std(axis=0) + 1e-9

    def _sample_pose(self):
        """按难度缩放：d=0 用标称位姿，d=1 用库里的真实位姿。"""
        d = self.difficulty
        if self.rng.uniform() > d:
            return WORKPIECE_CENTRE.copy(), 0.0
        i = int(self.rng.integers(len(self._bank_poses)))
        pose = self._bank_poses[i]
        return pose[:3].copy(), float(pose[3])

    def _bank_goal(self, centre, yaw):
        """库里找最接近该位姿的样本，返回其关节构型（查表，不做 IK）。"""
        scale = np.array([self.XY_RANGE, self.XY_RANGE, self.Z_RANGE, self.YAW_RANGE])
        delta = (self._bank_poses - np.array([*centre, yaw])) / scale
        return self._bank_goals[int(np.argmin((delta ** 2).sum(axis=1)))].copy()

    def _spheres_for(self, centre, yaw):
        """把标称包围盒按 (centre, yaw) 变换后填充球。"""
        base = kin.box_spheres(WORKPIECE_CENTRE, WORKPIECE_HALF, SPHERE_RADIUS)
        local = base[:, :3] - WORKPIECE_CENTRE
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        out = base.copy()
        out[:, :3] = local @ R.T + centre
        return out

    def _approach_goal(self, centre, yaw):
        """接近位姿 → IK 关节构型；失败返回 None。

        位姿 = 工件变换作用在标定好的局部接近位姿上（位置和姿态都转）。
        """
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        p = R @ APPROACH_LOCAL + centre
        R_tool = R @ _frame_from_z(_TOOL_Z)

        for _ in range(6):
            seed = np.clip(self.rng.uniform(-1.2, 1.2, 6) + kin.Q_HOME,
                           kin.LIMITS[:, 0], kin.LIMITS[:, 1])
            q, err = kin.ik(R_tool, p, seed, iters=60)
            if err > 2e-3:
                continue
            pts, _ = kin.forward(q)
            if kin.min_clearance(pts, self.obstacles) < 0.02:
                continue
            return q
        return None

    def reset(self) -> np.ndarray:
        for _ in range(12):
            centre, yaw = self._sample_pose()
            self.obstacles = self._spheres_for(centre, yaw)
            goal = self._bank_goal(centre, yaw)
            pts, _ = kin.forward(goal)
            if kin.min_clearance(pts, self.obstacles) < 0.02:
                continue
            self.goal = goal

            # 起点：离目标 TCP 0.15–0.5 m 的自由构型（模拟上一段的结束位姿）
            _, goal_tcp = kin.forward(goal)
            best, best_d = None, 0.0
            for _ in range(6):
                q = self._sample_free(0.6)
                _, tcp = kin.forward(q)
                d = float(np.linalg.norm(tcp - goal_tcp))
                if 0.15 < d < 0.50:
                    best, best_d = q, d
                    break
                if abs(d - 0.32) < abs(best_d - 0.32):
                    best, best_d = q, d
            if best is not None:
                self.q = best
                self.steps = 0
                self.prev = float(np.linalg.norm(self.q - self.goal))
                return self._obs()
        # 极端情况下退化为标称场景
        self.obstacles = self._spheres_for(WORKPIECE_CENTRE, 0.0)
        self.q = kin.Q_HOME.copy()
        self.goal = np.clip(kin.Q_HOME + 0.4, kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        self.steps = 0
        self.prev = float(np.linalg.norm(self.q - self.goal))
        return self._obs()
