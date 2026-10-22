"""工件可放置区域（全部相对机器人底座 base_link）。

用户定义（2026-10-22）：
  x  −1.40 … +1.40   （导轨全长 2.80 m）
  y  −1.50 … −0.50   和  +0.50 … +1.50   （机器人正前方 ±0.5 m 留空）
  z  −0.20 … +0.80
  yaw 任意（工件绕竖直轴旋转）

这个模块只负责"在区域里采样"和"粗筛可达性"，不依赖 ROS。
"""

from __future__ import annotations

import numpy as np

from kin import BASE_OFFSET

X_RANGE = (-1.40, 1.40)
Y_BANDS = ((-1.50, -0.50), (0.50, 1.50))
Z_RANGE = (-0.20, 0.80)

# 臂展（GP12 约 1.40 m，从底座算）。粗筛用，真正的判据是 IK 有解。
REACH = 1.40
# 焊接时工具还要从上方压下来，姿态约束会再吃掉一部分半径。
REACH_WELD = 1.05


def base_origin() -> np.ndarray:
    """底座在 base_link 系里的位置（z 抬高 BASE_OFFSET）。"""
    return np.array([0.0, 0.0, float(BASE_OFFSET[2])])


def sample(rng, n: int) -> np.ndarray:
    """均匀采样 n 个位姿：返回 (n, 4) = x, y, z, yaw。

    y 在两个带里各占一半概率，避免正前方空档被误采。
    """
    out = np.zeros((n, 4))
    for i in range(n):
        out[i, 0] = rng.uniform(*X_RANGE)
        band = Y_BANDS[int(rng.integers(len(Y_BANDS)))]
        out[i, 1] = rng.uniform(*band)
        out[i, 2] = rng.uniform(*Z_RANGE)
        out[i, 3] = rng.uniform(-np.pi, np.pi)
    return out


def distance_to_base(poses: np.ndarray) -> np.ndarray:
    """工件中心到底座的距离（只看位置）。"""
    o = base_origin()
    return np.linalg.norm(poses[:, :3] - o[None, :], axis=1)


def reachable(poses: np.ndarray, limit: float = REACH) -> np.ndarray:
    return distance_to_base(poses) <= limit
