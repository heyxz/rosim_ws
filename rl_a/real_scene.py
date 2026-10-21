"""真实碰撞几何：把 /planning_scene 导出的网格变成训练用的球集。

训练侧原来用一个"猜的盒子"（中心 0.54,0.45,0.6，半 0.11,0.17,0.08），
而真实工件在 (1.15, 0.08, 0.655) —— 位置完全不对，策略学的避障绕的是空气，
一上真实场景就撞。

这里改用 ``scripts/dump_scene.py`` 导出的真实网格（FCL 用的就是它），
在表面撒球近似，并按工件位姿变换。
"""

from __future__ import annotations

import json

import numpy as np


def load_mesh(path: str = ".scratch/scene.json"):
    """返回 (vertices [N,3], triangles [M,3])，世界系（工件在默认位姿时）。"""
    data = json.load(open(path))
    obj = data["objects"][0]
    return np.array(obj["vertices"], dtype=float), np.array(obj["triangles"], dtype=int)


def surface_spheres(V: np.ndarray, T: np.ndarray, spacing: float = 0.02):
    """在网格表面按网格间距采样球，返回 [K,4]（xyz + 半径）。

    用顶点 + 三角重心 + 边中点采样，再按 voxel 合并，避免重复球。
    """
    pts = [V]
    pts.append(V[T].mean(axis=1))                       # 重心
    pts.append((V[T[:, 0]] + V[T[:, 1]]) / 2)           # 边中点
    pts.append((V[T[:, 1]] + V[T[:, 2]]) / 2)
    pts.append((V[T[:, 2]] + V[T[:, 0]]) / 2)
    P = np.vstack(pts)

    # voxel 去重：每个 cell 只留一个点
    keys = np.floor(P / spacing).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    P = P[idx]

    out = np.zeros((len(P), 4))
    out[:, :3] = P
    out[:, 3] = spacing * 0.55          # 略小于半间距，避免整体"变胖"
    return out


class RealWorkpiece:
    """真实工件的球集，支持按 (centre, yaw) 变换。"""

    def __init__(self, scene_path: str = ".scratch/scene.json", spacing: float = 0.02):
        V, T = load_mesh(scene_path)
        self.centre0 = V.mean(axis=0)          # 标称质心
        spheres = surface_spheres(V, T, spacing)
        self.local = spheres[:, :3] - self.centre0   # 相对质心
        self.radius = float(spheres[0, 3])
        self.extent = V.max(axis=0) - V.min(axis=0)

    def spheres(self, centre: np.ndarray, yaw: float) -> np.ndarray:
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        out = np.zeros((len(self.local), 4))
        out[:, :3] = self.local @ R.T + centre
        out[:, 3] = self.radius
        return out
