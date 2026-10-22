"""构造"直线被工件挡住"的场景，评估 RL 是否真的会绕障。

之前所有评估的场景里，起点到目标的直线路径都不穿过工件，所以 100% 只说明
"能到达"，不说明避障。这里专门挑"两端无碰撞、但直线穿过工件"的场景：

  * 起点/目标都无碰撞（否则是谁都失败，没有意义）
  * 起点 TCP → 目标 TCP 的**直线**离工件表面 < 5 cm（直线规划器必然要撞）
  * 关节位移在 0.3~1.2 rad（低于 RL 的 1.5 rad 门限，保证 RL 有机会被调用）

用法:
    python3 rl_a/eval_blocked.py [--trials 40]
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import kin  # noqa: E402
import real_scene  # noqa: E402
import scene_env  # noqa: E402

MODELS = {
    "out_rail": "box + home±噪声",
    "out_bank": "box + 真实起点",
    "out_bank_scene": "真实几何 + 真实起点",
    "out_blocked": "加挡路场景 50%",
    "out_blocked2": "挡路场景 60% + 3M 步",
}

SCENE = os.path.join(HERE, "scene.json")
SEG_MIN = 0.3          # rad, 位移下界
SEG_MAX = 1.2          # rad, 位移上界（RL 门限 1.5）
BLOCK = 0.05           # m, TCP 直线离工件多近算"挡住"


def seg_point_distance(p0, p1, c):
    d = p1 - p0
    den = float(d @ d)
    t = 0.0 if den < 1e-12 else float(np.clip((c - p0) @ d / den, 0.0, 1.0))
    return float(np.linalg.norm(c - (p0 + t * d)))


def straight_clear_min(q0, q1, obstacles, n=40):
    """关节空间直线插值的最大碰撞程度（负值=撞）。line planner 的近似代理。"""
    worst = np.inf
    for t in np.linspace(0.0, 1.0, n):
        q = q0 + t * (q1 - q0)
        pts, _ = kin.forward(q)
        worst = min(worst, kin.min_clearance(pts, obstacles))
    return worst


def sample_scenario(rng, obstacles, centre, radius):
    for _ in range(4000):
        q0 = np.clip(kin.Q_HOME + rng.uniform(-1.0, 1.0, 7), kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        q1 = np.clip(kin.Q_HOME + rng.uniform(-1.0, 1.0, 7), kin.LIMITS[:, 0], kin.LIMITS[:, 1])
        if not (SEG_MIN <= np.linalg.norm(q1 - q0) <= SEG_MAX):
            continue
        p0, p1 = kin.forward(q0)[1], kin.forward(q1)[1]
        if seg_point_distance(p0, p1, centre) > radius + BLOCK:
            continue                      # 直线不经过工件 → 不是我们想要的场景
        pts0, _ = kin.forward(q0)
        pts1, _ = kin.forward(q1)
        if kin.min_clearance(pts0, obstacles) < 0.03 or kin.min_clearance(pts1, obstacles) < 0.03:
            continue                      # 端点自己就撞，谁都没戏
        if straight_clear_min(q0, q1, obstacles) > 0.0:
            continue                      # 关节直线其实不撞（line 能过），也不算
        return q0, q1
    return None


def load_policy(path):
    meta = json.load(open(os.path.join(path, "policy.json")))
    import onnxruntime as ort
    sess = ort.InferenceSession(os.path.join(path, "policy.onnx"),
                                providers=["CPUExecutionProvider"])
    return meta, sess, sess.get_inputs()[0].name


def rollout(policy, q0, q1, obstacles, horizon=300):
    meta, sess, inp = policy
    mean = np.asarray(meta["obs_mean"], np.float32)
    var = np.asarray(meta["obs_var"], np.float32)
    clip = float(meta.get("clip_obs", 5.0))
    ev = scene_env.WorkpieceEnv(seed=0, difficulty=1.0)
    ev.obstacles = obstacles
    ev.q = q0.copy()
    ev.goal = q1.copy()
    ev.steps = 0
    ev.prev = float(np.linalg.norm((ev.goal - ev.q) * getattr(ev, "DIST_WEIGHT", 1.0)))
    obs = ev._obs()
    min_clear = np.inf
    done = False
    while not done:
        x = (np.asarray(obs, np.float32) - mean) / np.sqrt(var + 1e-8)
        a = np.clip(sess.run(None, {inp: np.clip(x, -clip, clip)[None, :]})[0].ravel(), -1, 1)
        obs, _, done, info = ev.step(a)
        min_clear = min(min_clear, info["clearance"])
    return info["reached"], info["collided"], min_clear, ev.steps


def main() -> int:
    trials = int(sys.argv[sys.argv.index("--trials") + 1]) if "--trials" in sys.argv else 40
    part = real_scene.RealWorkpiece(SCENE)
    obstacles = part.spheres(part.centre0, 0.0)
    rng = np.random.default_rng(4242)

    scenarios = []
    while len(scenarios) < trials:
        s = sample_scenario(rng, obstacles, part.centre0, part.radius)
        if s:
            scenarios.append(s)
    print(f"构造 {len(scenarios)} 个场景：两端无碰撞、直线穿过工件、位移 0.3~1.2 rad")

    policies = {name: load_policy(os.path.join(HERE, name)) for name in MODELS}
    print(f"\n{'模型':<16}{'训练环境':<22}{'到达':>6}{'碰撞':>6}{'超时':>6}  {'最小间隙中位':>12}")
    for name, policy in policies.items():
        ok = coll = 0
        clears = []
        for q0, q1 in scenarios:
            reached, collided, mc, steps = rollout(policy, q0, q1, obstacles)
            ok += int(reached)
            coll += int(collided)
            clears.append(mc)
        timeout = len(scenarios) - ok - coll
        print(f"{name:<16}{MODELS[name]:<22}{ok:>5}  {coll:>5}  {timeout:>5}  "
              f"{np.median(clears) * 1000:>10.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
