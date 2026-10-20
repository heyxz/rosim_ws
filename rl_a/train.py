"""方案 A：关节目标 + 工件障碍的路径规划策略（sb3 PPO）。

    PYTHONPATH=.pylibs python3 train_sb3.py --timesteps 1500000 --envs 8

Compared with the numpy version (train.py):
  * PPO comes from sb3, so the network / optimiser / GAE are battle-tested;
  * N envs step in parallel processes (SubprocVecEnv) instead of a loop;
  * VecNormalize handles observation & reward scaling, saved with the model;
  * TensorBoard logs + a CSV eval curve + best-checkpoint tracking;
  * exports ONNX and a normalisation JSON so the C++ side can infer without torch.

The environment is unchanged (env.py): the policy sees joint angles, TCP,
target offset and per-obstacle geometry, and outputs 6 joint velocity deltas.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

import env as env_mod
import kin

# TensorBoard is optional: the eval CSV is enough to plot the curve, and a
# broken protobuf/numpy combination should not stop a training run.
try:
    # sb3 writes through torch's SummaryWriter, which needs a protobuf version
    # that agrees with the installed tensorboard. Probe the real path.
    from torch.utils.tensorboard import SummaryWriter  # noqa: F401
    TB_LOG = True
except Exception as exc:  # pragma: no cover - environment dependent
    print(f"tensorboard unavailable ({exc}); logging to eval.csv only")
    TB_LOG = False


MAX_DQ = env_mod.MAX_DQ


class ArmGym(gym.Env):
    """Gymnasium adapter around ArmEnv (env.py itself stays dependency-free)."""

    metadata = {"render_modes": []}

    def __init__(self, seed: int = 0, difficulty: float = 0.0):
        self.env = env_mod.PlanEnv(seed=seed, difficulty=difficulty)
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.env.obs_dim,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (self.env.act_dim,), np.float32)

    def set_difficulty(self, difficulty: float):
        self.env.difficulty = float(np.clip(difficulty, 0.0, 1.0))

    def reset(self, *, seed=None, options=None):
        return self.env.reset(), {}

    def step(self, action):
        obs, reward, done, info = self.env.step(np.asarray(action, dtype=np.float64))
        return obs, float(reward), bool(done), False, info


def make_env(rank: int, seed: int, difficulty: float):
    def _init():
        return Monitor(ArmGym(seed=seed + rank, difficulty=difficulty))
    return _init


class Curriculum(BaseCallback):
    """Raise difficulty while the policy keeps solving the current level."""

    def __init__(self, vec_env, out_dir: str, every: int = 20_000, episodes: int = 10):
        super().__init__()
        self.vec_env = vec_env
        self.out_dir = out_dir
        self.every = every
        self.episodes = episodes
        self.difficulty = 0.0
        self.best = -1.0
        self.next_check = every
        self.rows = []
        self._t_prev = None
        self.eval_env = env_mod.PlanEnv(seed=99991, difficulty=0.0)
        self.hard_env = env_mod.PlanEnv(seed=99992, difficulty=1.0)

    def _on_step(self) -> bool:
        if self.num_timesteps < self.next_check:
            return True
        self.next_check += self.every

        # Advance the curriculum on the *current* level; also record how the
        # policy does on the full task, which is what actually matters.
        succ = self._evaluate(self.eval_env, self.difficulty)
        succ_hard = self._evaluate(self.hard_env, 1.0)
        if succ_hard > self.best:
            self.best = succ_hard
            self.model.save(os.path.join(self.out_dir, "best_model"))
        if succ > 0.65:
            self.difficulty = min(1.0, self.difficulty + 0.04)
            self.vec_env.env_method("set_difficulty", self.difficulty)

        self.rows.append((self.num_timesteps, succ, succ_hard, self.difficulty))
        with open(os.path.join(self.out_dir, "eval.csv"), "w") as f:
            f.write("timesteps,level_success,hard_success,difficulty\n")
            for t, s, h, d in self.rows:
                f.write(f"{t},{s:.4f},{h:.4f},{d:.3f}\n")
        import time as _time
        now = _time.time()
        rate = ""
        if self._t_prev is not None:
            rate = f"  ({self.every / (now - self._t_prev):,.0f} steps/s)"
        self._t_prev = now
        print(f"[eval] step {self.num_timesteps:>8d}  level {succ:5.1%}  "
              f"hard {succ_hard:5.1%}  difficulty {self.difficulty:4.2f}{rate}", flush=True)
        return True

    def _evaluate(self, ev, difficulty: float) -> float:
        ev.difficulty = difficulty
        ok = 0
        for _ in range(self.episodes):
            obs = ev.reset()
            for _ in range(ev.horizon):
                # 策略看到的是 VecNormalize 归一化后的观测，评估时也必须一致
                obs_n = self.vec_env.normalize_obs(obs[None, :])[0]
                action, _ = self.model.predict(obs_n, deterministic=True)
                obs, _, done, info = ev.step(action)
                if done:
                    ok += int(info["reached"])
                    break
        return ok / self.episodes


def export_onnx(model, out_path: str):
    """Deterministic policy: obs -> action in [-1, 1]."""
    import torch

    class Deterministic(torch.nn.Module):
        def __init__(self, policy):
            super().__init__()
            self.policy = policy

        def forward(self, obs):
            actions, _, _ = self.policy(obs, deterministic=True)
            return torch.clamp(actions, -1.0, 1.0)   # 与环境里的裁剪一致

    wrapper = Deterministic(model.policy).eval()
    dummy = torch.zeros(1, model.observation_space.shape[0])
    torch.onnx.export(wrapper, dummy, out_path, input_names=["obs"],
                      output_names=["action"], opset_version=17,
                      dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}})
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=1_500_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--out", default="out_sb3")
    ap.add_argument("--device", default="auto", help="auto / cpu / cuda")
    ap.add_argument("--serial", action="store_true", help="DummyVecEnv instead of subprocesses")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print(f"device={args.device} envs={args.envs} timesteps={args.timesteps}")

    fns = [make_env(i, args.seed, 0.0) for i in range(args.envs)]
    venv = DummyVecEnv(fns) if args.serial else SubprocVecEnv(fns)
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=5.0)

    model = PPO(
        "MlpPolicy", venv, verbose=1, device=args.device,
        n_steps=256, batch_size=512, n_epochs=6, gamma=0.99, gae_lambda=0.95,
        clip_range=0.2, ent_coef=0.005, learning_rate=2e-3,
        # 标准对角高斯策略（不用 gSDE）：gSDE 在 20 维观测上探索效率很差，
        # 30 万步都学不会无障碍到达。动作由环境裁到 [-1,1]，ONNX 导出时同样裁。
        policy_kwargs=dict(net_arch=[128, 128]),
        tensorboard_log=os.path.join(args.out, "tb") if TB_LOG else None,
    )
    print("torch device in use:", model.device)

    cb = Curriculum(venv, args.out)
    model.learn(total_timesteps=args.timesteps, callback=cb)
    model.save(os.path.join(args.out, "final_model"))
    venv.save(os.path.join(args.out, "vecnormalize.pkl"))

    # ---- evaluate the best checkpoint at full difficulty -------------------
    best_path = os.path.join(args.out, "best_model")
    if os.path.exists(best_path + ".zip"):
        model = PPO.load(best_path, device="cpu")
        print("using best checkpoint:", best_path)
    vecnorm = VecNormalize.load(os.path.join(args.out, "vecnormalize.pkl"), venv)
    vecnorm.training = False

    ev = env_mod.PlanEnv(seed=4242, difficulty=1.0)
    ok = coll = 0
    best_rec = None
    for _ in range(40):
        obs = ev.reset()
        frames = [{"q": ev.q.tolist(), "tcp": kin.forward(ev.q)[1].tolist()}]
        done = False
        while not done:
            action, _ = model.predict(vecnorm.normalize_obs(obs[None, :])[0], deterministic=True)
            obs, _, done, info = ev.step(action)
            frames.append({"q": ev.q.tolist(), "tcp": kin.forward(ev.q)[1].tolist(),
                           "clearance": info["clearance"]})
        ok += int(info["reached"])
        coll += int(info["collided"])
        if info["reached"]:
            rec = {"frames": frames, "target": kin.forward(ev.goal)[1].tolist(),
                   "goal_q": ev.goal.tolist(),
                   "obstacles": ev.obstacles.tolist(), "steps": len(frames) - 1,
                   "reached": True, "collided": False}
            if best_rec is None or len(rec["frames"]) > len(best_rec["frames"]):
                best_rec = rec
    print(f"eval (40 episodes, full difficulty): reached {ok/40:.1%}  collision {coll/40:.1%}")

    if best_rec is not None:
        with open(os.path.join(args.out, "rollout.json"), "w") as f:
            json.dump(best_rec, f)
        print("wrote", os.path.join(args.out, "rollout.json"))

    # ---- artifacts for the C++ planner -------------------------------------
    onnx_path = os.path.join(args.out, "policy.onnx")
    # Export happens in export_best.py: training's own `best_model` is chosen on
    # 10-episode evals (an early lucky run sets an unreachable bar), so the final
    # model can be far better. Choose between them with one evaluation pass.
    import subprocess
    subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), "export_best.py"),
                    args.out, "40"], check=False)
    meta = {
        "obs_dim": int(model.observation_space.shape[0]),
        "act_dim": int(model.action_space.shape[0]),
        "max_dq": MAX_DQ,
        "obs_mean": vecnorm.obs_rms.mean.tolist(),
        "obs_var": vecnorm.obs_rms.var.tolist(),
        "clip_obs": 5.0,
        "eval_success": ok / 40.0,
    }
    with open(os.path.join(args.out, "policy.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print("wrote", onnx_path, "and policy.json")


if __name__ == "__main__":
    main()
