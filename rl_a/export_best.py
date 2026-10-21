"""Pick the better of final_model / best_model with identical normalisation stats,
then export ONNX + policy.json from it.

The training script exported whatever `best_model` happened to be, but that
checkpoint was selected on 10-episode evals (early lucky runs set an unreachable
bar), so it could be far worse than the final model. Selecting here, after
training, is the only reliable point.
"""
import json, sys
import numpy as np

import env as env_mod
import scene_env as scene_mod
from stable_baselines3 import PPO

OUT = sys.argv[1] if len(sys.argv) > 1 else "out_a2"
EPISODES = int(sys.argv[2]) if len(sys.argv) > 2 else 40
ENV_NAME = sys.argv[3] if len(sys.argv) > 3 else "plan"
ENV_CLS = scene_mod.WorkpieceEnv if ENV_NAME == "scene" else env_mod.PlanEnv

with open(f"{OUT}/policy.json") as f:
    meta = json.load(f)
mean = np.asarray(meta["obs_mean"], np.float32)
var = np.asarray(meta["obs_var"], np.float32)
clip = float(meta.get("clip_obs", 5.0))


def normalize(obs):
    x = (np.asarray(obs, np.float32) - mean) / np.sqrt(var + 1e-8)
    return np.clip(x, -clip, clip)


def evaluate(model_path):
    model = PPO.load(model_path, device="cpu")
    ev = ENV_CLS(seed=7777, difficulty=1.0)
    ok = coll = 0
    for _ in range(EPISODES):
        obs = ev.reset()
        done = False
        while not done:
            a, _ = model.predict(normalize(obs)[None, :], deterministic=True)
            obs, _, done, info = ev.step(a[0])
        ok += int(info["reached"])
        coll += int(info["collided"])
    return ok / EPISODES, coll / EPISODES, model


best = None
for name in ("final_model", "best_model"):
    try:
        s, c, model = evaluate(f"{OUT}/{name}")
    except Exception as e:
        print(f"{name}: 无法评估 ({e})")
        continue
    print(f"{name:12s} reached={s:.1%} collided={c:.1%}")
    if best is None or s > best[0]:
        best = (s, c, name, model)

success, collided, name, model = best
print(f"选择: {name}  (reached {success:.1%})")

import torch

class Deterministic(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs):
        actions, _, _ = self.policy(obs, deterministic=True)
        return torch.clamp(actions, -1.0, 1.0)


wrapper = Deterministic(model.policy).eval()
dummy = torch.zeros(1, model.observation_space.shape[0])
torch.onnx.export(wrapper, dummy, f"{OUT}/policy.onnx", input_names=["obs"],
                  output_names=["action"], opset_version=17,
                  dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}})
meta["eval_success"] = success
meta["source"] = name
with open(f"{OUT}/policy.json", "w") as f:
    json.dump(meta, f, indent=1)
print(f"已导出 {OUT}/policy.onnx（来源 {name}）")
