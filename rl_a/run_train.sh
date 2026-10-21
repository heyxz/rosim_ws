#!/usr/bin/env bash
# 一键训练：PyTorch + stable-baselines3
#   ./run_train.sh                      # 默认 150 万步，8 个并行环境
#   ./run_train.sh --timesteps 3000000  # 更多步数
#   ./run_train.sh --device cpu         # 强制 CPU
set -e
cd "$(dirname "$0")"
export PYTHONPATH="${RL_PYLIBS:-/home/xz/rosim_ws/.pylibs}:$PWD:${PYTHONPATH}"
export MPLCONFIGDIR="$PWD/.mpl"
mkdir -p "$MPLCONFIGDIR"

python3 - <<'PY'
import torch
print(f"torch {torch.__version__}  cuda_available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))
PY

if [ $# -eq 0 ]; then
  set -- --timesteps 2000000 --envs 8 --out out_a
fi
exec python3 train_sb3.py "$@"
