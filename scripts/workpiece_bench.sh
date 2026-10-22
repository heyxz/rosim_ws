#!/bin/bash
# M1 基准：工件在指定区域内随机摆放，跑完整链路（导入 → 接近 → 沿缝 → 退刀）。
#
#   ./scripts/workpiece_bench.sh [位姿数] [规划器]
#
# 区域见 rl_a/region.py（x ±1.40 轨道全长 / y ±0.5~±1.5 / z −0.2~0.8 / 任意 yaw）。
# 结果：每个位姿一行 SUCCESS/FAIL + 耗时；末尾给出成功率。
set +u
N=${1:-8}
PLANNER=${2:-auto}
cd "$(dirname "$0")/.."
mkdir -p .scratch/roslog .scratch/roshome
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR=$PWD/.scratch/roslog ROS_HOME=$PWD/.scratch/roshome

if [ "$PLANNER" != "auto" ]; then
  ros2 service call /rosim_planning/set_planner rosim_interfaces/srv/SetPlanner \
    "{planner_type: $PLANNER}" >/dev/null 2>&1
fi

PYTHONPATH="$PWD/rl_a" python3 - "$N" <<'PY' > .scratch/bench_poses.txt
import sys
sys.path.insert(0, 'rl_a')
import numpy as np, region
n = int(sys.argv[1])
rng = np.random.default_rng(2027)
P = region.sample(rng, 4000)
r = np.clip(P[:, 0], -1.05, 1.05)
d = np.sqrt((P[:, 0] - r) ** 2 + P[:, 1] ** 2 + (P[:, 2] - 0.167) ** 2)
keep = P[d <= 1.05]                      # 焊接舒适区
idx = rng.choice(len(keep), min(n, len(keep)), replace=False)
for p in keep[idx]:
    print(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} {p[3]:.3f}")
PY

ok=0; total=0
while read -r x y z yaw; do
  total=$((total+1))
  mid=$(node scripts/ws_import.js "$x" "$y" "$z" "$yaw" 2>/dev/null | tail -1)
  if [ -z "$mid" ] || [ "$mid" = "import-timeout" ]; then
    echo "pose$total ($x,$y,$z,yaw=$yaw): IMPORT_FAIL"; continue
  fi
  t0=$(date +%s)
  res=$(timeout 150 python3 scripts/seam_once.py "$mid" 2>&1 | grep -E "^result:" | tail -1)
  dt=$(( $(date +%s) - t0 ))
  if echo "$res" | grep -q "success=True"; then
    ok=$((ok+1)); echo "pose$total ($x,$y,$z,yaw=$yaw): SUCCESS ${dt}s  [$mid]"
  else
    echo "pose$total ($x,$y,$z,yaw=$yaw): FAIL ${dt}s  ${res:-超时}  [$mid]"
  fi
done < .scratch/bench_poses.txt
echo "----"
echo "规划器=$PLANNER  成功 $ok/$total"
