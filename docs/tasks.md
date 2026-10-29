# 任务队列（跨机）

## 待执行（主力机）

### 1. blocked 场景的判定口径（进行中，已回退到干净状态）
- **现状**：`scripts/benchmark_suite.py` 的 `blocked` 组用几何判据
  （起点 TCP → 目标 TCP 的直线须穿过工件球 `part.radius + 0.05`）。
  但工件半径只有 **1.1 cm**，离线扫描通过率约 **0.01%**（tol=0.05/norm≤1.2），
  实际跑出来 **0 个目标** —— 所以 `ompl` vs `hybrid` 的绕障对比**还没做成**。
- **试过但无效**：采样前先 `go_home()`（真正失败原因不是起点太远）。
- **下一步（未验证）**：把 blocked 改成**按结果定义**——
  候选目标先用 `line` 规划，失败的再用 `ompl` 验证，OMPL 能解才算 blocked。
  这样语义就是"必须绕行"，比几何启发式可靠。
  （今晚写好了 `build_blocked_by_outcome()` 但没跑通就回退了。）
- **判据**：能稳定生成 6 个 blocked 目标，再跑五规划器套件。

### 2. 绕障对比：ompl vs hybrid（依赖 #1）
- 目标：看 RL 初解能否减少 OMPL 的求解时间/失败率。
- 若 hybrid 在 blocked 上也不占优 → 承认当前策略无实用价值，
  转向"重训策略"或"专注 OMPL 线"。

### 3. 主力机版本锁定（用户执行，我代做不了）
```bash
sudo apt-mark hold ros-humble-ros2-control ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster \
  ros-humble-geometric-shapes ros-humble-rclcpp
```

## 已完成

- **2026-10-29 修 H3 闸门两处 bug**（`joint_delta` 按名匹配 + `lastNote()` 可观测）
- **2026-10-29 真实接近目标评测**：9 个样本，arm 位移中位数 3.3 rad，
  8/9 被闸门拦截，策略 **0/9 到达** → 见 `docs/rl_bench.md`
- **2026-10-29 服务单元化**：`rosim-bench` / `rosim-rl-policy`（`scripts/systemd/`）
- **2026-10-29 采集器**：`EdgeFollower` 写 `.scratch/approach_targets.jsonl`，
  `scripts/eval_approach_rl.py` 回放评测
