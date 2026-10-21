# OMPL vs RL 规划器对比（2026-10-21）

方法：`scripts/rl_benchmark.py`，对同一批随机关节目标分别调用
`/rosim_planning/plan_joint`，目标 = HOME + U(-r, r)·6，剔除偏移 < 0.25 rad。

## 纯 RL（`ROSIM_RL_NO_FALLBACK=1`，无回退）

| 目标偏移半径 r | 偏移范数均值 | OMPL 成功 | RL 成功 | OMPL 中位耗时 | RL 中位耗时 | OMPL 路径 | RL 路径 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.7 rad | 0.97 | 20/20 | **20/20** | 12 ms | **4 ms** | 1.29 rad | 1.77 rad |
| 1.2 rad | 1.66 | 15/15 | 12/15 | 12 ms | 6 ms | 1.78 rad | 2.53 rad |
| 1.8 rad | 2.49 | 15/15 | 6/15 | 13 ms | 7.5 ms | 2.46 rad | 3.36 rad |

结论：RL 比 OMPL 快 2–3 倍，但目标越大失败率越高（1.8 rad 时 60% 失败），
路径也长 25–35%。单独用 RL 不可靠。

## 加入 OMPL 兜底后

规则：
- 目标偏移 > `rl_max_delta_`（默认 1.5 rad）→ 直接走 OMPL，不试 RL
- 试了 RL 但失败 → 回退 OMPL，结果消息里注明 `rl failed (...), fell back to ompl`

| 目标偏移半径 r | OMPL 成功 | RL(混合) 成功 | OMPL 中位耗时 | RL(混合) 中位耗时 | OMPL 路径 | RL 路径 |
| --- | --- | --- | --- | --- | --- | --- |
| 0.7 rad | 20/20 | 20/20 | 12 ms | **4.5 ms** | 1.29 | 1.60 |
| 1.8 rad | 15/15 | 15/15 | 13 ms | 12 ms | 2.45 | 2.50 |

结论：小位移（焊接接近/退刀这类）RL 快 **2.7 倍**且 100% 成功；
大位移自动走 OMPL，耗时与纯 OMPL 持平。这条路可以用了。

## 环境变量

| 变量 | 作用 |
| --- | --- |
| `ROSIM_RL_NO_FALLBACK=1` | 关闭回退，用于测纯 RL 指标 |
| `ROSIM_RL_MAX_DELTA=1.5` | 覆盖 RL 的位移上限（rad，6 关节欧氏范数） |
