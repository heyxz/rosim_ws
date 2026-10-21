# 代码地图（2026-10-21）

这份文档只回答一个问题：**某个功能在哪个文件里**。协议与算法细节看
`src/README.md`，RL 对比数据看 `docs/rl_bench.md`。

## 1. ROS 2 包

| 包 | 职责 | 关键文件 |
| --- | --- | --- |
| `rosim_core` | 几何/STEP 导入、边提取、焊缝拼装、角点规划 | `src/edge_geometry.cpp`、`corner_planner.cpp` |
| `rosim_description` | URDF/xacro、网格、控制器与限位配置 | `urdf/robot.urdf.xacro`、`config/ros2_controllers.yaml`、`config/joint_limits.yaml` |
| `rosim_interfaces` | 全部 srv/action 定义 | `srv/*.srv`、`action/*.action` |
| `rosim_planning` | 规划与执行的核心 | 见下表 |
| `rosim_server` | WebSocket/HTTP 服务、调试页 | `src/server_node.cpp`、`web/index.html` |
| `rosim_rl` | RL 规划器插件（dlopen 进 planning） | `src/rl_remote_planner.cpp` |

`rosim_planning` 内部：

| 文件 | 内容 |
| --- | --- |
| `motion_planner.h` | `MotionPlanner` 抽象 + 工厂（"ompl"/"rl"/"line"） |
| `ompl_planner.cpp` | MoveIt/OMPL 规划器 |
| `line_planner.cpp` | 笛卡尔直线规划器（逐步 IK + 碰撞 + 跳变检查） |
| `fallback_planner.cpp` | 装饰器：先试主规划器，失败或超位移门限则回退 OMPL |
| `edge_follower.cpp` | 焊缝执行：接近 → 沿缝 → 退刀，时间参数化与重采样 |
| `trajectory_timing.cpp` | `regularizeTiming()`：最短段钳制 + 50 ms 均匀网格 + 速度填充 |
| `planning_node.cpp` | 节点：服务、动作、规划器缓存与切换 |

## 2. 规划链路

```
/rosim_planning/plan_joint            ← 测试/基准入口（关节目标）
/rosim_planning/follow_seams (action) ← 焊缝批量执行
        │
        └─ MotionPlanner::plan()
             ├─ ompl   : RRTConnect + TOTG
             ├─ rl     : FallbackPlanner( RlRemotePlanner, ompl )
             │             └─ /rl_plan 服务 → scripts/rl_plan_node.py（ONNX）
             └─ line   : FallbackPlanner( LinePlanner, ompl )
```

规划器切换：`set_planner` 服务。实例缓存在 `planner_cache_` 里，**切换不销毁
旧实例**——OMPL 求解失败后销毁会段错误（2026-10-21，`e9a102a`）。

`set_planner` 对非 ompl 规划器自动包一层 `FallbackPlanner`：

- 位移超 `ROSIM_RL_MAX_DELTA`（默认 1.5 rad，仅对 rl 生效）→ 直接走 OMPL
- 主规划器失败 → 回退 OMPL，消息里注明 `... fell back`
- `ROSIM_RL_NO_FALLBACK=1` 关闭回退（测纯 RL 指标用）

## 3. RL 训练侧（`rl_a/`）

| 文件 | 内容 |
| --- | --- |
| `kin.py` | 从 URDF 重建的运动学（含导轨 + 0.167 m 基座偏移）、胶囊碰撞 |
| `env.py` | 与 `RlPlanner` 同构的环境（obs 22 / 6 轴；带导轨版本 obs 25 / 7 轴） |
| `scene_env.py`、`real_scene.py` | 用导出的真实规划场景做训练环境 |
| `build_bank.py`、`build_bank_from_log.py` | 生成起止位姿样本库（标定 / 真实调用日志） |
| `train.py` | sb3 PPO 训练（课程学习 + 评估回调） |
| `export_best.py` | 训练后在 final/best 之间评估，导出更好的那个到 ONNX |

模型产物（`policy.onnx` + `policy.json` 的归一化统计 + `eval_success`）：

| 目录 | 维度 | 自评成功率 |
| --- | --- | --- |
| `rl_a/out_rail/` | 7 轴（导轨 + 6 关节） | 97.5%（训练分布内） |
| `rl_a/out_a2/` | 6 轴 | 90.0%（训练分布内） |

**注意**：自评只代表训练分布。实际起点常落在分布外，此时 RL 失败、由
`FallbackPlanner` 回退 OMPL——这是设计行为，不是 bug。

## 4. 脚本（`scripts/`）

| 脚本 | 用途 |
| --- | --- |
| `debug.sh` | 重启服务 + 打开调试页 |
| `run-server.sh` | systemd 服务的实际启动包装 |
| `rosim-server.service` | 用户级 unit（不随开机自启，手动 start） |
| `rl_plan_node.py` / `rl_plan.sh` | RL 策略推理服务（`/rl_plan`） |
| `verify_rl.py` | 四种规划器用例 + TCP 直线偏离度量 |
| `rl_benchmark.py` | OMPL vs RL 基准（成功率/耗时/路径长度） |
| `dump_scene.py` | 导出 MoveIt 规划场景（给训练侧用真实几何） |

## 5. 数据与依赖路径

| 路径 | 内容 | 可否删除 |
| --- | --- | --- |
| `.pylibs/`（仓库根） | torch 2.6 + sb3 + onnxruntime（5.3 GB） | 不能，RL 运行时依赖 |
| `rl_a/out_*/` | 模型与评估曲线（`out_rail`/`out_real`/`out_scene`/`out_a2`） | — |
| `.scratch/home` | 服务进程的 HOME（`run-server.sh` 指定，ROS 日志落在这里） | **不能删** |

## 6. 回归与验证入口

```bash
# 边/焊缝规划回归
ros2 run rosim_planning edge_regress -- --step <file.step> --seams "6,10" --seam-uniform
# 规划器矩阵（需要先起 rl_plan.sh）
python3 scripts/verify_rl.py
# 对比基准
python3 scripts/rl_benchmark.py --trials 20
```
