# rosim — 六轴工业机器人随边作业运动学仿真

STEP 工件导入 → 在浏览器里建焊缝 → 6 轴 MOTOMAN-GP12 + 第 7 轴直线导轨协同规划
无碰撞路径 → 沿焊缝连续执行，支持多条焊缝一次规划、连续走完。

- 完整设计、目录说明、协议与算法细节：**[src/README.md](src/README.md)**
- 代码地图（哪个功能在哪个文件）：**[docs/code_map.md](docs/code_map.md)**
- OMPL vs RL 对比数据：**[docs/rl_bench.md](docs/rl_bench.md)**
- 落地路线与里程碑：**[docs/roadmap.md](docs/roadmap.md)**
- 换台电脑装环境（依赖+版本+坑）：**[docs/install.md](docs/install.md)**
- 代码位于 `src/`（7 个 ROS 2 包）
- 依赖：ROS 2 Humble、MoveIt2、OpenCASCADE 7.5、VTK 9.1、Qt5（仅旧版桌面 GUI）

## 编译

```bash
cd ~/rosim_ws
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash      # MoveIt2 源码 overlay
colcon build --symlink-install
```

## 运行

```bash
source install/setup.bash
ros2 launch rosim_server server.launch.py
# 调试页 http://localhost:8765/    生产页 http://localhost:8765/production
```

## RL 规划（可选）

焊缝的接近/退刀支持三种规划器：`ompl`（默认）、`line`（笛卡尔直线）、
`rl`（策略 + OMPL 兜底）。

```bash
./scripts/rl_plan.sh &                      # 策略推理节点（必须先起）
ros2 service call /rosim_planning/set_planner rosim_interfaces/srv/SetPlanner \
  "{planner_type: rl}"
```

调试页右上「规划器」下拉可切换，旁边「规划演示」按钮直接规划一次并画出路径。
训练与模型说明见 [docs/code_map.md](docs/code_map.md#3-rl-训练侧rl_a)。

## 回归

```bash
# 单边回归（全部边，与基线对比）
ros2 run rosim_planning edge_regress -- --out /tmp/run.json --compare <baseline.json>
# 焊缝批量
ros2 run rosim_planning edge_regress -- --seams "17;22;1,17"
```
