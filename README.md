# rosim — 六轴工业机器人随边作业运动学仿真

STEP 工件导入 → 在浏览器里建焊缝 → 6 轴 MOTOMAN-GP12 + 第 7 轴直线导轨协同规划
无碰撞路径 → 沿焊缝连续执行，支持多条焊缝一次规划、连续走完。

- 完整设计、目录说明、协议与算法细节：**[src/README.md](src/README.md)**
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

## 回归

```bash
# 单边回归（全部边，与基线对比）
ros2 run rosim_planning edge_regress -- --out /tmp/run.json --compare <baseline.json>
# 焊缝批量
ros2 run rosim_planning edge_regress -- --seams "17;22;1,17"
```
