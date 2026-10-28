# 笔记本（第二台机器）开发环境：可编译、可加载 MoveIt

> 目标不是"第二台工作站"，而是**主力机不在时仍能编译 + 做轻量验证**。
> 实测日期 2026-10-29，Ubuntu 22.04 / 8 核 / 15 GB / 无 GPU。

## 结论

| 能力 | 状态 |
| --- | --- |
| `colcon build` 全 7 个包 | ✅ 通过（约 2.5 分钟） |
| 加载 MoveIt（运行时） | ✅ `ik_probe` 能起，`ldd` 无缺失 |
| OMPL / 直线 / 混合规划节点 | ✅ 可用 |
| RL 训练、ONNX 策略推理 | ❌ 缺 `.pylibs` 与 libtorch |
| GUI / rviz | ❌ 无显示（可远程 X11，但没必要） |

自检：`bash scripts/check_env.sh` → **「没有致命问题」**（仅提示 `.pylibs` 缺失）。

## 端到端验证（2026-10-29 实测）

```
ros2 launch rosim_server server.launch.py     # 8765 → 200，6 个节点
./scripts/workpiece_bench.sh 3 ompl           # 3/3 SUCCESS，37~40 秒/位姿
```

与主力机的 34~40 秒基本持平（规划是单线程瓶颈，8 核够用）。

**注意**：某次单个位姿失败（`fraction 0.696, edge not fully traversable`）是
**真实的运动学极限**，不是环境问题——同一位置在主战机上同样失败（y≈1.03 的
可达性只到 70%）。

## 一、APT 依赖（一次性）

```bash
sudo apt update && sudo apt install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster ros-humble-xacro \
  ros-humble-geometric-shapes ros-humble-ompl \
  ros-humble-moveit-msgs ros-humble-control-msgs ros-humble-trajectory-msgs \
  ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-pluginlib \
  ros-humble-octomap ros-humble-octomap-msgs ros-humble-ruckig \
  liboctomap-dev libfcl-dev liburdfdom-dev libyaml-cpp-dev \
  libocct-foundation-dev libocct-data-exchange-dev libocct-ocaf-dev
```

> 这些是 MoveIt 的**传递依赖**，apt 装 MoveIt 时会带上；我们是从主力机拷预编译
> MoveIt，所以必须手动补齐。漏一个就会在 `find_package` 阶段报错，
> 报错信息会指明缺哪个包（`fcl` → `libfcl-dev`，`ruckig` → `ros-humble-ruckig`…）。

## 二、MoveIt：拷预编译产物（约 40 秒）

不源码编译（那要 40~70 分钟），直接从主力机拷 `install/` 树：

```bash
# 主力机
rsync -a ~/ws_moveit/install laptop:~/ws_moveit/

# 笔记本
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
ros2 pkg list | grep -c moveit        # 应约 49
```

## 三、唯一需要手工修的地方：geometric_shapes soname

预编译的 `moveit_core` 链接的是 `libgeometric_shapes.so.2.3.2`，而笔记本 apt 只能
装到 **2.3.4** → 运行时 `error while loading shared libraries: libgeometric_shapes.so.2.3.2`。

修复（把主力机那个精确版本的 so 拷过去，两个版本共存）：

```bash
# 主力机
scp /opt/ros/humble/lib/libgeometric_shapes.so.2.3.2 laptop:/tmp/
# 笔记本
sudo cp /tmp/libgeometric_shapes.so.2.3.2 /opt/ros/humble/lib/
sudo ldconfig
ldd ~/rosim_ws/install/rosim_planning/lib/rosim_planning/ik_probe | grep "not found"   # 应无输出
```

**这是"拷预编译件"方案的固有代价**：只要两台机器的 ROS 包补丁版本不同，就可能
再撞上别的 soname。判断方法：

```bash
ldd <二进制> | grep "not found"
```

有输出就照同样办法补对应版本的 `.so`；补得太多次说明该改为源码编译 MoveIt。

## 四、编译

```bash
cd ~/rosim_ws
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run rosim_planning ik_probe     # 会报缺 robot_description，属正常
```

## 五、踩过的坑（已在仓库修复）

| 现象 | 原因 | 修复 |
| --- | --- | --- |
| `can't find .../rosim_planning/launch` | 空目录 git 不保存 | CMake 加 `if(EXISTS)` |
| `can't find .../rosim_rl/include/` | 同上 | 同上 |
| `Could not find fcl` | 缺 `libfcl-dev` | apt 补 |
| `Could not find ruckig` | 缺 `ros-humble-ruckig` | apt 补 |
| `No rule to make target liboctomap.so` | 缺 `ros-humble-octomap` | apt 补 |
| `libgeometric_shapes.so.2.3.2: not found` | 补丁版本不同 | 拷精确版本 so |

## 六、RL 能力（可选）

想在这台跑 RL 推理/训练，还需要：

```bash
# 从主力机同步 Python 依赖（5.3 GB）
rsync -a 主力机:~/rosim_ws/.pylibs ~/rosim_ws/
```

libtorch（C++ 侧）默认关闭（`ROSIM_WITH_TORCH=OFF`），不影响 OMPL/直线规划。

## 两台机器的版本差异（2026-10-29 实测 diff）

**完全相同**：gcc 11.4.0 / cmake 3.22.1 / python 3.10 / ROS humble / ompl 1.7.0 /
eigen 3.4.0 / boost 1.74 / occt 7.5.1 / vtk 9.1.0 / qt 5.15.3 / xacro 2.1.1 /
robot_state_publisher 3.0.3

**笔记本更高**（apt 镜像取自更新日期，主力机是早先装的）：

| 包 | 主力机 | 笔记本 | 影响面 |
| --- | --- | --- | --- |
| rclcpp | 16.0.16 | 16.0.21 | 补丁级，无感 |
| ros2-control | 2.52.2 | 2.54.3 | 控制器行为/时序可能有细微差别 |
| controller-manager | 2.52.2 | 2.54.3 | 同上 |
| joint-trajectory-controller | 2.50.2 | 2.54.1 | 同上 |
| joint-state-broadcaster | 2.50.2 | 2.54.1 | 同上 |
| geometric-shapes | 2.3.2 | 2.3.4 | **本机手工补了 2.3.2 的 so** |
| tf2-eigen | 0.25.17 | 0.25.24 | 补丁级 |
| node | 22.23.3 | 22.23.4 | 补丁级 |

### 风险判断

1. **我们自己的代码不直接 `find_package(ros2_control)`**，只在 launch 里用控制器 →
   版本差异体现在「轨迹执行」环节，可能让同一轨迹的时间参数化略有不同。
2. **geometric-shapes 是唯一的混合态**：编译用 2.3.4 头文件，MoveIt 运行时加载
   2.3.2 的库。实测 3/3 通过，但理论上如果两版改了 inline 函数或结构体布局，
   会出现难查的运行时错误。
3. **RL 相关完全不可验证**：笔记本没有 `.pylibs`（torch/sb3/onnx 全缺）。

### 如果要严格一致

```bash
# 主力机导出可复现的版本清单
dpkg-query -W -f='${Package}=${Version}\n' \
  'ros-humble-*' 'libocct-*' 'libfcl-dev' 'liboctomap-dev' > versions.lock
# 笔记本按清单装（旧版本可能已从镜像移除，那就只能统一到"新版本"）
```

实话：**目前做不到完全一致**——apt 镜像上旧版本已被替换（实测 2.3.2 已无法安装）。
现实策略是「**主力机为准，笔记本做冒烟**」，并接受上面第 2 条的风险。

## 记忆同步（实测修复）

`scripts/memory_sync.sh` 早期版本有缺陷：**本机无改动就 `exit`，从不 `pull`**，
导致被动方永远停在旧版本（实测差 1 个提交，且会一直差下去）。

现已改为：`pull --rebase --autostash` → 有改动则提交 → 再 pull → push。
两台每 5 分钟各自对齐一次，实测收敛到同一提交。

记忆规模：40 条 / 约 7 KB（硬上限 40 条 / 12 KB，不会膨胀）。
