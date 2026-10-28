# 开发环境安装（在另一台电脑上复现）

> 目标：clone 下来能编译、能起服务、能跑基准。
> 所有版本号都来自当前开发机（2026-10-28 记录），可直接当基线用。
>
> 如果你的机器和这里不一致，先按本表装，再对照第 9 节"已知坑"排查。

---

## 0. 一键流程（熟练后照这个走）

```bash
# 1) 系统基础
sudo apt update && sudo apt install -y build-essential cmake git python3-pip \
     libeigen3-dev libboost-all-dev libtinyxml2-dev libconsole-bridge-dev \
     nodejs npm

# 2) ROS 2 Humble（见第 2 节）
# 3) MoveIt 2 源码（见第 3 节，必须和本文版本一致）
# 4) 本项目
git clone git@github.com:heyxz/smartwelding.git ~/rosim_ws
cd ~/rosim_ws && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

# 5) Python 训练/策略依赖（见第 5 节）
# 6) 验证（见第 6 节）
```

---

## 1. 硬件 / 操作系统

| 项 | 要求 | 开发机实测 |
| --- | --- | --- |
| 系统 | **Ubuntu 22.04 LTS（jammy）** | 22.04.5 LTS，内核 6.8.0 |
| 架构 | x86_64 | x86_64 |
| CPU | 4 核以上（MoveIt 编译吃 8 核） | — |
| 内存 | ≥ 16 GB（源码编译 MoveIt 时峰值较高） | — |
| 磁盘 | ≥ 40 GB 可用（`.pylibs` 5.3 GB + 构建产物约 1 GB） | — |
| GPU | **可选**：只有训练需要；纯规划/仿真不需要 | 带 CUDA 12.4 |

> ⚠️ Ubuntu 版本必须是 22.04。20.04 装不了 Humble 官方二进制；24.04 是 Jazzy，代码里的
> `rclcpp::Logger`、MoveIt 2.5 接口都不兼容。

---

## 2. ROS 2 Humble

```bash
# 官方 apt 源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
```

**版本基线**（本项目实测可用的组合）：

| 包 | 版本 |
| --- | --- |
| `ros-humble-rclcpp` | 16.0.16 |
| `ros-humble-ros2-control` | 2.52.2 |
| `ros-humble-controller-manager` | 2.52.2 |
| `ros-humble-joint-trajectory-controller` | 2.50.2 |
| `ros-humble-joint-state-broadcaster` | 2.50.2 |
| `ros-humble-robot-state-publisher` | 3.0.3 |
| `ros-humble-xacro` | 2.1.1 |
| `ros-humble-geometric-shapes` | 2.3.2 |
| `ros-humble-tf2-eigen` | 0.25.17 |
| `ros-humble-ompl` | 1.7.0 |

> 控制器用的是 `joint_state_broadcaster` + `joint_trajectory_controller`（不是 ROS 1 那套），
> 配置在 `src/rosim_description/config/`。

---

## 3. MoveIt 2（**必须源码编译**，不能用 apt）

这一点最容易踩坑：**开发机的 MoveIt 是源码构建的**，apt 里没有 `ros-humble-moveit-core`。
本项目 CMake 通过 `find_package(moveit_core)` 找它，所以要么源码装到 `~/ws_moveit`，
要么改成 apt 版（`sudo apt install ros-humble-moveit`）——后者版本不同（Humble 的 apt 版通常 2.5.3 左右），
本项目在 2.5.9 上验证过，**建议照下面来**。

```bash
mkdir -p ~/ws_moveit/src && cd ~/ws_moveit/src
git clone -b humble https://github.com/moveit/moveit2.git
vcs import < moveit2/moveit2.repos   # 需要 python3-vcstool
cd ~/ws_moveit
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**版本基线**：MoveIt `2.5.9`（git 描述 `2.5.9-7-gfec72cb44`，分支 `humble`）。

编译完成后每次开新终端都要：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash      # ← 关键，否则找不到 moveit_core
```

> 构建大约 15–25 分钟（8 核）。如果 `colcon build` 报 `moveit_core not found`，
> 99% 是忘了 source `~/ws_moveit/install/setup.bash`。

---

## 4. 本项目编译

```bash
git clone git@github.com:heyxz/smartwelding.git ~/rosim_ws
cd ~/rosim_ws

source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash

rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

7 个包：`rosim_core`、`rosim_description`、`rosim_interfaces`、`rosim_planning`、
`rosim_rl`、`rosim_server`、`rosim_viewer`。

系统库要求（都是 Ubuntu 22.04 自带版本）：

| 库 | 版本 | 用途 |
| --- | --- | --- |
| gcc / g++ | 11.4.0 | C++17 |
| cmake | 3.22.1 | 构建 |
| Eigen3 | 3.4.0 | 位姿/矩阵 |
| Boost | 1.74.0 | `libboost-dev` `libboost-system-dev` |
| tinyxml2 | 9.0.0 | URDF/SRDF 解析 |
| console-bridge | 1.0.1 | 日志 |
| OpenCASCADE (occt) | **7.5.1** | STEP 导入 / 边提取（`rosim_core`） |
| VTK | **9.1.0** | 可视化库 `librosim_viz`（仅 viewer 用） |
| Qt5 | 5.15.3 | 旧版桌面 GUI（`rosim_viewer`，可跳过） |
| Python | **3.10.12** | 脚本与训练侧 |

---

## 5. Python 依赖（策略节点 / 训练侧）

训练和策略推理的依赖**故意装在仓库内的 `.pylibs`**（不入 git），避免污染系统 Python：

```bash
cd ~/rosim_ws
python3 -m pip install --target .pylibs -r rl_a/requirements.txt
```

GPU 版本的 torch（CUDA 12.4）单独装：

```bash
python3 -m pip install --target .pylibs \
  --index-url https://download.pytorch.org/whl/cu124 torch==2.6.0
```

**版本基线**（`.pylibs` 实测）：

| 包 | 版本 |
| --- | --- |
| torch | **2.6.0+cu124**（纯 CPU 机器装 `torch==2.6.0` 即可） |
| stable-baselines3 | 2.9.0 |
| onnx | 1.23.2 |
| onnxruntime | 1.23.2 |
| numpy | 2.2.6 |
| gymnasium | 1.4.0 |
| pandas | 2.3.3 |
| tensorboard | 2.21.0 |

脚本通过 `RL_PYLIBS` 找这个目录（默认 `<仓库>/.pylibs`，见 `scripts/rl_plan.sh`）。

> 不用训练的话（只起服务、跑 OMPL/直线规划），**第 5 节可以整段跳过**。

---

## 6. Node.js（基准脚本用）

`scripts/ws_import.js` 需要一个 WebSocket 实现：

```bash
sudo apt install -y nodejs npm
sudo npm install -g ws        # 或者在本仓库根目录 npm i ws
```

开发机：Node `v22.23.3`。脚本会先尝试 `require("ws")`，找不到才回退到开发机路径，
所以在新机器上装好 `ws` 就没问题。

---

## 7. 启动与验证

### 7.1 起服务

```bash
systemctl --user start rosim-server        # 或者 bash scripts/debug.sh
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8765/    # 期望 200
```

> `rosim-server.service` 是**手动启动**（`disabled`），不会开机自启——这是有意的。
> 如果新机器用户名不是 `xz`，需要改 `scripts/rosim-server.service` 里的
> `WorkingDirectory` / `ExecStart` / `HOME` 路径。

### 7.2 冒烟测试（不需要 RL）

```bash
./scripts/workpiece_bench.sh 1 auto      # 导入 → 接近 → 沿缝 → 退刀，期望 SUCCESS
```

### 7.3 RL 策略节点（可选）

```bash
./scripts/rl_plan.sh &                   # 需要 .pylibs 和模型文件
# 日志出现：rl_plan 就绪：.../rl_a/out_bank_scene
```

### 7.4 规划器对比

```bash
python3 scripts/benchmark_suite.py       # ompl / rl / hybrid / auto / line
```

---

## 8. 目录与数据说明

| 路径 | 是否入库 | 说明 |
| --- | --- | --- |
| `src/`、`scripts/`、`rl_a/*.py`、`docs/` | ✅ | 源码与工具 |
| `rl_a/out_*/policy.onnx` `policy.json` | ✅ | 推理用的模型（小） |
| `rl_a/out_*/best_model.zip` `vecnormalize.pkl` | ❌ | 训练中间产物，体积大 |
| `rl_a/scene.json` `scene_bank.npz` `blocked_bank.npz` | ✅ | 训练/基准用的几何与场景库 |
| `.pylibs/` | ❌ | 5.3 GB 依赖，按第 5 节重装 |
| `.scratch/` | ❌ | 运行日志与临时产物（`.scratch/home` 是服务进程的 HOME） |
| `build/ install/ log/` | ❌ | colcon 产物 |

---

## 9. 已知坑（都踩过）

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| `find_package(moveit_core) failed` | 没 source `~/ws_moveit/install/setup.bash` | 每次开终端都 source |
| 服务起不来、8765 无响应 | 有旧进程占着端口 | `pkill -x rosim_server_node; pkill -x rosim_planning_` 再启动 |
| `pgrep -x rosim_planning_node` 匹配不到 | Linux `comm` 截断到 15 字符 | 用 `pgrep -x rosim_planning_` |
| rclpy 脚本报 `ROS_LOG_DIR` 相关错误 | `ROS_LOG_DIR` 指向的目录不存在 | 先 `mkdir -p` 该目录，或别设置它 |
| `bind: Address already in use` | 上一实例没退干净 | 同上，按精确进程名清理 |
| `ModuleNotFoundError: rclpy` | 没 source ROS 环境 | `source /opt/ros/humble/setup.bash` |
| RL 节点报找不到 `policy.onnx` | 模型目录不对 | `RL_OUT_DIR` 指向 `rl_a/out_bank_scene` |
| `colcon build` 极慢 | 没加 `--symlink-install` 且没开并行 | 用 `--symlink-install`，`--parallel-workers 8` |
| 服务重复启动两份 | systemd 单元 + 手动脚本同时起 | 只用一种方式启动 |

---

## 10. 版本总表（速查）

```
Ubuntu          22.04.5 LTS (jammy)
内核             6.8.0-138
gcc/g++         11.4.0        C++17
cmake           3.22.1
Python          3.10.12
Node.js         v22.23.3
ROS 2           Humble (rclcpp 16.0.16)
ros2_control    2.52.2
MoveIt 2        2.5.9  ← 源码构建，~/ws_moveit
OMPL            1.7.0
Eigen3          3.4.0
Boost           1.74.0
torch           2.6.0+cu124
stable-baselines3  2.9.0
onnxruntime     1.23.2
numpy           2.2.6
```
