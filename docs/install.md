# 开发环境安装（在另一台电脑上复现）

> 目标：clone 下来能编译、能起服务、能跑基准。
> **前提假设：目标机器可能已经装了一部分东西，而且某些组件版本更高。**
> 所以顺序是：**先体检 → 再按差异决策 → 最后才安装**。
>
> 所有版本号都来自开发机实测（2026-10-28），且是"**实际导入生效**"的版本，
> 不是 pip 里的最新元数据（两者在开发机上不一致，见 5.2）。

---

## 0. 第一步永远是体检

```bash
git clone git@github.com:heyxz/rosim_ws.git ~/rosim_ws
cd ~/rosim_ws
bash scripts/check_env.sh          # 只读，不装任何东西
```

逐项打印 `期望 / 实际 / 状态`，状态含义：

| 状态 | 含义 | 该怎么做 |
| --- | --- | --- |
| `OK` | 与基线一致（允许 `+cu124`、`.3ubuntu7` 这类后缀） | 不用动 |
| `偏高` | 比基线新，通常是补丁版本 | 按第 2 节矩阵判断 |
| `偏低` | 比基线旧 | 先试，失败再升 |
| `缺失` | 没装 | 按对应章节装 |
| `冲突` / `需清理` | 多版本共存 / 重复安装，**结果不确定** | 必须处理，见第 2、5.3 节 |

退出码 `0` = 无致命问题；`1` = 有必须处理项（末尾会列清单）。

> 开发机上跑应全绿，只剩一条".pylibs 里有重复包"的提醒——那台机器上不影响运行
> （实际导入版本已验证），但**新机器不要这样装**（见 5.3）。

---

## 1. 硬件 / 操作系统

| 项 | 要求 | 开发机实测 |
| --- | --- | --- |
| 系统 | **Ubuntu 22.04 LTS（jammy）** | 22.04.5，内核 6.8.0 |
| 架构 | x86_64 | x86_64 |
| CPU | ≥ 4 核（源码编译 MoveIt 建议 8 核） | — |
| 内存 | ≥ 16 GB（编译 MoveIt 峰值高） | — |
| 磁盘 | ≥ 40 GB 可用（`.pylibs` 5.3 GB + 构建产物 ~1 GB） | — |
| GPU | **可选**，只有训练需要 | CUDA 12.4 |

**Ubuntu 版本是硬约束**：20.04 装不了 Humble 二进制；24.04 是 Jazzy，`rclcpp` 与
MoveIt API 都不兼容本项目。如果目标机是 24.04，只有三条路：

1. 换台 22.04 机器（最省事）
2. 用 `docker/Dockerfile` 起 22.04 容器（第 12 节）
3. 装 22.04 虚拟机

---

## 2. 版本兼容矩阵（目标机版本更高/更低时怎么判断）

### 2.1 系统与工具链

| 组件 | 基线 | 更高 | 更低 | 处理 |
| --- | --- | --- | --- | --- |
| Ubuntu | 22.04 | ❌ 不行 | ❌ 不行 | 换机器 / 容器 / 虚拟机 |
| gcc | 11.4 | ⚠️ 12/13 通常能编（C++17），未验证 | ❌ <10 不建议 | 官方镜像里用 11 |
| cmake | 3.22 | ✅ | ⚠️ <3.16 可能缺特性 | `snap install cmake --classic` |
| Python | **3.10** | ❌ **3.11/3.12 跑不了 rclpy** | ❌ 3.9 不行 | 保留 3.10，脚本显式 `python3.10` |
| Node.js | 18+ | ✅ | ⚠️ <16 建议升 | `ws` 模块只要 Node ≥14 |

**Python 最容易踩**：Humble 的 `rclpy` 是为 3.10 编译的。若目标机 `python3` 默认 3.12，
所有 `import rclpy` 的脚本会失败。处理（不改系统默认）：

```bash
python3.10 -m pip install ...
python3.10 scripts/benchmark_suite.py     # 显式用 3.10
```

### 2.2 ROS 与 MoveIt

| 组件 | 基线 | 更高 | 说明 |
| --- | --- | --- | --- |
| ROS 2 | Humble | ❌ Jazzy/Iron 不行 | 只支持 Humble |
| `ros-humble-*` 小版本 | 见 2.3 | ✅ 通常可以 | apt 补丁版本，接口稳定 |
| **MoveIt 2** | **2.5.9（源码）** | ❌ 2.6+ 未验证 | 用 humble 分支的 2.5.x |
| OMPL | 1.7.0 | ✅ 1.6/1.7 均可 | 用 ROS 里的版本 |

**MoveIt 两个坑**：

1. **apt 里没有 `ros-humble-moveit-core`**（开发机装的是源码版）。若目标机
   `apt install ros-humble-moveit` 了，apt 版和源码版会打架（先 source 的生效），
   建议二选一，优先源码 2.5.9。
2. 源码版应在**独立 workspace**（默认 `~/ws_moveit`）。目标机若已有别的 MoveIt
   workspace，指过去即可：
   ```bash
   MOVEIT_WS=~/moveit2_ws bash scripts/check_env.sh
   ```

### 2.3 ROS 包基线

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

### 2.4 系统库

| 库 | 基线 | 更高 | 更低 | 备注 |
| --- | --- | --- | --- | --- |
| Eigen3 | 3.4.0 | ✅ | ⚠️ 3.3 可能编不过 | `libeigen3-dev` |
| Boost | 1.74 | ✅ | ⚠️ <1.65 不行 | `libboost-dev libboost-system-dev` |
| tinyxml2 | 9.0.0 | ✅ | ✅ 8.x 可用 | |
| console-bridge | 1.0.1 | ✅ | ✅ | |
| **OpenCASCADE** | **7.5.1** | ⚠️ 7.6/7.7 要改 CMake 路径 | ❌ <7.3 接口不同 | STEP 导入 |
| **VTK** | **9.1.0** | ⚠️ 9.2/9.3 未验证 | ❌ 8.x 不行 | 仅 `rosim_viewer` |
| Qt5 | 5.15.3 | ✅ | ✅ | 仅旧版桌面 GUI |

> 只跑服务端 + 规划（不编 `rosim_viewer`）的话，VTK/Qt 可跳过：
> `colcon build --symlink-install --packages-skip rosim_viewer`

---

## 3. ROS 2 Humble

```bash
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions

# STEP 导入与可视化
sudo apt install -y libocct-foundation-dev libocct-data-exchange-dev \
     libvtk9-dev qtbase5-dev
```

验证：`source /opt/ros/humble/setup.bash && ros2 --help >/dev/null && echo OK`

---

## 4. MoveIt 2（**源码 2.5.9**）

```bash
mkdir -p ~/ws_moveit/src && cd ~/ws_moveit/src
git clone -b humble https://github.com/moveit/moveit2.git
cd moveit2 && git checkout 2.5.9        # 锁定基线
cd ~/ws_moveit/src
python3 -m pip install vcstool          # 或 apt install python3-vcstool
vcs import < moveit2/moveit2.repos
cd ~/ws_moveit
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

编译（约 15–25 分钟）后**每次开终端都要**：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
```

**目标机已有 MoveIt 源码时**：

```bash
grep -m1 "<version>" ~/你的moveit_ws/src/moveit2/moveit_core/package.xml
MOVEIT_WS=~/你的moveit_ws bash scripts/check_env.sh
# 若是 2.6+：不要改现有工作区，另建 2.5.9（避免破坏别的项目）
```

---

## 5. Python 依赖（策略节点 / 训练侧）

### 5.1 装到仓库内的 `.pylibs`

```bash
cd ~/rosim_ws
python3 -m pip install --target .pylibs -r rl_a/requirements.txt
python3 -m pip install --target .pylibs \
  --index-url https://download.pytorch.org/whl/cu124 torch==2.6.0
```

### 5.2 版本基线（**实际导入生效**）

| 包 | 实际版本 | 注意 |
| --- | --- | --- |
| torch | **2.6.0+cu124** | 纯 CPU 装 `torch==2.6.0` |
| stable-baselines3 | **2.3.2** | 不是 2.9 |
| gymnasium | **0.29.1** | sb3 2.3 配套 |
| numpy | **1.26.4** | **必须 1.x** |
| onnx | **1.16.2** | 导出 ONNX |
| onnxruntime | 1.23.2 | 推理 |
| pandas | 2.3.3 | 统计 |
| tensorboard | ≥2.11 | 训练曲线 |

**两个必须记住的坑**：

1. **numpy 不能升到 2.x**：torch 2.6 / sb3 2.3 的二进制按 numpy 1.x ABI 编译，
   升上去会 `numpy.dtype size changed` 或 import 失败。
2. **`pip list` 会骗人**：开发机 `.pylibs` 曾重复安装，元数据报
   `sb3 2.9.0 / numpy 2.2.6 / onnx 1.23.2`，**实际导入**的是
   `2.3.2 / 1.26.4 / 1.16.2`。判断版本一律用：
   ```bash
   PYTHONPATH=$PWD/.pylibs python3 -c "import stable_baselines3 as s; print(s.__version__)"
   ```
   `scripts/check_env.sh` 就是这么检测的。

### 5.3 `.pylibs` 里有多份同名包（体检报"需清理"）时

```bash
cd ~/rosim_ws
rm -rf .pylibs                      # 5.3 GB，可重装
python3 -m pip install --target .pylibs -r rl_a/requirements.txt
python3 -m pip install --target .pylibs \
  --index-url https://download.pytorch.org/whl/cu124 torch==2.6.0
# 验证（应为 2.3.2 / 0.29.1 / 1.26.4）
PYTHONPATH=$PWD/.pylibs python3 -c "
import stable_baselines3 as s, gymnasium as g, numpy as n
print(s.__version__, g.__version__, n.__version__)"
```

> 不想装 `.pylibs` 也行：**纯 OMPL / 直线规划不需要它**，只有
> `scripts/rl_plan.sh`（策略节点）和 `rl_a/`（训练）需要。

---

## 6. Node.js

```bash
sudo apt install -y nodejs npm
sudo npm install -g ws          # 或在仓库根 npm i ws
```

`scripts/ws_import.js` 先 `require("ws")`，找不到才回退到开发机路径。

---

## 7. 编译本仓库

```bash
cd ~/rosim_ws
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash

rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --parallel-workers 8
source install/setup.bash
```

不要旧版桌面 GUI 时：

```bash
colcon build --symlink-install --packages-skip rosim_viewer
```

---

## 8. 启动与验证

```bash
bash scripts/check_env.sh                                   # 体检
systemctl --user start rosim-server                         # 起服务
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8765/   # 200
./scripts/workpiece_bench.sh 1 auto                         # 冒烟（不需要 RL）
./scripts/rl_plan.sh &                                      # 可选：策略节点
```

> `scripts/rosim-server.service` 里写死了 `WorkingDirectory` / `HOME` / 用户名 `xz`，
> 新机器要改成自己的路径；或直接用 `bash scripts/run-server.sh` 前台跑。

---

## 9. 目录与数据说明

| 路径 | 入库 | 说明 |
| --- | --- | --- |
| `src/`、`scripts/`、`rl_a/*.py`、`docs/` | ✅ | 源码与工具 |
| `rl_a/out_*/policy.onnx`、`policy.json` | ✅ | 推理模型（小） |
| `rl_a/out_*/best_model.zip`、`vecnormalize.pkl` | ❌ | 训练中间产物（大） |
| `rl_a/scene.json`、`scene_bank.npz`、`blocked_bank.npz` | ✅ | 几何与场景库 |
| `.pylibs/` | ❌ | 5.3 GB，按第 5 节重装 |
| `.scratch/` | ❌ | 运行产物；`.scratch/home` 是服务进程 HOME |
| `build/ install/ log/` | ❌ | colcon 产物 |

---

## 10. 常用命令速查

```bash
bash scripts/check_env.sh                    # 体检
colcon build --symlink-install               # 编译
systemctl --user start|stop rosim-server     # 起/停服务
./scripts/workpiece_bench.sh 8 auto          # 随机位姿基准
python3 scripts/benchmark_suite.py           # 五规划器对比
```

---

## 11. 已知坑（都踩过）

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| `find_package(moveit_core) failed` | 没 source `~/ws_moveit/install/setup.bash` | 每次开终端都 source |
| `No module named rclpy` | 没 source ROS，或 `python3` 不是 3.10 | source ROS；用 `python3.10` |
| 8765 无响应 / `bind: Address already in use` | 旧进程占着 | `pkill -x rosim_server_node; pkill -x rosim_planning_` |
| `pgrep -x rosim_planning_node` 匹配不到 | 进程名截断到 15 字符 | 用 `pgrep -x rosim_planning_` |
| `ROS_LOG_DIR` 相关报错 | 目录不存在 | `mkdir -p` 该目录，或不设置 |
| `numpy.dtype size changed` | numpy 被升到 2.x | 锁 `numpy==1.26.4` |
| `pip list` 版本与实际不符 | `.pylibs` 重复包 | 5.3 节干净重装 |
| torch 是 CPU 版 | 没加 `--index-url .../cu124` | 重装 GPU 版（CPU 也能跑，只是慢） |
| `colcon build` 慢 | 没并行 | `--parallel-workers $(nproc)` |
| 服务起两份 | systemd 与手动脚本同时启 | 只用一种 |
| `set -u` 让 ROS 脚本报 unbound variable | `set -u` 与 setup.bash 冲突 | 脚本里用 `set +u` |

---

## 12. 备选：容器（目标机版本不匹配时的兜底）

目标机是 24.04 / 已有 Jazzy / 环境太乱时，容器最干净：

```bash
cd ~/rosim_ws
docker build -f docker/Dockerfile -t rosim:humble .
docker run --rm -it --net=host -v "$PWD":/workspace rosim:humble
# 容器内
cd /workspace && bash scripts/check_env.sh && colcon build --symlink-install
```

> `docker/Dockerfile` 按本文档手写（22.04 + Humble + MoveIt 2.5.9 源码 + Python 依赖）。
> **开发机没装 docker，未实际构建验证**；首次构建若失败把报错发我。
> GPU 训练需要 `--gpus all` 和 nvidia-container-toolkit。

---

## 13. 版本总表（速查）

```
Ubuntu          22.04.5 LTS (jammy)          ← 硬约束
内核             6.8.0
gcc/g++         11.4.0        C++17
cmake           3.22.1
Python          3.10.12                      ← rclpy 要求 3.10
Node.js         v22.x                        （≥18 均可）
ROS 2           Humble                       ← 硬约束
ros2_control    2.52.2
MoveIt 2        2.5.9（源码 ~/ws_moveit）    ← 不用 apt 版
OMPL            1.7.0
Eigen3          3.4.0
Boost           1.74.0
OpenCASCADE     7.5.1
VTK             9.1.0（仅 viewer）
torch           2.6.0+cu124
stable-baselines3  2.3.2
gymnasium       0.29.1
numpy           1.26.4                       ← 必须 1.x
onnx / onnxruntime  1.16.2 / 1.23.2
```


> 第二台机器（笔记本）的部署有更省时的做法：见 **docs/laptop_env.md**（拷预编译 MoveIt，约 5 分钟，不用源码编译）。
