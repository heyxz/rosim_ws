# 六轴工业机器人随边作业运动学仿真软件

导入 STEP/BREP 工件并 3D 显示，在浏览器中选边，由 6 轴 MOTOMAN-GP12 + 第 7 轴
直线导轨协同规划无碰撞路径，沿焊缝/棱边完成随边作业并动画执行；预留 RL 规划扩展。
系统由两个进程组成：**rosim_planning**（几何、IK、OMPL、轨迹）与 **rosim_server**
（无头渲染、WebSocket、浏览器 UI）。**不启动 move_group**，OMPL 以内嵌插件形式
运行在规划节点进程内。

---

## 1. 功能概览

- **工件导入**：STEP/BREP 经 OpenCASCADE 加载、边枚举、离散化与三角化；支持世界位姿
  设置、拖动与精调。
- **边选择**：WebUI 中点击焊缝/工件表面，或在边列表中选；3D 视图中高亮当前边。
- **7 轴规划**：`arm`（6 轴，KDL/Cartesian）与 `manipulator`（6 轴 + `rail_joint`，
  OMPL）两个 SRDF group；自由空间与跨站位移动由 7 轴 OMPL 一起规划。
- **随边跟随**：多种子 IK → OMPL 接近 → 笛卡尔沿边 → 退刀 → TOTG 全轨时间参数化。
- **模拟执行闭环**：`ros2_control` mock 硬件 + `joint_trajectory_controller`，
  `/joint_states` 同时驱动 TF、服务端显示与浏览器动画。
- **双 WebUI**：
  - 调试模式 `/`：工件导入/移动、焊缝列表与建缝、跟随参数、JOG 手动（轴运动/笛卡尔切换、按住连续运动）、回零、日志。
  - 生产模式 `/production`：只读工件视图、焊缝列表、状态/进度、开始焊接/停止。
- **RL 扩展**：`rosim_rl` 编译为 `librosim_rl.so`，运行时 dlopen 注册 `"rl"` 规划器；
  真正的策略推理在 Python 侧（`/rl_plan` 服务）。另有实验性 `hybrid` 入口，
  保留 RL 已走通的前缀、只让 OMPL 修后半段。

### 技术栈

| 层次 | 技术 | 用途 |
| --- | --- | --- |
| 中间件 | ROS 2 Humble（rclcpp / rclcpp_action） | 节点、服务、Action、参数 |
| 运动学/规划 | MoveIt2 core：RobotModel、PlanningScene、KDL、FCL | 模型、碰撞、IK |
| 采样规划 | OMPL（`ompl_interface/OMPLPlanner`，pluginlib 内嵌） | 自由空间与 7 轴路径 |
| 轨迹 | MoveIt `CartesianInterpolator` + TOTG | 沿边笛卡尔插值、时间参数化 |
| 几何内核 | OpenCASCADE 7.5（OCCT） | STEP/BREP、边遍历、网格化 |
| 执行 | ros2_control `mock_components/GenericSystem` + JTC | 无物理仿真的闭环执行 |
| 服务端渲染 | VTK 9.1（离屏 X/GLX，PIMPL 隔离 X11） | 服务端场景帧（可选 JPEG 流） |
| Web 服务 | Boost.Beast（HTTP + WebSocket）、nlohmann/json | 页面托管与 JSON 协议 |
| 浏览器渲染 | three.js（vendored）+ OrbitControls | 调试/生产页面 WebGL 视图 |
| 桌面 GUI（保留） | Qt5 + VTK（QVTKOpenGLNativeWidget） | 旧版本地调试界面 |
| RL（可选） | ONNX Runtime（Python 侧）+ rclpy 服务 | 策略推理；C++ 只负责轨迹拼装与碰撞校验 |

工具链基线（实测 2026-10-29）：Ubuntu 22.04、gcc 11.4、CMake 3.22、OpenCASCADE
7.5.1、VTK 9.1.0、ROS 2 Humble + MoveIt2（`~/ws_moveit` 源码 overlay）。

---

## 2. 运行架构

```text
浏览器（/ 调试页，/production 生产页）
   │  HTTP + WebSocket :8765
   ▼
rosim_server_node
   ├── WsServer（Boost.Beast）        页面/资产托管、JSON 广播、可选 JPEG 帧广播
   ├── FrameRenderer（VTK 离屏）     机器人 + 工件 + 边；captureJpeg()
   ├── 场景 JSON                      URDF 碰撞几何 + 12 Hz 连杆位姿
   └── ROS 客户端 ───────────────┐
                                  │ service / action
                                  ▼
                         rosim_planning_node
                           ├── RobotModel + PlanningScene（FCL）
                           ├── EdgeFollower（IK / OMPL / Cartesian / 退刀 / TOTG）
                           └── /arm_controller/joint_trajectory
                                  │
                         ros2_control（mock）+ JTC
                                  │ /joint_states
                    ┌─────────────┴─────────────┐
              robot_state_publisher        rosim_server 显示状态
```

要点：

- 浏览器是**瘦客户端**：调试页接收 `scene`（机器人全部碰撞体）+ `workpiece`
  （工件网格与焊缝折线）+ 12 Hz `robot_state`（连杆位姿）；生产页只接收
  `workpiece`，不接收机器人场景与高频位姿。
- `subscribe_stream` 可让客户端接收服务端 VTK 渲染的 JPEG 二进制帧；当前两个页面
  均使用 WebGL，JPEG 流作为可选通道保留。
- 所有业务状态（当前工件、选中边、速度倍率、规划器）在 `rosim_server` 内维护并广播；
  `rosim_planning` 只负责几何/规划/执行。

### 2.1 规划器组合（`set_planner` 的取值）

`PlanningNode::makePlanner()` 是唯一决定组合的地方，**五种取值**：

| `planner_type` | 实际组合 | 说明 |
| --- | --- | --- |
| `auto`（默认） | `FallbackPlanner(line, ompl, 1e9)` | 直线可行就用直线（确定性、毫秒级），否则 OMPL。**不经过 RL** |
| `line` | `FallbackPlanner(line, ompl, 1e9)` | 同上，显式指定 |
| `ompl` | OMPL 本体 | 采样规划，几何上有保证但慢且不可复现 |
| `rl` | `FallbackPlanner(rl, ompl, rl_max_delta)` | 策略求解；位移超门限或失败则整段回退 OMPL |
| `hybrid` | `HybridPlanner(rl, ompl, rl_max_delta)` | RL 前缀 + OMPL 修尾，不丢弃 RL 已走通的段 |

> ⚠️ **`auto` 历史上曾经是 `line → rl → ompl`，现已改成 `line → ompl`。**
> 原因写在 `planning_node.cpp` 的注释里：真实请求中 94% 的直连路径本身就无碰撞，
> 剩下 6% 恰好是策略最不擅长的绕行场景，把 RL 放进默认链只会增加延迟和不确定性。
> 想验证 RL 请显式选 `rl` / `hybrid`，它们仍是实验性入口。

---

## 3. 代码结构与文件说明

```text
~/rosim_ws/src/
├── README.md
├── rosim_interfaces/          # msg / srv / action
├── rosim_description/         # 机器人、导轨、焊枪、控制器配置
├── rosim_core/                # 纯几何 + VTK 显示库
├── rosim_planning/            # 规划节点与随边跟随算法
├── rosim_server/              # 无头渲染 + WebSocket + 浏览器 UI
├── rosim_rl/                  # RL 远程规划器插件（把 rollout 委托给 /rl_plan）
└── rosim_viewer/              # 旧版 Qt+VTK 桌面 GUI（保留）
```

### 3.1 rosim_interfaces

| 文件 | 作用 |
| --- | --- |
| `action/FollowEdge.action` | 随边跟随 Action。Goal：`model_id`、`edge_index`、姿态模式、`tangent_tilt`、退刀/逼近/采样间距；Result：`success`、`error_code`、`cartesian_fraction`、`trajectory`；Feedback：`progress`、`phase`（resolving/planning/following/executing/done）。 |
| `srv/ImportModel.srv` | 导入工件（路径 + 世界位姿），返回 `model_id` 与全部 `EdgeInfo`。 |
| `srv/MoveToPose.srv` | 规划并执行到 `tool0` 目标位姿；自动尝试导轨站位。 |
| `srv/MoveWorkpiece.srv` | 移动已导入工件（重新生成碰撞体与显示网格）。 |
| `srv/GoHome.srv` | OMPL 规划回 SRDF `home`，不强制吸附。 |
| `srv/JogJoint.srv` | 单关节增量微调（含导轨，限幅 + 插值碰撞检查）。 |
| `srv/SetPlanner.srv` | 切换 `ompl` / `rl`。 |
| `srv/SetSpeedScale.srv` | 速度倍率。 |
| `msg/EdgeInfo.msg` | 单条边的 `model_id`、`edge_index`、世界坐标折线、长度、闭合标记。 |
| `msg/ModelInfo.msg` | 工件模型摘要。 |

### 3.2 rosim_description

| 文件 | 作用 |
| --- | --- |
| `urdf/robot.urdf.xacro` | 完整机器人：2.8 m 直线导轨 + GP12 + RM62 焊枪。链为 `rail_base_link → rail_joint(prismatic X) → rail_carriage → base_link → j1…j6 → flange → tool0 → tool_tip`。导轨行程 ±1.05 m，`effort=3000`，`velocity=1.2 m/s`；车架顶面把机器人基座抬高到 z=0.167 m。 |
| `urdf/robot.ros2_control.xacro` | `mock_components/GenericSystem` 硬件；7 个关节的 `initial_value` 与 SRDF `home` 一致，启动无跳变。 |
| `config/robot.srdf` | `arm`（base_link→tool0，6-DOF）与 `manipulator`（arm + `rail_joint`，7-DOF）两个 group；`home`/`zeros` 命名状态；`world→rail_base_link` 固定虚拟关节；经 `acm_probe` 实测的 `disable_collisions` 表。 |
| `config/kinematics.yaml` | KDL 仅求解 6 轴 `arm`；沿边 IK/Cartesian 时导轨固定在分配站位，保证腕部求解器为方阵且良态。 |
| `config/joint_limits.yaml` | TOTG 使用的速度/加速度限值；导轨 `1.2 m/s`、`2.0 m/s²`，其余按 GP12 厂商值。 |
| `config/ompl_planning.yaml` | `manipulator` 的 OMPL 配置：RRTConnect/RRT/PRM，投影 `joints(j1,j2)`，`longest_valid_segment_fraction=0.005`。 |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster` + `arm_controller`（JTC，7 关节，position 接口，100 Hz）。 |
| `launch/server.launch.py`（在 rosim_server 下） | 当前主入口：robot_state_publisher → ros2_control → spawner → rosim_planning → rosim_server（+ 可选 RViz）。 |
| `launch/robot_bringup.launch.py` | 旧版全量 bringup；`use_viewer` 默认 `true`（Qt viewer），viewer 退出会级联关闭系统。 |
| `launch/display.launch.py` | 仅模型显示（RViz）。 |
| `scripts/check_home_clear.py` | 纯 numpy FK + 连杆 STL，预检工件与 home 位的最小间隙，不需要启动 ROS。 |
| `meshes/` | `gp12/*.stl`（base_link、link1–5、flange、torch_rm62）、`workpiece_sample.step`；总约 61 MB，仿真网格约 126 万三角形。 |

### 3.3 rosim_core（`librosim_occ` / 显示库）

| 文件 | 作用与关键函数 |
| --- | --- |
| `include/rosim_core/occ_geometry_service.h`、`src/occ_geometry_service.cpp` | 无状态 OCC 服务。`loadShape()` 按扩展名加载 STEP/BREP；`importModel()` 加载 + 世界位姿 + 边枚举；`topologicalEdges()` 以 `TopExp_Explorer` 稳定枚举 `TopAbs_EDGE`，保证规划端与显示端边号一致；`discretizeEdge()` 弧长近似均匀离散；`edgeLength()`、`edgeIsClosed()`；`tessellate()` 用 `BRepMesh` 生成三角网格；`toCollisionMesh()` 生成 `geometric_shapes::Mesh` 供 FCL 使用；`eigenToTrsf()`。 |
| `include/rosim_core/edge_frame_generator.h`、`src/edge_frame_generator.cpp` | 把边折线转为 TCP 位姿。`generate()` 按模式生成帧；`tangentAt()` 中心差分（端点单侧）；`frameAt()` 用参考向量构造平行传输帧。TANGENT 模式工具 +Z 沿切线，`tangent_tilt` 向帧上轴抬起（焊枪导前角）；FIXED 恒定姿态；NORMAL 预留。 |
| `include/rosim_core/occ_to_vtk.h`、`src/occ_to_vtk.cpp` | VTK 图元。`eigenToVtkMatrix()`（逐元素写入，避免行/列主序错位）；`buildSolidPolyData()`；`buildSolidActor()`（法线 + 高光）；`buildEdgeActor()`（约 1.5 mm 管状可拾取边）；`setEdgeHighlight()`（青色高亮）。 |
| `include/rosim_core/robot_link_actors.h`、`src/robot_link_actors.cpp` | `RobotLinkActors::init()` 按 URDF 碰撞形状建 actor；`addToRenderer()`；`updateFromState()` 用 `global link transform × shape origin` 更新；`setAllVisible()`。 |
| `tools/gp12_prepare.cpp` | 从厂商 STEP 提取关节轴心/坐标系并导出连杆 STL（`probe` / `export`）。 |
| `tools/step_generator.cpp` | 生成测试用 STEP。 |
| `test/test_occ_step.cpp` | OCC 导入/边提取测试。 |

### 3.4 rosim_planning

| 文件 | 作用与关键函数 |
| --- | --- |
| `include/rosim_planning/planning_node.h`、`src/planning_node.cpp` | 规划节点。`loadRobotModel()` 加载 URDF+SRDF+kinematics；`setupInterfaces()` 注册所有 service/action；`handleImportModel()` 导入工件并注入碰撞体；`executeFollowEdge()` 在 worker 线程跑完整管线并发布 feedback；`monitorExecution()` 等控制器收敛（关节误差 < 0.003）后返回；`publishTrajectory()` 发 `/arm_controller/joint_trajectory`；`snapshotCurrentState()` 取 `/joint_states` 真值；`handleMoveToPose()`、`handleGoHome()`、`handleJogJoint()`、`handleMoveWorkpiece()`、`handleSetSpeedScale()`、`handleSetPlanner()`。 |
| `include/rosim_planning/edge_follower.h`、`src/edge_follower.cpp` | 随边跟随核心（纯计算，不碰 ROS 话题）。`execute()` 编排：重采样 → 导轨站位 → 帧生成 → 接近 IK → OMPL 预接近 → Cartesian 沿边 → 截断处理/重定位 → 退刀/末端间隙修剪 → 合并 + TOTG。`solveIkMultiSeed()` 多种子 IK + 碰撞过滤；`solveIkWithRailAllocation()` 0.25/0.5 m 步进换站位；`runCartesian()` 4 mm 步长笛卡尔；`railStationFor()` 按目标 X 与臂展分配站位；`setRailStation()`；`toolpathFeasible()` 整条边可达性预检。 |
| `include/rosim_planning/ompl_planner.h`、`src/ompl_planner.cpp` | `OmplPlanner::plan()`：pluginlib 加载 `ompl_interface/OMPLPlanner`，构造 `PlanningContext`，求解 `PlanRequest`，失败返回原因；成功后 TOTG 参数化。 |
| `include/rosim_planning/line_planner.h`、`src/line_planner.cpp` | `LinePlanner`（注册名 `"line"`）：笛卡尔直线规划，逐步 IK + 碰撞检查 + 关节跳变门限；确定性、毫秒级。 |
| `include/rosim_planning/fallback_planner.h`、`src/fallback_planner.cpp` | `FallbackPlanner`：装饰器。先试主规划器，失败（或位移超 `max_delta`）则整体回退到兜底规划器，并在结果消息里注明 `... fell back`。 |
| `include/rosim_planning/hybrid_planner.h`、`src/hybrid_planner.cpp` | `HybridPlanner`（注册名 `"hybrid"`，H3）：RL 出初解 → 逐点校验 → **保留无碰撞前缀**，用 OMPL 修掉后半段。与 FallbackPlanner 的区别是「不丢弃 RL 已走通的部分」；四种走向都写进 `lastNote()`。 |
| `include/rosim_planning/trajectory_timing.h`、`src/trajectory_timing.cpp` | `regularizeTiming()`：最短段钳制 + 50 ms 均匀网格 + 速度填充，保证 JTC 能平滑执行。 |
| `include/rosim_planning/motion_planner.h`、`src/motion_planner.cpp` | `MotionPlanner` 抽象与 `MotionPlannerFactory` 注册表；`"ompl"` 内置，`"rl"` 由 dlopen 的 `librosim_rl.so` 注册。 |
| `src/main.cpp` | 多线程 `MultiThreadedExecutor`：Action worker 可长时间规划，service 仍可响应取消/状态查询。 |
| `tools/home_probe.cpp` | 候选 home 位姿自碰撞与 TCP 位置测量。 |
| `tools/ik_probe.cpp` | 单个 TCP 位姿多种子 IK，枚举解分支。 |
| `tools/edge_scan.cpp` | 扫描工件每条边的接近位姿可达性，批量预筛。 |
| `tools/traverse_probe.cpp` | 逐路点输出 OK / COLLIDE / NO-IK 诊断。 |
| `tools/acm_probe.cpp` | 空 ACM 扫描自碰撞对，生成 SRDF `disable_collisions`（换机器人/网格后必须重跑）。 |

### 3.5 rosim_server

| 文件 | 作用与关键函数 |
| --- | --- |
| `include/rosim_server/server_node.h`、`src/server_node.cpp` | 服务端节点。`init()` 加载模型、建渲染器、启动 WS 与渲染线程；`onWsMessage()` 分发全部浏览器消息；`handleImport()` 转发规划服务后本地再做一次显示三角化并缓存；`handleFollow()` 发 Action 并转发 feedback/result；`buildRobotScene()` 把 URDF 碰撞形状序列化为 `scene`；`workpieceJson()` 输出 Base64 网格 + 边折线；`robotStateJson()` 12 Hz 输出连杆位姿；`broadcastScene()` / `broadcastWorkpiece()`；`renderLoop()` 按订阅者数量决定是否抓 JPEG、是否序列化位姿；`applyPendingJointState()` 用 `/joint_states` 更新显示状态。 |
| `include/rosim_server/ws_server.h`、`src/ws_server.cpp` | Boost.Beast HTTP/WebSocket。启动时把 `web_root` 下页面与 three.js 资产读入内存（**改 HTML 后必须重启**）；路由 `/` → `index.html`，`/production` → `production.html`，`/vendor/*` 静态资源；广播目标 `All` / `Stream` / `Scene` / `Workpiece`；`broadcastBinary()` 仅在 `subscribe_stream` 后发送 JPEG；所有广播线程安全。 |
| `include/rosim_server/frame_renderer.h`、`src/frame_renderer.cpp` | 离屏 VTK 渲染。`init()` 建 `vtkXOpenGLRenderWindow` 与灯光/地面；`updateRobot()` 推 FK；`addWorkpiece()` / `removeWorkpiece()` / `clearWorkpieces()`；`setHighlightedEdge()`；`captureJpeg()` 用 `vtkWindowToImageFilter` + `vtkJPEGWriter` 在内存编码。X11 类型只在 .cpp 中出现（PIMPL），避免与 rclcpp 宏冲突。 |
| `src/server_main.cpp` | 入口：在建首个 GL 上下文前设置 swap 行为，避免合成器低频刷新导致离屏渲染卡顿；随后 spin 节点。 |
| `launch/server.launch.py` | 当前主入口（见 3.2）。 |
| `web/index.html` | 调试页。three.js 场景、选框/拖动工件、边列表、跟随/运动/参数/日志。n 型布局：顶部状态栏 + 左栏（导入、边列表）+ 中间 3D + 右栏（跟随、运动、参数、日志）；窄屏自动上下堆叠。关键函数：`initViewer()`、`onScene()`、`onWorkpiece()`、`onRobotState()`、`highlightEdge()`、`pick()`、`poseFromInputs()`、`beginWorkpieceMove()`、拖动相关函数、`connect()` / `onJson()`。 |
| `web/production.html` | 生产页。只订阅工件几何；焊缝列表选择、自动定位、状态/进度、开始/停止。不显示机器人、不允许在模型上误点选边。 |
| `web/viewer_common.js` | 两个页面共享：`decodeMeshData()`（Base64 → 顶点/索引）、`buildCreasedGeometry()`（30° crease 法线）、`poseToMatrix4()`、`buildEdgeMarker()`（管 + 端点 + 标签）。 |
| `web/vendor/` | `three.min.js`、`OrbitControls.js`。 |
| `tools/render_probe.cpp` | 离屏 VTK 渲染冒烟测试。 |

### 3.6 rosim_rl

| 文件 | 作用 |
| --- | --- |
| `src/rl_remote_planner.cpp` | `RlRemotePlanner : MotionPlanner`，注册名 `"rl"`。**不做本地推理**：把起点/目标按变量名映射成策略的固定顺序（`[rail_joint, j1..j6]`；6 维请求表示"只有手臂"，导轨保持起点值），发给 `/rl_plan` 服务（`rosim_interfaces/srv/RlPlan`），拿回关节序列后自己拼轨迹、做 TOTG 与 FCL 逐状态校验。服务客户端必须挂在 `Reentrant` 回调组——`plan()` 阻塞等结果时，默认的 `MutuallyExclusive` 组会把响应回调一起堵死，必然超时。 |

推理侧在 Python（服务端是 `scripts/rl_plan_node.py`，观测/归一化复用 `rl_a/` 的训练代码）：

| 文件 | 作用 |
| --- | --- |
| `scripts/rl_plan_node.py` | `/rl_plan` 服务端。`onnxruntime` 加载策略，`kin` 复用 `rl_a/kin.py` 的运动学，因此不必在 C++ 里重写一遍观测拼接。 |
| `rl_a/kin.py` | 由 URDF 重建的运动学（含导轨 + 0.167 m 基座偏移）与胶囊碰撞模型，训练与推理共用同一份。 |

> **设计取舍**：观测构造留在 Python，是为了消掉一整类 sim2sim 偏差（归一化统计、
> 间隙模型、关节缩放）；C++ 只保留必须与真实场景严格一致的部分——轨迹拼装、
> 碰撞校验、时间参数化。

### 3.7 rosim_viewer（保留的桌面 GUI）

| 文件 | 作用 |
| --- | --- |
| `src/main.cpp` | Qt 应用入口，创建 ROS 节点与 `MainWindow`。 |
| `src/main_window.cpp` | 左侧控制面板（导入、边树、跟随参数、速度、回零、微调）+ 中央 3D。 |
| `src/scene_view.cpp` | VTK 视口、地面/坐标轴、工件与边 actor、边悬停/点选交互。 |
| `src/ros_bridge.cpp` | ROS ↔ Qt：独立 executor 线程，回调转 Qt 信号。 |
| `src/robot_assembly.cpp` | 从 `robot_description` 建连杆 actor，按 `/joint_states` 做 FK。 |
| `launch/viewer.launch.py` | 单独启动 Qt viewer（假定 planning 与控制器已在运行），与 bringup 内嵌启动等价。 |
| `tools/crash_probe.cpp`、`robot_pose_probe.cpp` | 崩溃/位姿诊断工具。 |

> 当前主界面为浏览器 UI；`rosim_viewer` 仅在 `robot_bringup.launch.py use_viewer:=true`
> 时使用，不参与服务器链路。

---

## 4. 关键流程与算法

### 4.1 工件导入

1. WebUI 发送 `{"type":"import", "file_path":..., "pose":...}`。
2. `ServerNode::handleImport()` 调用 `/rosim_planning/import_model`。
3. `PlanningNode` 用 `OccGeometryService::importModel()` 加载，枚举边并以
   `toCollisionMesh()` 注入 `PlanningScene`。
4. 服务端再次调用 OCC 对显示网格做三角化，缓存 `workpiece_mesh_` / `workpiece_edges_`，
   广播 `workpiece`（Base64 网格 + 世界坐标边折线）与 `status`。
5. 浏览器 `onWorkpiece()` 重建工件实体、边线和选中标记。

> 直接 `ros2 service call .../import_model` 只更新规划节点；`rosim_server` 的显示缓存
> 不会更新，因此 UI 不显示新工件。需要 UI 同步时请走 WebUI 导入。

### 4.2 随边跟随管线（`EdgeFollower::execute`）

1. **真实起点**：从 `/joint_states` 取当前状态 `start`；它必须保持真值，用于所有
   `PlanRequest` 与发布轨迹首点。
2. **边重采样**：按 `max_waypoint_spacing`（默认 5 mm）沿弧长取点。
3. **导轨站位分配**：`railStationFor()` 由边的位置估算车架 X 站位，写入独立的
   `ik_seed`（不能改 `start`，否则首航点与控制器当前状态不一致，导轨会瞬移）。
4. **TCP 帧生成**：`EdgeFrameGenerator::generate()`，TANGENT 默认带 0.6 rad 导前角。
5. **接近位姿 IK**：`solveIkMultiSeed()` 多种子求解 `tool_tip`（不是 `tool0`，否则
   焊枪会埋进工件）；必要时翻转倾角方向并 `toolpathFeasible()` 整条边预检。
6. **OMPL 预接近**：`manipulator`（7 轴）从真实状态规划到接近位姿，**含导轨转移**。
7. **笛卡尔沿边**：4 mm 平移步长，有效性回调 `!scene->isStateColliding()`；
   若截断则按 `max_relocations` 尝试换解重连。
8. **退刀**：在工具轴方向/世界上方多个距离候选中，按整段插入路径（4 个中间位姿）
   是否无碰来挑选；没有候选时停在焊缝终点。
9. **末端安全间隙**：把末端 clearance < 1.5 mm 的路点裁掉（退化到 0.5 mm）；若有裁剪，
   从新末端 OMPL 退到 4–25 cm 外的自由空间，避免以“贴住工件”的状态结束
   （否则后续 GoHome 会因 OMPL `START_STATE_INVALID` 失败）。
10. **合并与 TOTG**：预接近 + 沿边 + 退刀合并为一条 `RobotTrajectory`，统一时间参数化，
    发布并监控执行。

### 4.3 导轨（第 7 轴）

- 物理：2.8 m 型材导轨，行程 ±1.05 m，车架 300×240 mm，安装板把机器人基座抬到
  z=0.167 m。
- 规划：`rail_joint` 属于 `manipulator` group，OMPL 采样覆盖 7 维；`arm` group 仅 6 维，
  保证 KDL 与 Cartesian 求解器是方阵。
- 站位：随边时按目标 X 选一个舒适站位；点到点 `MoveToPose` 先尝试当前站位，再以
  0.25/0.5 m 步进换站。
- 执行：JTC 同时控制 7 个关节；轨迹首点必须等于真实状态，否则控制器会在第一个
  控制周期把导轨吸附到首航点（“瞬移”）。

### 4.4 执行与监控

- `publishTrajectory()` 发布 `trajectory_msgs/JointTrajectory` 到 `/arm_controller/joint_trajectory`。
- `monitorExecution()` 以 `/joint_states` 与轨迹末点比较（容差 0.003，混合 rad/m），
  取消时置位 `cancel_`；超时/取消返回失败。
- `speed_scale` 作用于时间参数化后的轨迹时间戳。
- mock 硬件按 JTC 插值输出 `/joint_states`，因此 UI、TF、碰撞状态完全同步。

### 4.5 Web 渲染与交互

- **调试页**：`scene` 一次性下发 URDF 碰撞几何（box/cylinder/sphere/mesh），
  `robot_state` 以 12 Hz 下发连杆位姿，浏览器直接改 three.js 矩阵（无重传网格）。
- **生产页**：只订阅 `workpiece`，三点选边→高亮→开始焊接；不允许在模型上点选，
  防止误触改动作业对象。
- **服务端渲染**：无人订阅 `stream` 时跳过 GL 抓帧；无人订阅场景时跳过位姿序列化。
- **布局**：调试页为小写 n 型（顶部栏 + 左右两列 + 中间视图），窗口 < 1000 px 时
  自动改为“视图 / 左栏 / 右栏”纵向堆叠。

---

## 5. WebSocket 协议（端口 8765）

客户端 → 服务端（JSON 文本帧）：

| `type` | 字段 | 说明 |
| --- | --- | --- |
| `subscribe_scene` / `unsubscribe_scene` | — | 订阅机器人场景 + 工件 + 位姿 |
| `subscribe_workpiece` / `unsubscribe_workpiece` | — | 只订阅工件几何（生产页） |
| `subscribe_stream` / `unsubscribe_stream` | — | 订阅服务端 JPEG 二进制帧 |
| `import` | `file_path`, `pose` | 导入工件 |
| `list_dir` | `path?` | 列出目录下的模型文件（页面上的文件选择器用；`path` 省略时从上次导入位置开始） |
| `select_edge` | `edge_index` | 选中边并高亮 |
| `follow` | `edge_index`, `orientation_mode?`, `tangent_tilt?`, `tool_offset?`, `approach_distance?`, `max_waypoint_spacing?` | 发起随边跟随 |
| `stop` | — | 取消当前 Action |
| `go_home` | — | 回 home |
| `move_to_pose` | `pose{position, orientation}` | `tool0` 点到点 |
| `set_workpiece_pose` | `pose` | 移动工件 |
| `jog_joint` | `joint_name`, `delta` | 关节/导轨微调 |
| `jog_cartesian` | `axis`, `delta` | 笛卡尔微调：`axis` 0~5 对应平移 xyz / 旋转 rpy，`delta` 单位米或弧度 |
| `set_speed` | `scale` | 速度倍率 |
| `set_planner` | `planner_type` | `ompl`（几何兜底）/ `line` / `auto`（默认）/ `rl` / `hybrid`，见 §2.1 的组合表 |
| `plan_demo` | `trials?`, `delta?` | 用当前规划器规划到关节目标并画出 TCP 路径；`trials` 取 1~20 时跑批量验证，汇报成功率与耗时（`delta` 是 6 维关节增量） |
| `clear_tcp_path` | — | 清空 3D 视图里的 TCP 轨迹 |
| `get_state` | — | 请求当前状态快照 |
| `create_seam` | `edge_indices[]`, `tolerance?`, `name?` | 把相连的边合并成一条焊缝（服务端调 `~/create_seam` 校验） |
| `delete_seam` / `rename_seam` | `id` / `id, name` | 删除 / 重命名焊缝 |
| `select_seam` | `id` | 选中焊缝（高亮合并折线） |
| `reorder_seams` | `order[]` | 调整执行顺序（拖拽） |
| `follow_seams` | `ids[]`, 姿态/逼近参数 | 多条焊缝一次连续规划执行（跳过失败项并汇总） |

服务端 → 客户端：

| `type` | 主要字段 | 说明 |
| --- | --- | --- |
| `status` | `ok`, `message` | 操作结果/错误 |
| `state` | `model_id`, `file_path`, `selected_edge`, `speed_scale`, `planner`, `edges` | 状态快照 |
| `edges` | `model_id`, `edges[]` | 导入后边列表 |
| `scene` | `robot[]`（`link` 名称 + `shapes[]`；每个 shape 含 `kind`、尺寸参数、`data`（mesh）、`pos`、`quat`） | 机器人静态几何 |
| `workpiece` | `model_id`, `pos`, `quat`, `mesh{vcount,tcount,data(base64)}`, `edges[{index,length,closed,points}]` | 工件显示数据 |
| `robot_state` | `links[{name,pos,quat}]` | 12 Hz 连杆位姿 |
| `seams` | `seams[]{id,name,length,closed,parts[],polyline[],last_result,last_run}` | 焊缝列表（含上次执行结果） |
| `seams_feedback` | `progress`, `seam_index`, `seam_count`, `phase` | 批量执行进度 |
| `seams_result` | `success`, `executed`, `failed`, `failed_ids[]`, `failed_messages[]` | 批量执行结果（失败被跳过并汇总） |
| `follow_feedback` | `progress`, `phase` | 跟随进度 |
| `follow_result` | `success`, `error_code`, `message` | 跟随结果 |
| 二进制帧 | JPEG bytes | 仅 `subscribe_stream` 客户端 |

---

## 6. 机器人模型与配置要点

- **运动链**：
  `rail_base_link → rail_joint(prismatic X) → rail_carriage → base_link → j1(Z) →
  link1 → j2(Y) → link2 → j3(-Y) → link3 → j4(-X) → link4 → j5(-Y) → link5 →
  j6(-X) → flange → tool0 → tool_tip`。
- **TCP**：`tool_tip` 为焊丝末端，不是法兰 `tool0`；所有随边 IK 都求解 `tool_tip`。
  焊枪为 ESAB RM62（36° 鹅颈）真实网格，带碰撞体；枪身会与工件/夹具发生碰撞。
- **home**：`j1=0, j2=0, j3=-0.5, j4=0, j5=0.3, j6=0, rail=0`，TCP 约
  `(1.251, 0, 0.928)`，比默认工件顶面高约 0.2 m；旧的低位 home（TCP z=0.525）
  会与工件相交，导致 GoHome 拒绝。
- **SRDF group**：`arm` 供 KDL/Cartesian；`manipulator` 供 OMPL（含导轨）。
- **ACM**：`disable_collisions` 由 `acm_probe` 实测生成；漏掉相邻对会让 KDL 所有
  种子失败。导轨与机器人之间的接触对保留真实碰撞（伸到导轨下方是真碰撞）。
- **控制器**：`arm_controller` 为 7 关节 JTC；mock 硬件初始值与 home 一致。
- **OMPL**：默认 `RRTConnectkConfigDefault`；`longest_valid_segment_fraction=0.005`，
  投影 `j1,j2`。

---

## 7. 编译

依赖：ROS 2 Humble、MoveIt2（建议 `~/ws_moveit` 源码 overlay）、OpenCASCADE、
VTK 9.1（含 RenderingOpenGL2 / IOImage / IOGeometry）、Qt5（仅旧 viewer）、
Boost（system）、Eigen3、`ros2_control` / `ros2_controllers`。

```bash
cd ~/rosim_ws
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
colcon build --symlink-install
```

可选 RL 策略推理：

```bash
colcon build --symlink-install --cmake-args -DROSIM_WITH_TORCH=ON
```

> ⚠️ 必须在工作区根目录 `~/rosim_ws` 执行 `colcon build`。在子目录或 `~` 下执行会让
> colcon 递归发现包并在错误位置生成 `build/`、`install/`、`log/`（源码树中的
> `COLCON_IGNORE` 只是保护，不产生正确产物）。构建后可用
> `strings install/rosim_planning/lib/librosim_planning_core.so | grep '<新日志>'`
> 验证产物确实更新。

---

## 8. 运行

### 8.1 浏览器模式（当前主流程）

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/rosim_ws/install/setup.bash
ros2 launch rosim_server server.launch.py
```

- 调试模式：<http://localhost:8765/>
- 生产模式：<http://localhost:8765/production>
- 可选 RViz：`ros2 launch rosim_server server.launch.py use_rviz:=true`

启动内容：`robot_state_publisher` → `ros2_control_node`（mock）→
`joint_state_broadcaster` / `arm_controller` → `rosim_planning` → `rosim_server`。
**不启动 move_group**。

> 服务端在启动时把 `web/` 页面读进内存；修改 HTML/JS 后需要重启节点。

> **需要把 HOME 隔离到仓库内时**（多机部署、无人值守、以及本仓库的沙箱环境），
> 不要直接 `ros2 launch`，用包装脚本：
>
> ```bash
> bash scripts/run-server.sh        # HOME → .scratch/home，ROS 日志与 seams.json 都落在这里
> ```
>
> 它做的事就是 source 三个 setup + 设 `HOME` / `ROS_HOME` / `ROS_LOG_DIR` 再 exec
> launch；直接 launch 会把日志写到 `~/.ros`，在受限环境里可能不可写。

### 8.2 旧版桌面 GUI / RViz

```bash
# 默认启动 Qt viewer（use_viewer:=true）
ros2 launch rosim_description robot_bringup.launch.py
# 无桌面的机器上只开 RViz
ros2 launch rosim_description robot_bringup.launch.py use_viewer:=false use_rviz:=true
```

关闭 viewer 窗口会级联关闭整个 launch。无 `DISPLAY` 的环境请用浏览器模式或 RViz。

---

## 9. 使用示例

### 9.1 导入工件

WebUI：在调试页左栏填写 STEP 路径与位姿，点“导入”。默认示例：

```text
路径：~/rosim_ws/src/rosim_description/meshes/workpiece_sample.step
位置：(1.0, 0, 0.6)
姿态：RPY (0, 0, 0)
```

CLI（仅更新规划节点，不刷新 UI 显示缓存）：

```bash
ros2 service call /rosim_planning/import_model rosim_interfaces/srv/ImportModel \
  "{file_path: '$HOME/rosim_ws/src/rosim_description/meshes/workpiece_sample.step',
    world_pose: {position: {x: 1.0, y: 0.0, z: 0.6},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 9.2 随边跟随

在调试页左栏“识别边列表”中选择边（或直接点击模型上的焊缝），设置**运行方向**、
姿态模式、导前角、退刀、逼近段与采样间距后点“跟随”。生产页从焊缝列表选边后点“开始焊接”。

**运行方向**（`direction`）：`0` AUTO（正向优先，正向不可达才反向，历史行为）、
`1` 强制正向、`2` 强制反向。调试页选中边后会在起焊端画一个绿点，切换方向时绿点
跟着换端；结果里 `direction_used` 回报实际采用的方向（0 正向 / 1 反向 / 255 未定）。
强制方向不可达时返回 `ERROR_IK` 并提示改用 AUTO 或反方向 —— **不会静默换向**。

CLI 示例（边号以当前列表为准；默认摆位下 edge 4 为一条 0.3 m 直边）：

```bash
ros2 action send_goal /rosim_planning/follow_edge rosim_interfaces/action/FollowEdge \
  "{model_id: 'workpiece_1', edge_index: 4, orientation_mode: 0,
    tangent_tilt: 0.6, tool_offset: 0.0,
    approach_distance: 0.08, max_waypoint_spacing: 0.005}"
```

`orientation_mode`：`0` TANGENT（工具 +Z 沿切线，默认导前角 0.6 rad）、
`1` FIXED（恒定姿态，需给 `fixed_orientation`）、`2` NORMAL（预留）。

结果字段：`success`、`error_code`（0 无错、1 IK、2 OMPL、3 Cartesian、
4 碰撞、5 取消、6 参数非法）、`cartesian_fraction`、`direction_used`、`message`。

### 9.3 点到点 / 回零 / 微调 / 参数

```bash
# tool0 到 (2.0, 0.0, 0.5)，姿态为单位四元数；必要时自动换导轨站位
ros2 service call /rosim_planning/move_to_pose rosim_interfaces/srv/MoveToPose \
  "{target_pose: {position: {x: 2.0, y: 0.0, z: 0.5},
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

ros2 service call /rosim_planning/go_home rosim_interfaces/srv/GoHome "{}"

# 导轨前进 0.15 m（单次最多 0.2 m）
ros2 service call /rosim_planning/jog_joint rosim_interfaces/srv/JogJoint \
  "{joint_name: 'rail_joint', delta: 0.15}"

ros2 service call /rosim_planning/set_speed_scale rosim_interfaces/srv/SetSpeedScale "{scale: 0.3}"
ros2 service call /rosim_planning/set_planner rosim_interfaces/srv/SetPlanner "{planner_type: 'ompl'}"
```

执行中取消：调试页“停止”按钮，或 Action client 取消 goal（返回 `ERROR_CANCELED=5`）。

---

## 10. 调试工具（rosim_planning / rosim_core）

```bash
ros2 run rosim_planning home_probe      # 候选 home 位姿自碰撞 / TCP 位置
ros2 run rosim_planning ik_probe        # 单 TCP 位姿多种子 IK，枚举解分支
ros2 run rosim_planning edge_scan       # 扫描工件所有边的接近可达性
ros2 run rosim_planning traverse_probe  # 逐路点 OK / COLLIDE / NO-IK 诊断
ros2 run rosim_planning acm_probe       # 空 ACM 扫自碰撞对 -> SRDF disable_collisions
ros2 run rosim_core gp12_prepare <stp> probe|export <dir>   # CAD 轴心提取 / STL 导出
```

多数工具需要 `robot_description` 与 `kinematics.yaml` 参数，见各文件头注释。

### 10.1 回归测试（`edge_regress`）

改动规划逻辑前先跑一份基线，改完再跑一遍对比，避免"某条边悄悄从能走变成截断"。

```bash
# 记录基线（全部边，每条边之后回 home，保证与顺序无关）
ros2 run rosim_planning edge_regress -- \
    --step ~/rosim_ws/src/rosim_description/meshes/workpiece_sample.step \
    --out ~/rosim_ws/.scratch/regress/baseline.json

# 快速子集
ros2 run rosim_planning edge_regress -- --edges 4,17,22 --out /tmp/subset.json

# 与基线对比（有退化时退出码 1，可直接接 CI）
ros2 run rosim_planning edge_regress -- --out /tmp/new.json --compare ~/rosim_ws/.scratch/regress/baseline.json
```

输出每条边的 `success` / `error_code` / `cartesian_fraction` / 耗时；对比模式会列出
"新增失败""fraction 漂移""平均耗时变化"。内置基线（2026-10-19，示例工件 × 默认摆位）：
54 条边中 40 条成功，失败集中在 20–40 mm 倒角与闭合圆边（均为笛卡尔截断，非 IK/OMPL 失败）。

---

## 11. RL 扩展

- `librosim_rl.so` 在规划节点启动时按参数 `planner_libraries` dlopen；静态注册
  `"rl"` 到 `MotionPlannerFactory`。
- `set_planner("rl")` 后，Action/service 的下游执行与 OMPL 完全一致。
- 推理不在本进程内：`RlRemotePlanner` 调 `/rl_plan` 服务，服务端是
  `scripts/rl_plan_node.py`（`onnxruntime` + `rl_a/` 的训练代码）。**必须先起这个
  节点**（`./scripts/rl_plan.sh`），否则 `plan()` 会因服务不可用而快速失败。
- 服务不可用 / 策略未加载时 `plan()` 返回明确原因，不会阻塞 OMPL。
- `hybrid` 入口（H3）：RL 出初解 → 校验 → 保留无碰撞前缀 → OMPL 修后半段。
  想比较"整段回退"和"部分保留"的差异，就在 `rl` 与 `hybrid` 之间切换。

---

## 12. 已知限制与注意事项

1. **并非所有边都可达**。TANGENT 倾斜枪的扫掠面比 FIXED 大，摆位甜区外常见
   `fraction 0.5–0.6` 截断（j4/j5 限位或前臂/枪身扫件），属几何必然；可用
   `edge_scan` 预筛或改 FIXED。
2. **起点必须无碰**。OMPL 对接触/碰撞起点返回 `START_STATE_INVALID (-26)`；
   跟随结束时若末端 clearance < 1.5 mm，管线会先裁剪再 OMPL 退刀，保证下一个动作
   有干净起点。
3. **首航点必须等于真实状态**。JTC 若发现首点 `time_from_start=0` 且与当前状态
   不一致，会在第一个控制周期吸附过去（导轨“瞬移”）。因此 `EdgeFollower` 始终用
   真实 `start` 规划，另用独立的 `ik_seed` 承载目标站位。
4. **服务端显示缓存与规划节点分离**。通过 CLI service 导入不会更新 WebUI；
   请使用页面导入或先调 service 再刷新页面（仍不会显示，因为缓存未更新）。
   需要 UI 显示时必须走 `rosim_server` 的 WS `import`。
5. **Web 资产启动时缓存**。修改 HTML/JS 后重启 `rosim_server`。
6. **无头渲染依赖 GL**。当前 `FrameRenderer` 使用 `vtkXOpenGLRenderWindow`，
   需要 X/GLX；在无 X 的加速卡上换成 `vtkEGLRenderWindow`（并保留
   `vtk_module_autoinit`）。没有订阅者时渲染线程会跳过抓帧。
7. **无物理仿真**。mock 硬件只做轨迹插值，碰撞检查发生在规划阶段；执行阶段不会
   重新做碰撞检测。
8. **焊丝尖端没有独立碰撞体**。碰撞以焊枪网格（含枪体/喷嘴）为准。
9. **性能基线**：GP12 约 35 万三角网格 + FCL 下，多种子 IK 约 36 ms/解
   （甜区位姿 30 次约 1.1 s）；若网格加细导致变慢，可回退到 4 mm 粗网格单独喂碰撞。
10. **工作区整洁**：不要在源码目录内生成 `build/`、`install/`、`log/`；本仓库已清理
    历史遗留的此类目录。

---

## 13. 验证记录（当前基线）

- 默认工件 `(1.0, 0, 0.6)`，最长边跟随：`fraction = 1.0`，末端间隙 4.4 mm，
  随后 `go_home` 成功（46 航点），机器人回到 home、导轨回 0。
- 导轨参与跟随：0 → 0.717 m，最大单步 4.7 mm / 10 ms（0.47 m/s，低于 1.2 m/s
  限速），运动持续约 3.1 s，无跳变。
- 点到点 + 导轨换站、导轨 jog + 回零、生产页选边/跟随均通过回归。
- 调试页布局在 1600×900 与 900 px 宽度下验证：三栏/堆叠均无重叠，3D 视图正常。
