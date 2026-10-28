# 多主机协作：笔记本常开网关 + 主力机夜间算力

> 目标：**飞书一个入口**，白天在笔记本上改码/编译/冒烟，晚上在主力机上训练/全量验证。
> 两端共用一份代码、一份记忆。

---

## 1. 拓扑

```
                      飞书（唯一对话入口）
                             │  WebSocket 长连接
                 ┌───────────▼────────────┐
                 │ 旧笔记本（常开·网关）     │
                 │  dsh 0.2.0-rc.2        │
                 │  dsh-bridge  ← 只有它跑 │
                 │  ROS 环境（轻活）        │
                 └───────────┬────────────┘
                    SSH（局域网）
                 ┌───────────▼────────────┐
                 │ 主力机（晚上开·算力）     │
                 │  dsh + GUI :3080        │
                 │  训练 / 基准 / GPU       │
                 └────────────────────────┘

  共享状态：git 仓库（代码 + docs/memory + docs/tasks）
  大文件：局域网 rsync（.pylibs / *.zip 不进 git）
```

## 2. 三条铁律

| # | 规则 | 违反的后果 |
| --- | --- | --- |
| 1 | **飞书桥只在一台机器上跑** | 飞书 SDK 是 WebSocket 长连接，两台同时连 → 同一条消息被两边都收到 → 双重回复、两个 agent 同时改代码 |
| 2 | **记忆只在仓库里**（`docs/memory.md`） | 两台各一份 `AGENTS.local.md` → 各说各话，记忆分裂 |
| 3 | **大文件不进 git**（`.pylibs` 5.3 GB、`best_model.zip`） | 仓库膨胀、clone 变慢、push 超时 |

---

## 3. 版本必须统一（关键）

`dsh-bridge` 依赖一个**官方没有的协议扩展**：`session/resume`。它是对
`@deepseek-ai/dsh-sdk-jsonrpc-server/lib/index.js` 的 31 行补丁（给 SDK 加
"从磁盘恢复会话"的能力）。**dsh 版本一变，补丁就要重打**。

| 机器 | dsh 版本 | 补丁 |
| --- | --- | --- |
| 主力机（当前） | 0.2.0-rc.2 | ✅ 已打 |
| 笔记本 | 0.2.1-rc.2 | ❌ 需处理 |

**推荐：两台都统一到 0.2.0-rc.2**（主力机已验证，补丁已验证最省事）：

```bash
# 笔记本上
npm i -g @deepseek-ai/dsh@0.2.0-rc.2
bash ~/rosim_ws/scripts/patch_dsh_resume.sh      # 幂等；--check 只检查
```

> 想升级到 0.2.1-rc.2 也可以，但**两台一起升**，然后各跑一次
> `patch_dsh_resume.sh`，并确认 `systemctl --user restart dsh-bridge` 后飞书能正常回话。

---

## 4. 笔记本部署清单

### 4.0 一条命令接入（推荐先跑）

```bash
git clone git@github.com:heyxz/smartwelding.git ~/rosim_ws && cd ~/rosim_ws
bash scripts/setup_laptop.sh
```

它会：环境体检 → 建立共享记忆软链 → 安装"每 5 分钟同步记忆"定时器 →
告诉你还差什么（dsh 版本 / 飞书桥 / SSH）。幂等，可反复跑。

下面是逐项的细节说明。

### 4.1 环境

按 [docs/install.md](install.md) 走，先体检：

```bash
git clone git@github.com:heyxz/smartwelding.git ~/rosim_ws
cd ~/rosim_ws && bash scripts/check_env.sh
```

> 笔记本性能弱的话，可以**不编译 `rosim_viewer`**：
> `colcon build --symlink-install --packages-skip rosim_viewer`
> 也可以**不装 `.pylibs`**（纯 OMPL/直线规划不需要），只做改码 + 编译 + 冒烟。

### 4.2 dsh + bridge

```bash
# dsh（版本要与主力机一致）
npm i -g @deepseek-ai/dsh@0.2.0-rc.2
bash ~/rosim_ws/scripts/patch_dsh_resume.sh

# bridge 目录（被 gitignore，从主力机拷）
# 主力机上执行：
tar czf /tmp/dsh-bridge.tgz -C ~/rosim_ws dsh-bridge \
    --exclude=dsh-bridge/node_modules --exclude=dsh-bridge/.dsh-bridge-home
scp /tmp/dsh-bridge.tgz laptop:~/
# 笔记本上：
mkdir -p ~/rosim_ws && tar xzf ~/dsh-bridge.tgz -C ~/rosim_ws
cd ~/rosim_ws/dsh-bridge && npm install
```

### 4.3 切换飞书桥（**必须先停主力机，再起笔记本**）

```bash
# 主力机：停掉并禁止自启
systemctl --user stop dsh-bridge
systemctl --user disable dsh-bridge
systemctl --user is-active dsh-bridge          # inactive

# 笔记本：装单元并启用（改 ExecStart/WorkingDirectory 为笔记本路径）
cp ~/rosim_ws/dsh-bridge/dsh-bridge.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now dsh-bridge
systemctl --user status dsh-bridge --no-pager | head -5
```

验证：给飞书机器人发「在么」，**只能有一条回复**。如果收到两条，说明主力机的
bridge 没停干净。

回滚：笔记本 `systemctl --user stop dsh-bridge`；主力机
`systemctl --user enable --now dsh-bridge`。

### 4.4 SSH 双向可达（局域网）

```bash
# 两台都装
sudo apt install -y openssh-server
# 两台各自生成/交换公钥
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519   # 已存在则跳过
ssh-copy-id laptop     # 在主力机上执行
ssh-copy-id desktop    # 在笔记本上执行
```

`~/.ssh/config`（两台都写，别名统一）：

```
Host desktop
    HostName 192.168.x.x
    User xz

Host laptop
    HostName 192.168.x.x
    User xz
```

验证：

```bash
./scripts/hosts.sh        # 两台状态看板
```

---

## 5. 日常怎么用

### 5.1 白天（主力机关着）

飞书里说：「把 XXX 改一下，编译并冒烟」。笔记本上的 agent：

```bash
cd ~/rosim_ws && git pull --rebase
# 改代码
colcon build --symlink-install
bash scripts/workpiece_bench.sh 1 auto     # 轻量验证
git add -A && git commit && git push
```

重活**不入队直接跑**，而是记进任务队列：

```bash
./scripts/tasks.sh add "训练 blocked_ratio=0.6 / 3M 步 → out_blocked3"
./scripts/tasks.sh add "20 位姿基准（hybrid）"
git add docs/tasks.md && git commit -m "tasks: 夜间任务" && git push
```

### 5.2 晚上（主力机开机）

```bash
cd ~/rosim_ws && git pull --rebase
./scripts/tasks.sh list            # 看白天排的队
./scripts/tasks.sh next            # 取最早一条
# 执行（示例：训练）
PYTHONPATH=$PWD/.pylibs:$PWD/rl_a python3 rl_a/train.py --env scene \
  --timesteps 3000000 --envs 8 --device cuda --out out_blocked3 \
  --bank scene_bank.npz --bank-ratio 0.3 --blocked-ratio 0.6
./scripts/tasks.sh done 1
git add -A && git commit -m "训练 out_blocked3" && git push
```

### 5.3 反过来看笔记本状态（你要的第 2 点）

主力机上（或在笔记本上互相看）：

```bash
./scripts/hosts.sh
```

输出示例：

```
──────── 主力机 (desktop) ────────
  主机: xz-desktop  运行: up 3 hours
  代码: c4852aa  未提交 0
  飞书桥: inactive
  rosim服务: active
  8765: 200
  磁盘: 286G 可用
  GPU: 0 %, 512 MiB, 24564 MiB
──────── 笔记本 (laptop) ────────
  主机: old-laptop  运行: up 12 days
  代码: c4852aa  未提交 2
  飞书桥: active
  rosim服务: inactive
  8765: -
  磁盘: 41G 可用
```

也可以在主力机上直接 `ssh laptop`，或从笔记本 `ssh desktop`
用 `./scripts/remote_run.sh desktop "命令"` 触发远端任务。

---

## 5.5 主备切换：用主力机时笔记本只待命

**目标**：同一时刻只有一台"值班"（跑飞书桥），另一台只同步记忆。

```
主力机开着 → 笔记本待命：飞书桥停、只跑记忆同步
主力机下线 → 笔记本接管：飞书桥起、正常干活
```

### 判断依据

能否 `ssh desktop true`（3 秒超时）。主力机开着一定通，关机/S3 一定不通。

### 防抖

连续 **2 次**（约 2 分钟）不可达才接管——避免主力机重启的那几分钟被抢。

### 命令

```bash
./scripts/role.sh status     # 当前角色 + 主力机是否在线
./scripts/role.sh auto       # 自动判断（定时器每分钟调这个）
./scripts/role.sh desktop    # 临时锁定：本机待命（比如你在主力机上干活）
./scripts/role.sh laptop     # 临时锁定：本机值班
./scripts/role.sh clear      # 解除锁定
```

定时器：`rosim-role.timer`（每分钟）。`setup_laptop.sh` 在 SSH 打通后会自动装上。

### 边界情况

| 情况 | 行为 |
| --- | --- |
| 主力机开着，但你在用飞书 | 笔记本待命，飞书由主力机回 → 正确 |
| 主力机关机 | 2 分钟后笔记本接管，飞书由笔记本回 → 正确 |
| 主力机睡眠（S3） | SSH 不通 → 同"关机"，笔记本接管 |
| 两台同时在线且都值班 | **不应该发生**；若发生说明定时器没跑，`role.sh desktop` 手动纠正 |
| 笔记本自己重启 | 定时器开机 1 分钟后跑第一次，按主力机状态决定 |

---

## 6. 记忆互通（已落地）

共享记忆现在**在仓库里**：`docs/memory.md`（本机 `AGENTS.local.md` 是指向它的软链）。

```
飞书（笔记本 bridge） ─┐
GUI / VSCode（任一机器）├─→ docs/memory.md ─→ git push/pull ─→ 另一端
笔记本本地 agent ─────┘
```

- `dsh-bridge` 已改为写 `docs/memory.md`（`feishu.mjs` 第 41 行）
- 本机 `AGENTS.local.md` → 软链 → `docs/memory.md`（agent 每轮自动读到）
- `scripts/memory_sync.sh`：只提交/推送这个文件（不碰其他改动）
- systemd 用户定时器 `rosim-memory-sync.timer`：**每 5 分钟**自动同步一次

### 6.1 笔记本上要做的（已封装）

```bash
cd ~/rosim_ws && bash scripts/setup_laptop.sh
```

等价于手动做两步：

```bash
ln -sf docs/memory.md AGENTS.local.md
cp scripts/systemd/rosim-memory-sync.{service,timer} ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now rosim-memory-sync.timer
```

> systemd 单元在仓库 `scripts/systemd/` 里，用 `%h` 表示家目录，两台机器通用。

### 6.2 大小

硬上限 **40 条 / 12 KB**（`MemoryBus` 的滚动窗口），不会膨胀。
长期结论写进 `docs/memory/archive.md`。

### 6.3 冲突处理

两台同时改记忆时会 rebase 冲突。`memory_sync.sh` 冲突时会 abort 并提示；
处理办法：`git checkout --theirs docs/memory.md` 或手工合并后重跑。
因为只追加、不删改，实际冲突概率很低。

---

## 6.5 在 VSCode 里对话（方案 A，与三端兼容）

**VSCode 只是网页的另一个窗口**，不是第四条通道——同一个 `dsh web` 服务、同一个会话、同一份记忆：

```
Ctrl+Shift+P → "Simple Browser: Show" → http://127.0.0.1:3080
```

- 左边写代码，右边对话，历史与会话都不变
- 三端通完全兼容：飞书（笔记本）、GUI（浏览器/VSCode）、笔记本 agent 共用 `docs/memory.md`
- 想省事：把 `dsh web` 设为 VSCode 任务（`.vscode/tasks.json` 已带）或用 `code --goto`

> 注意：`dsh web` 服务需要 `DISPLAY` 才能从 GUI 里唤起 VSCode 窗口，
> 已写入 `~/.config/dsh-web.env`；改动后要 `systemctl --user restart dsh-web` 生效。

## 7. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 飞书一条消息收到两次回复 | 两台 bridge 同时在线 | `systemctl --user stop dsh-bridge` 停掉旧的 |
| 笔记本上 dsh 起不来 | 版本/补丁不匹配 | 统一版本 + `patch_dsh_resume.sh` |
| `remote_run.sh` 报 Host key verification failed | 第一次连接 | 先手动 `ssh desktop` 接受密钥 |
| 主力机离线时 `hosts.sh` 报错 | 正常（晚上才开机） | 脚本已按"离线"处理，不算失败 |
| 训练结果在笔记本上看不到 | 大文件没进 git | 用 `rsync -av desktop:~/rosim_ws/rl_a/out_x/ rl_a/out_x/` |
| 两台都改了同一文件冲突 | 同时编辑 | 只在一台改；另一台开工前 `git pull --rebase` |

---

## 8. 相关脚本

| 脚本 | 作用 |
| --- | --- |
| `scripts/check_env.sh` | 环境体检（只读） |
| `scripts/patch_dsh_resume.sh` | 给 dsh 打 `session/resume` 补丁（幂等） |
| `scripts/tasks.sh` | 跨机任务队列（add/list/next/done） |
| `scripts/remote_run.sh` | 在另一台机器上执行命令（自动 source 环境） |
| `scripts/hosts.sh` | 两台机器状态看板 |
