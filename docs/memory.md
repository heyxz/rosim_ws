# 跨机共享记忆（飞书 / GUI / 笔记本 三方共用）

> 本文件由 `dsh-bridge` 自动维护下方**标记区块**内的内容；区块外的文字不会被覆盖。
> 硬上限 **40 条 / 12 KB**（滚动窗口），重要结论请另存 `docs/memory/archive.md`。
> 两台机器通过 git 同步：开工前 `git pull --rebase`，写完 `bash scripts/memory_sync.sh`。

# 共享记忆（自动维护）

> 由 dsh-bridge 写入：把「飞书」和「GUI」两条通道最近的对话摘要合并到下面。
> 两个通道的会话每轮都会自动读到最新版本，所以不用手动同步。
> 只改下面标记区块内的内容；区块外你可以随便写，不会被覆盖。

<!-- dsh-memory:start -->
- 2026-10-29 00:22 [GUI] agent：`launch` 目录在主力机上**是空的**（git 不存空目录），所以笔记本上根本没有 → `install(DIRECTORY launch)` 失败。这是仓库的真实 bug，两边都会踩。修 CMake：
- 2026-10-29 00:23 [GUI] agent：主力机 `2.3.2`，笔记本装到了 `2.3.4`——**soname 不一致**，这是预编译件移植的典型坑。先把编译跑完，再实测能不能跑：
- 2026-10-29 00:26 [GUI] 你：background job bash-91 (bash: sleep 240; timeout 25 ssh -o BatchMode=yes laptop 'cd ~/rosim_ws && ls install 2>/dev/null | tr "\n" " "; echo…
- 2026-10-29 00:26 [GUI] 你：background job bash-98 (bash: timeout 900 ssh -o BatchMode=yes laptop "cd ~/rosim_ws && git pull --rebase -q 2>&1 | tail -2; source /opt/ros…
- 2026-10-29 00:26 [GUI] agent：`rosim_planning` 过了 ✅ 下一个是 `rosim_rl`——大概率同类问题：
- 2026-10-29 00:26 [GUI] agent：同类问题：`rosim_rl/include/` 也是空目录。**系统性扫一遍**，一次修完：
- 2026-10-29 00:29 [GUI] 你：background job bash-105 (bash: timeout 900 ssh -o BatchMode=yes laptop "cd ~/rosim_ws && git pull --rebase -q 2>&1 | tail -2; source /opt/ro…
- 2026-10-29 00:29 [GUI] 你：background job bash-99 (bash: sleep 240; timeout 25 ssh -o BatchMode=yes laptop 'cd ~/rosim_ws && ls install 2>/dev/null | tr "\n" " "; echo…
- 2026-10-29 00:30 [GUI] agent：编译**全部通过**（7 个包）✅ 现在验最关键的：预编译 MoveIt 在运行时能不能加载（soname 差异）：
- 2026-10-29 00:30 [GUI] agent：确认撞上 ABI 墙：`libgeometric_shapes.so.2.3.2` 找不到（笔记本是 2.3.4）。先看能不能装回旧版：
- 2026-10-29 00:30 [GUI] agent：旧版 apt 里没有了。试**直接把主力机那个 2.3.2 的 so 拷过去**（208 KB，MoveIt 就是针对它编译的）：
- 2026-10-29 00:31 [GUI] agent：🎉 **成功了**——`ldd` 无缺失，`ik_probe` 真的跑起来了（它报"找不到 robot_description"是正常的，说明 MoveIt 已加载）：
- 2026-10-29 00:31 [GUI] agent：笔记本环境**跑通了** ✅ 提交 `0a17363`，文档在 [docs/laptop_env.md](docs/laptop_env.md)。 ## 最终验证结果 ``` colcon build → 7 个包全部通过（2.5 分钟） …
- 2026-10-29 00:31 [GUI] 你：background job bash-106 (bash: sleep 270; timeout 25 ssh -o BatchMode=yes laptop 'cd ~/rosim_ws && echo "install: $(ls install 2>/dev/null |…
- 2026-10-29 00:31 [GUI] agent：两台都干净、都在 `69d60bc`。 ## 今晚这轮做完的事 | 项 | 结果 | | --- | --- | | 笔记本能编译 | ✅ 7 个包，2.5 分钟 | | 笔记本能加载 MoveIt | ✅ `ldd` 无缺失，`ik_pr…
- 2026-10-29 00:32 [GUI] 你：可以，跑一次
- 2026-10-29 00:32 [GUI] agent：服务在笔记本上起来了（8765 → 200，6 个节点全在，OMPL 已加载）。跑规划：
- 2026-10-29 00:33 [GUI] agent：缺 `ws` 模块（WebSocket 导入脚本要用）。装上再跑：
- 2026-10-29 00:34 [GUI] agent：导入通了（`workpiece_1`），但规划失败。看具体原因：
- 2026-10-29 00:34 [GUI] agent：规划成功但**执行失败**（`executed=0 failed=1`）。看服务端日志：
- 2026-10-29 00:35 [GUI] agent：这不是环境问题——是**真实的运动学失败**： ``` cartesian path incomplete (fraction 0.70), relocation 1 edge follow failed: edge not fully t…
- 2026-10-29 00:37 [GUI] agent：**3/3 全成功**（每个 37~40 秒，和主力机的 34~40 秒基本持平）——环境端到端验证通过 🎉 收尾：停掉测试服务、把 mDNS 抽风的事记进配置：
- 2026-10-29 00:38 [GUI] agent：端到端跑通了，收尾完成。 ## 最终验证：笔记本能独立跑完整规划链路 ``` ros2 launch rosim_server server.launch.py → 8765 = 200，6 个节点 ./scripts/workpiece_…
- 2026-10-29 00:39 [GUI] 你：明天再继续搞，我现在想确认，现在两台主机编译环境，运行环境基本一致，三端互通共享记忆也没有问题了么
- 2026-10-29 00:39 [GUI] agent：好问题，我用数据回答而不是印象。**两边各跑一次体检，直接 diff**：
- 2026-10-29 00:40 [GUI] agent：发现一个**真实缺陷**：笔记本 `db29b67`，主力机 `503c3a2`——记忆没同步过去。原因在 `memory_sync.sh`： ```bash if git diff --cached --quiet; then echo …
- 2026-10-29 00:41 [GUI] agent：直接说结论：**两个都不是 100% 干净，一个是"基本一致但有偏差"，一个是我刚修了一个真缺陷。** ## 一、编译/运行环境：核心一致，但有 8 个包版本不同 **完全相同**（决定能不能编译、能不能跑的关键项）： ``` gcc 11…
- 2026-10-29 00:42 [GUI] 你：然后笔记本的桥和harness是开机自启，不是的话改一下
- 2026-10-29 00:43 [GUI] agent：笔记本这边有两个问题，先把情况摸清：
- 2026-10-29 00:43 [GUI] agent：找到了**大问题**：笔记本的 harness 服务其实是个**僵尸**——它跑的是 `~/.local/share/Trash/files/smartweld`（回收站里的旧项目），而单元文件指向的 `/home/xz/smartweld…
- 2026-10-29 00:44 [GUI] agent：Harness 修好了（跑真实仓库、3080 正常）。现在处理桥——**你要求开机自启，但直接 enabled 会造成"两台同时回飞书"**。我加一个启动闸门同时满足两者：
- 2026-10-29 00:45 [GUI] agent：查出来的问题比"没自启"严重——**笔记本的 harness 服务指向的是回收站里的目录**。 ## 修之前的状态 | 服务 | 显示 | 实际情况 | | --- | --- | --- | | `dsh-web`（harness） | …
- 2026-10-29 00:46 [GUI] 你：可以，你帮我测试验证把，密码你也知道
- 2026-10-29 00:46 [GUI] agent：开始重启验证。先记基线，然后重启、等它回来、逐项核对：
- 2026-10-29 00:47 [GUI] agent：重启验证成功 ✅ 再等 2 分钟看记忆定时器是否自动追上（顺带验证修复后的 pull 逻辑）：
- 2026-10-29 00:50 [GUI] agent：重启验证**全部通过**。 ## 实测记录 ``` 重启前: boot 2026-10-28 21:00 up 3h46m HEAD c6c7030 重启: 00:46:54 发出 重连: 20 秒后 SSH 恢复 重启后: boot 20…
- 2026-10-29 00:52 [GUI] 你：你总结一下明天要做的事
- 2026-10-29 00:52 [GUI] agent：当前 `247b9d7`，工作区干净。明天的事按**优先级**排，你按时间挑。 --- ## P0 · 收尾（10 分钟，建议一开机就做） | # | 事项 | 命令 | 为什么 | | --- | --- | --- | --- | | …
- 2026-10-29 00:54 [GUI] 你：ok，晚安主力机小屁
- 2026-10-29 00:54 [GUI] agent：晚安 🌙 「主力机小屁」我收下了，虽然它今晚确实被笔记本抢了不少活。 ## 今晚的战果 **修掉了 6 个真问题**（都不是配置笔误，是设计/仓库缺陷）： | 问题 | 类型 | | --- | --- | | 空目录导致换机器编译必挂（…
<!-- dsh-memory:end -->
