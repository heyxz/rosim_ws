#!/usr/bin/env bash
# 笔记本初始化：一条命令把自己接入"三端通"
#
#   cd ~/rosim_ws && bash scripts/setup_laptop.sh
#
# 做四件事（幂等，可重复跑）：
#   1) 环境体检（只报告，不装东西）
#   2) 建立 AGENTS.local.md → docs/memory.md 软链，让 agent 读仓库里的共享记忆
#   3) 安装并启用"每 5 分钟同步记忆"的 systemd 用户定时器
#   4) 打印还差什么（dsh 版本、飞书桥、SSH）
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
say() { printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

say "1/4 环境体检"
bash scripts/check_env.sh || echo "（体检有未通过项，先按 docs/install.md 补齐，其余步骤仍可继续）"

say "2/4 共享记忆软链"
if [ -L AGENTS.local.md ] && [ "$(readlink AGENTS.local.md)" = "docs/memory.md" ]; then
  echo "✅ 已存在：AGENTS.local.md -> docs/memory.md"
elif [ -e AGENTS.local.md ] && [ ! -L AGENTS.local.md ]; then
  cp -a AGENTS.local.md AGENTS.local.md.bak
  ln -sf docs/memory.md AGENTS.local.md
  echo "⚠ 原 AGENTS.local.md 已备份为 .bak，并改建软链"
else
  ln -sf docs/memory.md AGENTS.local.md
  echo "✅ 已建立：AGENTS.local.md -> docs/memory.md"
fi

say "3/4 记忆同步定时器"
UD="$HOME/.config/systemd/user"
mkdir -p "$UD"
cp scripts/systemd/rosim-memory-sync.service scripts/systemd/rosim-memory-sync.timer "$UD/"
if systemctl --user daemon-reload 2>/dev/null && \
   systemctl --user enable --now rosim-memory-sync.timer 2>/dev/null; then
  echo "✅ 定时器已启用（每 5 分钟）"
  systemctl --user list-timers rosim-memory-sync.timer --no-pager 2>/dev/null | head -2
else
  echo "⚠ 无法启用 systemd 用户定时器（当前 shell 连不上 user bus？）"
  echo "   手动执行：systemctl --user daemon-reload && systemctl --user enable --now rosim-memory-sync.timer"
fi

say "4/4 还差什么"
echo "· dsh 版本（两台必须一致）："
printf "   本机: %s\n" "$(dsh --version 2>/dev/null | head -1 || echo '未安装')"
echo "    若与主力机不一致：npm i -g @deepseek-ai/dsh@0.2.0-rc.2 && bash scripts/patch_dsh_resume.sh"
echo "· 飞书桥（只有这台能跑）："
if [ -d dsh-bridge ]; then
  echo "    ✅ dsh-bridge/ 已存在；启动：systemctl --user enable --now dsh-bridge"
  echo "    注意：先在主力机执行 systemctl --user stop dsh-bridge，避免双重回复"
else
  echo "    ❌ 缺 dsh-bridge/（被 gitignore）。从主力机拷："
  echo "       # 主力机:"
  echo "       tar czf /tmp/dsh-bridge.tgz -C ~/rosim_ws dsh-bridge --exclude=dsh-bridge/node_modules --exclude=dsh-bridge/.dsh-bridge-home"
  echo "       scp /tmp/dsh-bridge.tgz laptop:~/"
  echo "       # 笔记本:"
  echo "       tar xzf ~/dsh-bridge.tgz -C ~/rosim_ws && cd ~/rosim_ws/dsh-bridge && npm install"
fi
echo "· SSH 双向（主力机看笔记本状态用）："
ssh -o ConnectTimeout=3 -o BatchMode=yes desktop true 2>/dev/null \
  && echo "    ✅ 能连到 desktop" \
  || echo "    未打通 → 见 docs/multi_host.md 4.4 节（ssh-copy-id）"

echo
echo "完成。验证：bash scripts/hosts.sh"
