#!/usr/bin/env bash
# 让"另一台机器上的 agent"干活：同步委派 + 结果回传。
#
#   ./scripts/peer.sh laptop "把 docs/laptop_env.md 第五节整理成表格"
#   ./scripts/peer.sh laptop --pull "跑 3 个位姿的基准，把结果写进 .scratch/"
#
# 机制：ssh 到目标机 → 载入 bridge 的 .env（里面有 API key）→ dsh headless "<task>"
#       → 该机的 agent 在它自己的沙箱里干活，返回最终文本。
#
# 注意：
#   1) 远端 agent **看不到当前对话**，任务描述必须自包含（它只能读仓库 + 共享记忆）
#   2) 每调用一次消耗一份 token 配额
#   3) 两台同时改同一个文件会冲突；约定：谁的任务谁负责，改动后各自 commit
set +u
HOST="$1"; shift || true
[ -z "$HOST" ] && { echo "用法: peer.sh <host> [--pull] <任务描述>"; exit 1; }
PULL=0
[ "$1" = "--pull" ] && { PULL=1; shift; }
TASK="$*"
[ -z "$TASK" ] && { echo "缺少任务描述"; exit 1; }

REMOTE=$(cat <<'EOS'
cd ~/rosim_ws || exit 1
set -a; . dsh-bridge/.env 2>/dev/null; set +a
export PATH="$HOME/.npm-global/bin:$HOME/.local/node/bin:$PATH"
EOS
)
[ "$PULL" = 1 ] && REMOTE="$REMOTE
git pull --rebase -q 2>/dev/null || true"

echo "── 委派给 $HOST ──"
ESCAPED=$(printf '%s' "$TASK" | sed "s/\\\\/\\\\\\\\/g; s/\"/\\\\\"/g")
timeout 900 ssh -o BatchMode=yes "$HOST" "$REMOTE
dsh headless \"$ESCAPED\" 2>&1 | tail -40
echo '── 远端改动 ──'
git status --short | head -20" 2>&1 | tail -60

echo
echo "（提示：若远端改了文件，记得让它在那边 commit，或你在本机 git pull）"
