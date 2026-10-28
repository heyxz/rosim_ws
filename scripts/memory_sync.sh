#!/usr/bin/env bash
# 把跨机共享记忆（docs/memory.md）同步到远端仓库。
#
#   ./scripts/memory_sync.sh          # 提交并推送（无变化则什么都不做）
#
# 设计：只动 docs/memory.md，不碰工作区其他改动（避免把半成品代码推上去）。
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
F="docs/memory.md"
[ -f "$F" ] || { echo "缺少 $F"; exit 1; }

git add "$F" 2>/dev/null
if git diff --cached --quiet -- "$F"; then
  echo "记忆无变化，跳过"
  exit 0
fi
git commit -q -m "memory: $(date '+%F %H:%M')" -- "$F" || { echo "提交失败"; exit 1; }
if git pull --rebase --autostash -q 2>/dev/null; then
  git push -q 2>/dev/null && echo "✅ 记忆已推送" || echo "⚠ 推送失败（远端不可达？稍后重试）"
else
  git rebase --abort 2>/dev/null
  echo "⚠ 远端有冲突，已中止 rebase；请手动处理 docs/memory.md"
fi
