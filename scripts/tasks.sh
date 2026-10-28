#!/usr/bin/env bash
# 跨机任务队列（白天笔记本排队，晚上主力机消费）
#
#   ./scripts/tasks.sh add "训练 blocked_ratio=0.6 / 3M 步"   # 入队
#   ./scripts/tasks.sh list                                  # 看队列
#   ./scripts/tasks.sh done 1                                # 标记第 1 条完成
#   ./scripts/tasks.sh next                                  # 取最早一条（输出后不删除）
#
# 队列存在 docs/tasks.md（进 git，两台机器都能看到）。
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
F="$ROOT/docs/tasks.md"
mkdir -p "$(dirname "$F")"
[ -f "$F" ] || printf '# 任务队列（跨机）\n\n## 待执行（主力机）\n\n## 已完成\n' > "$F"

cmd="${1:-list}"; shift 2>/dev/null || true
case "$cmd" in
  add)
    [ -z "${1:-}" ] && { echo "用法: tasks.sh add \"任务描述\""; exit 1; }
    ts=$(date '+%Y-%m-%d %H:%M')
    python3 - "$F" "$ts" "$*" <<'PY'
import sys, pathlib
f, ts, text = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
s = f.read_text()
head, sep, rest = s.partition("## 待执行（主力机）")
pending, sep2, done = rest.partition("## 已完成")
line = f"- [ ] {ts} {text}\n"
pending = pending.rstrip("\n") + "\n" + line + "\n" if pending.strip() else "\n" + line + "\n"
f.write_text(head + sep + pending + sep2 + done)
print("已入队:", text)
PY
    ;;
  list|next)
    echo "── 待执行 ──"
    awk '/## 待执行/{f=1;next} /## 已完成/{f=0} f && /- \[ \]/' "$F" \
      | sed 's/^- \[ \] //' | nl -w2 -s'. '
    if [ "$cmd" = "next" ]; then
      first=$(awk '/## 待执行/{f=1;next} /## 已完成/{f=0} f && /- \[ \]/' "$F" | head -1)
      [ -z "$first" ] && { echo "（队列为空）"; exit 0; }
      echo "→ 最早一条：$first"
    fi
    ;;
  done)
    idx="${1:-}"; [ -z "$idx" ] && { echo "用法: tasks.sh done <序号>"; exit 1; }
    python3 - "$F" "$idx" <<'PY'
import sys, pathlib, re
f, idx = pathlib.Path(sys.argv[1]), int(sys.argv[2])
s = f.read_text()
head, sep, rest = s.partition("## 待执行（主力机）")
pending, sep2, done = rest.partition("## 已完成")
pl = pending.splitlines()
c = 0
for i, ln in enumerate(pl):
    if ln.strip().startswith("- [ ]"):
        c += 1
        if c == idx:
            item = ln.replace("- [ ]", "- [x]")
            pl.pop(i)
            done = done.rstrip("\n") + "\n" + item + "\n"
            break
else:
    sys.exit("没有这个序号")
f.write_text(head + sep + "\n" + "\n".join(pl).strip("\n") + "\n\n" + sep2 + done)
print("已完成第", idx, "条")
PY
    ;;
  *) echo "用法: tasks.sh {add|list|next|done}"; exit 1;;
esac
