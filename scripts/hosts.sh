#!/usr/bin/env bash
# 两台机器的状态看板（主力机上用：能看到笔记本；笔记本上也能看到主力机）
#
#   ./scripts/hosts.sh
#
# 主机别名走 ~/.ssh/config；主力机没开时会显示"离线"，不算错误。
set +u
DESKTOP="${DESKTOP_HOST:-desktop}"
LAPTOP="${LAPTOP_HOST:-laptop}"

probe() {
  local host="$1" label="$2"
  echo "──────── $label ($host) ────────"
  if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" 'true' 2>/dev/null; then
    echo "  状态: 离线 / 不可达"; echo; return
  fi
  ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" '
    printf "  主机: %s  运行: %s\n" "$(hostname)" "$(uptime -p 2>/dev/null)"
    cd ~/rosim_ws 2>/dev/null && {
      printf "  代码: %s  未提交 %s\n" "$(git rev-parse --short HEAD 2>/dev/null)" "$(git status --porcelain 2>/dev/null | wc -l)"
    }
    s=$(systemctl --user is-active dsh-bridge 2>/dev/null); printf "  飞书桥: %s\n" "${s:-未知}"
    r=$(systemctl --user is-active rosim-server 2>/dev/null); printf "  rosim服务: %s\n" "${r:-未知}"
    printf "  8765: %s\n" "$(curl -s -m 2 -o /dev/null -w "%{http_code}" http://127.0.0.1:8765/ 2>/dev/null || echo "-")"
    printf "  磁盘: %s\n" "$(df -h ~ | tail -1 | awk "{print \$4\" 可用\"}")"
    command -v nvidia-smi >/dev/null && printf "  GPU: %s\n" "$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader 2>/dev/null | head -1)"
    t=$(ls -t ~/rosim_ws/rl_a/*.log 2>/dev/null | head -1); [ -n "$t" ] && printf "  最近日志: %s\n" "$(basename "$t")"
  ' 2>/dev/null
  echo
}

probe "$DESKTOP" "主力机"
probe "$LAPTOP"  "笔记本"
