#!/usr/bin/env bash
# 两台机器的状态看板
#
#   ./scripts/hosts.sh
#
# 主机别名走 ~/.ssh/config；DESKTOP_HOST=local 表示"本机"（不走 ssh）。
# 主力机没开机时显示"离线"，不算错误。
set +u
DESKTOP="${DESKTOP_HOST:-desktop}"
LAPTOP="${LAPTOP_HOST:-laptop}"

# 在目标机（或本机）上执行的采集脚本
read -r -d '' STATUS <<'EOS'
printf "  主机: %s  运行: %s\n" "$(hostname)" "$(uptime -p 2>/dev/null)"
if cd ~/rosim_ws 2>/dev/null; then
  printf "  代码: %s  未提交 %s\n" "$(git rev-parse --short HEAD 2>/dev/null)" "$(git status --porcelain 2>/dev/null | wc -l)"
fi
b=$(systemctl --user is-active dsh-bridge 2>/dev/null); printf "  飞书桥: %s\n" "${b:-未知}"
r=$(systemctl --user is-active rosim-server 2>/dev/null); printf "  rosim服务: %s\n" "${r:-未知}"
c=$(curl -s -m 2 -o /dev/null -w "%{http_code}" http://127.0.0.1:8765/ 2>/dev/null); printf "  8765: %s\n" "${c:-000}"
printf "  磁盘: %s 可用\n" "$(df -h ~ 2>/dev/null | tail -1 | awk '{print $4}')"
if command -v nvidia-smi >/dev/null 2>&1; then
  printf "  GPU: %s\n" "$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader 2>/dev/null | head -1)"
fi
t=$(ls -t ~/rosim_ws/rl_a/*.log 2>/dev/null | head -1)
[ -n "$t" ] && printf "  最近日志: %s\n" "$(basename "$t")"
EOS

probe() {
  local host="$1" label="$2"
  echo "──────── $label ($host) ────────"
  case "$host" in
    local|localhost|127.0.0.1)
      bash -c "$STATUS" 2>/dev/null
      echo; return ;;
  esac
  if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$host" true 2>/dev/null; then
    echo "  状态: 离线 / 不可达"; echo; return
  fi
  ssh -o ConnectTimeout=8 -o BatchMode=yes "$host" "$STATUS" 2>/dev/null
  echo
}

probe "$DESKTOP" "主力机"
probe "$LAPTOP"  "笔记本"
