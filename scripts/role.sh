#!/usr/bin/env bash
# 主备角色管理：同一时刻只让一台机器"值班"（跑飞书桥），另一台只同步状态。
#
#   ./scripts/role.sh status         # 看当前角色和判断依据
#   ./scripts/role.sh auto           # 自动判断（定时器调这个）
#   ./scripts/role.sh desktop        # 强制待命（桥停，只同步记忆）
#   ./scripts/role.sh laptop         # 强制值班（桥起）
#   ./scripts/role.sh clear          # 清除强制，回到自动
#
# 判断依据：能否 SSH 到主力机（desktop）。主力机开着就一定通，关了就超时。
# 待命 = 停飞书桥 + 保留记忆同步定时器；值班 = 起飞书桥。
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
STATE_DIR="${ROSIM_STATE_DIR:-$HOME/.config/rosim}"
if ! mkdir -p "$STATE_DIR" 2>/dev/null; then
  STATE_DIR="$ROOT/.scratch/rosim"; mkdir -p "$STATE_DIR"
  echo "（$HOME/.config 不可写，状态放 $STATE_DIR）"
fi
OVERRIDE="$STATE_DIR/role.override"      # 内容: desktop | laptop
FAILS="$STATE_DIR/role.fails"            # 连续失败次数
DESKTOP="${DESKTOP_HOST:-desktop}"
NEED_FAILS="${ROLE_FAILS:-2}"            # 连续几次连不上才接管（≈2 分钟）
BRIDGE_UNIT="dsh-bridge"

probe_desktop() { ssh -o ConnectTimeout=3 -o BatchMode=yes "$DESKTOP" 'true' 2>/dev/null; }

bridge_running() { systemctl --user is-active --quiet "$BRIDGE_UNIT" 2>/dev/null; }
have_bridge()    { systemctl --user list-unit-files "$BRIDGE_UNIT.service" 2>/dev/null | grep -q "$BRIDGE_UNIT"; }

set_role() {   # $1=active|standby
  if ! have_bridge; then echo "（本机没有 $BRIDGE_UNIT 单元，跳过桥的启停）"; return 0; fi
  if [ "$1" = "active" ]; then
    bridge_running || { systemctl --user start "$BRIDGE_UNIT" 2>/dev/null && echo "桥: 已启动（本机值班）"; }
  else
    bridge_running && { systemctl --user stop "$BRIDGE_UNIT" 2>/dev/null && echo "桥: 已停止（本机待命）"; }
  fi
}

case "${1:-status}" in
  desktop) echo desktop > "$OVERRIDE"; echo "已锁定：本机待命"; set_role standby ;;
  laptop)  echo laptop  > "$OVERRIDE"; echo "已锁定：本机值班"; set_role active ;;
  clear)   rm -f "$OVERRIDE"; echo "已恢复自动判断" ;;
  status)
    echo "主机: $(hostname)"
    echo "覆盖: $(cat "$OVERRIDE" 2>/dev/null || echo 无)"
    echo "桥:   $(bridge_running && echo running || echo stopped)"
    if probe_desktop; then echo "主力机: 在线"; else echo "主力机: 不可达"; fi
    ;;
  auto)
    if [ -f "$OVERRIDE" ]; then
      [ "$(cat "$OVERRIDE")" = laptop ] && set_role active || set_role standby
      echo "（按强制设置执行）"; exit 0
    fi
    if probe_desktop; then
      echo 0 > "$FAILS"; set_role standby
      echo "主力机在线 → 本机待命（只同步记忆）"
    else
      f=$(( $(cat "$FAILS" 2>/dev/null || echo 0) + 1 )); echo "$f" > "$FAILS"
      if [ "$f" -ge "$NEED_FAILS" ]; then
        set_role active; echo "主力机连续 $f 次不可达 → 本机接管（值班）"
      else
        echo "主力机不可达（$f/$NEED_FAILS），暂不接管"
      fi
    fi
    ;;
  *) echo "用法: role.sh {status|auto|desktop|laptop|clear}"; exit 1 ;;
esac
