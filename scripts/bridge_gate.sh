#!/usr/bin/env bash
# 飞书桥的"启动闸门"：确认本机该值班后才真正启动桥。
#
# 为什么需要：桥如果直接开机自启，主力机也在线时会出现**两台同时回飞书**。
# 单元保持 enabled（开机自启），但 ExecStart 先过这道闸门：
#   - 主力机可达 → 本机待命，闸门退出（systemd 记为 inactive，正确）
#   - 主力机不可达 → 立刻 exec 真正的桥
#   - 开机初期网络没就绪 → 每 10 秒重试，最多 10 分钟
#
# role.sh 的定时器负责后续的启动/停止，本脚本只处理"启动时机"。
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROLE="$ROOT/scripts/role.sh"
BRIDGE_RUN="$ROOT/dsh-bridge/run.sh"
TRIES=${BRIDGE_GATE_TRIES:-60}     # 60 × 10s = 10 分钟

[ -x "$BRIDGE_RUN" ] || { echo "缺少 $BRIDGE_RUN"; exit 1; }

i=0
while [ "$i" -lt "$TRIES" ]; do
  i=$((i+1))
  if "$ROLE" is-active 2>/dev/null; then
    echo "闸门：本机应值班（第 $i 次检查）→ 启动飞书桥"
    exec "$BRIDGE_RUN"
  fi
  echo "闸门：主力机在线，本机待命（第 $i/$TRIES 次检查）"
  sleep 10
done

echo "闸门：$TRIES 次检查后仍待命，退出（由 rosim-role.timer 后续接管）"
exit 0
