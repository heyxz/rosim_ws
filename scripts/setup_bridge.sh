#!/usr/bin/env bash
# 把飞书桥装到本机（解包 → 依赖 → 补丁 → systemd 单元），默认不启动。
#
#   bash scripts/setup_bridge.sh                      # 解包 ~/dsh-bridge.tgz 并装好
#   bash scripts/setup_bridge.sh --tarball /path.tgz  # 指定包
#   bash scripts/setup_bridge.sh --start              # 装好并立刻启动（谨慎：先确认主力机已停桥）
#   bash scripts/setup_bridge.sh --dry-run            # 只打印要做什么
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"
BRIDGE="$ROOT/dsh-bridge"
TARBALL="$HOME/dsh-bridge.tgz"
START=0; DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --tarball) TARBALL="$2"; shift 2 ;;
    --start)   START=1; shift ;;
    --dry-run) DRY=1; shift ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done
run() { [ "$DRY" = 1 ] && { echo "  [dry] $*"; return 0; }; "$@"; }
say() { printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

say "1/5 解包"
if [ -d "$BRIDGE" ]; then
  echo "已存在 $BRIDGE，跳过解包"
elif [ -f "$TARBALL" ]; then
  run tar xzf "$TARBALL" -C "$ROOT"
  echo "已解包 $TARBALL"
else
  echo "❌ 找不到 bridge 目录，也没有 $TARBALL"
  echo "   从主力机传：scp 主力机:~/rosim_ws/.scratch/dsh-bridge.tgz ~/"
  exit 1
fi

say "2/5 检查 .env"
if [ -f "$BRIDGE/.env" ]; then
  run chmod 600 "$BRIDGE/.env"
  echo "✅ .env 存在，权限已收紧"
else
  echo "❌ 缺 $BRIDGE/.env（飞书 App ID/Secret）"
  echo "   从主力机传：scp 主力机:~/rosim_ws/dsh-bridge/.env $BRIDGE/.env"
  exit 1
fi

say "3/5 安装依赖"
if [ -d "$BRIDGE/node_modules" ] && [ -n "$(ls -A "$BRIDGE/node_modules" 2>/dev/null)" ]; then
  echo "node_modules 已存在，跳过（要重装就删掉它）"
else
  ( cd "$BRIDGE" && run npm install ) || { echo "❌ npm install 失败"; exit 1; }
fi

say "4/5 打 dsh 补丁（session/resume）"
if [ -f "$BRIDGE/tools/patch-sdk-resume.mjs" ]; then
  if [ "$DRY" = 1 ]; then echo "  [dry] node $BRIDGE/tools/patch-sdk-resume.mjs"; else
    node "$BRIDGE/tools/patch-sdk-resume.mjs" --check >/dev/null 2>&1 \
      && echo "已打过补丁" \
      || node "$BRIDGE/tools/patch-sdk-resume.mjs"
  fi
else
  echo "⚠ 没有 tools/patch-sdk-resume.mjs，跳过（bridge 会在恢复失败时改用新会话）"
fi

say "5/5 安装 systemd 单元"
UD="$HOME/.config/systemd/user"; mkdir -p "$UD"
run cp "$BRIDGE/dsh-bridge.service" "$UD/dsh-bridge.service"
run sed -i "s#/home/xz/rosim_ws#${ROOT}#g" "$UD/dsh-bridge.service"
# PATH 行原本写死 /home/xz/.local/node/bin，换机后不存在 → 找不到 dsh；
# 改成"本机 node 所在目录"，systemd 子进程才找得到 dsh。
NODE_BIN="$(dirname "$(command -v node 2>/dev/null || echo /usr/bin/node)")"
# ExecStart 走启动闸门：开机自启但主力机在线时不连飞书（避免两台同时回消息）
GATE="$ROOT/scripts/bridge_gate.sh"
run sed -i "s#^ExecStart=.*#ExecStart=${GATE}#" "$UD/dsh-bridge.service"
run sed -i "s#^Environment=PATH=.*#Environment=PATH=${NODE_BIN}:/usr/local/bin:/usr/bin:/bin#" "$UD/dsh-bridge.service"
run systemctl --user daemon-reload
echo "已写入 $UD/dsh-bridge.service"
grep -E "^(WorkingDirectory|ExecStart|Environment=DSH_HOME)" "$UD/dsh-bridge.service" | sed 's/^/  /'

say "完成"
if [ "$START" = 1 ]; then
  run systemctl --user enable --now dsh-bridge
  echo "已启动。查看状态：systemctl --user status dsh-bridge --no-pager"
else
  echo "还没启动。确认主力机已停桥后再执行："
  echo "  systemctl --user enable --now dsh-bridge"
  echo "  飞书发「在么」→ 应该只收到一条回复"
fi
