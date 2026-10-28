#!/usr/bin/env bash
# 截当前屏幕 → 存进仓库内 .scratch/shots/ → 打印绝对路径。
#
# 用法：
#   ./scripts/shot.sh              # 全屏
#   ./scripts/shot.sh -w           # 当前窗口
#   ./scripts/shot.sh -d :1        # 指定 DISPLAY
#
# 拿到路径后，在飞书会话里让桥发出去：
#   - 我的回复里写 [[send:/abs/path.png]]   → 桥自动发图
#   - 或你在飞书直接发 /send /abs/path.png
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$ROOT/.scratch/shots"; mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/shot-$(date +%Y%m%d-%H%M%S).png"

MODE=""; DISP="${DISPLAY:-}"
while [ $# -gt 0 ]; do
  case "$1" in
    -w|--window) MODE="-u" ;;              # scrot -u = 当前窗口
    -d|--display) DISP="$2"; shift ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
  shift
done

command -v scrot >/dev/null || { echo "缺少 scrot：sudo apt install -y scrot"; exit 1; }

# 依次尝试可用 DISPLAY（无显示时给出明确原因）
try() {
  local d="$1"
  DISPLAY="$d" scrot $MODE -o "$OUT" >/dev/null 2>&1 && return 0
  return 1
}
if [ -n "$DISP" ]; then
  try "$DISP" || { echo "在 DISPLAY=$DISP 下截图失败"; exit 1; }
else
  for d in :0 :1 :2; do try "$d" && { echo "$OUT"; exit 0; }; done
  ls /tmp/.X11-unix/ >/dev/null 2>&1 || echo "（沙箱里看不到 /tmp/.X11-unix，无法访问 X）"
  echo "没有可用 DISPLAY（服务/沙箱环境可能访问不到 X）"; exit 1
fi
echo "$OUT"
