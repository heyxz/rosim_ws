#!/usr/bin/env bash
# 开发环境体检（只读，不安装任何东西）
#
#   bash scripts/check_env.sh
#
# 输出每一项的：期望版本 / 实际版本 / 状态（OK / 偏高 / 偏低 / 缺失 / 冲突）
# 最后给出"必须先处理"的清单。退出码 0 = 没有致命问题。
#
# 设计前提：目标机器可能已经装了一部分、而且某些组件版本更高。所以这里只
# *报告* 差异，不擅自改系统。

set +u   # ROS 的 setup.bash 与 set -u 冲突

BASELINE_OS="22.04"
BASELINE_PY="3.10"
BASELINE_ROS="humble"

fatal=()
warn=()

row() { printf "  %-34s %-16s %-16s %s\n" "$1" "$2" "$3" "$4"; }

ver_ge() {   # ver_ge A B -> A>=B
  [ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" = "$2" ]
}

# got 的上游版本等于 want（忽略 +cu124、.3ubuntu7 这类后缀）
same_upstream() { case "$1" in "$2"*) return 0;; *) return 1;; esac; }

echo "=============================================================="
echo " rosim 开发环境体检   $(date '+%F %T')"
echo "=============================================================="

# ---------------------------------------------------------------- 1. 系统
echo
echo "[1] 操作系统"
os_id=""; os_ver=""
if [ -r /etc/os-release ]; then
  . /etc/os-release
  os_id="$ID"; os_ver="$VERSION_ID"
fi
if [ "$os_id" = "ubuntu" ] && [ "$os_ver" = "$BASELINE_OS" ]; then
  row "Ubuntu" "$BASELINE_OS" "$os_ver" "OK"
elif [ "$os_id" = "ubuntu" ] && ver_ge "$os_ver" "$BASELINE_OS"; then
  row "Ubuntu" "$BASELINE_OS" "$os_ver" "偏高（危险）"
  fatal+=("Ubuntu $os_ver 不是 22.04：ROS 2 Humble 的二进制只支持 22.04。"
          "→ 方案 A：换 22.04 机器；方案 B：用 docker/ 里的镜像（docs/install.md 第 12 节）")
else
  row "Ubuntu" "$BASELINE_OS" "${os_id:-未知} ${os_ver:-?}" "不匹配"
  fatal+=("系统不是 Ubuntu 22.04，ROS 2 Humble 装不了。")
fi
row "内核" "-" "$(uname -r)" "信息"

# ---------------------------------------------------------------- 2. 工具链
echo
echo "[2] 编译工具链"
check_ver() {   # check_ver 名称 期望 实际命令
  local name="$1" want="$2" got="$3"
  if [ -z "$got" ]; then
    row "$name" "$want" "缺失" "缺失"
    fatal+=("缺少 $name")
    return 1
  fi
  if same_upstream "$got" "$want"; then row "$name" "$want" "$got" "OK"
  else
    local same; same=$(printf '%s\n%s\n' "$want" "$got" | sort -V | head -1)
    if [ "$same" = "$want" ]; then row "$name" "$want" "$got" "偏高"
    else row "$name" "$want" "$got" "偏低"; warn+=("$name 低于基线（$got < $want），可能可以，但未验证"); fi
  fi
  return 0
}

check_ver "gcc" "11.4.0" "$(gcc -dumpfullversion 2>/dev/null)"
check_ver "cmake" "3.22.1" "$(cmake --version 2>/dev/null | head -1 | awk '{print $3}')"
check_ver "python3" "$BASELINE_PY" "$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])' 2>/dev/null)"
check_ver "node" "22.23.3" "$(node --version 2>/dev/null | tr -d v)"

# ---------------------------------------------------------------- 3. ROS
echo
echo "[3] ROS 2"
if [ -r /opt/ros/$BASELINE_ROS/setup.bash ]; then
  row "ros-$BASELINE_ROS" "$BASELINE_ROS" "已安装" "OK"
else
  row "ros-$BASELINE_ROS" "$BASELINE_ROS" "缺失" "缺失"
  fatal+=("没装 ROS 2 Humble（/opt/ros/humble 不存在）")
fi
for other in /opt/ros/*/setup.bash; do
  [ -e "$other" ] || continue
  d=$(basename "$(dirname "$other")")
  [ "$d" = "$BASELINE_ROS" ] && continue
  row "检出其他 ROS 版本" "仅 $BASELINE_ROS" "$d" "冲突"
  warn+=("机器上有 ROS $d：本项目只支持 humble，混用会让 source 顺序出错")
done

ros_pkgs=(
  "ros-humble-rclcpp|16.0.16"
  "ros-humble-ros2-control|2.52.2"
  "ros-humble-controller-manager|2.52.2"
  "ros-humble-joint-trajectory-controller|2.50.2"
  "ros-humble-joint-state-broadcaster|2.50.2"
  "ros-humble-robot-state-publisher|3.0.3"
  "ros-humble-xacro|2.1.1"
  "ros-humble-geometric-shapes|2.3.2"
  "ros-humble-tf2-eigen|0.25.17"
  "ros-humble-ompl|1.7.0"
)
missing_ros=0
for entry in "${ros_pkgs[@]}"; do
  pkg="${entry%%|*}"; want="${entry##*|}"
  got=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null | cut -d- -f1)
  if [ -z "$got" ]; then
    row "$pkg" "$want" "缺失" "缺失"; missing_ros=$((missing_ros+1))
  elif [ "$got" = "$want" ]; then
    row "$pkg" "$want" "$got" "OK"
  else
    same=$(printf '%s\n%s\n' "$want" "$got" | sort -V | head -1)
    [ "$same" = "$want" ] && row "$pkg" "$want" "$got" "偏高" || row "$pkg" "$want" "$got" "偏高/偏低"
  fi
done
[ "$missing_ros" -gt 0 ] && fatal+=("缺 $missing_ros 个 ROS 包（apt install 补齐，见 docs/install.md 第 2 节）")

# ---------------------------------------------------------------- 4. MoveIt
echo
echo "[4] MoveIt 2（本项目要求源码 2.5.x）"
MOVEIT_WS="${MOVEIT_WS:-$HOME/ws_moveit}"
if [ -r "$MOVEIT_WS/install/setup.bash" ]; then
  row "MoveIt 源码工作区" "$MOVEIT_WS" "存在" "OK"
else
  row "MoveIt 源码工作区" "$MOVEIT_WS" "缺失" "缺失"
  fatal+=("没有 $MOVEIT_WS（MoveIt 2.5.9 需要源码编译，apt 里没有 ros-humble-moveit-core）")
fi
if dpkg -s ros-humble-moveit-core >/dev/null 2>&1; then
  row "apt 版 moveit-core" "不用" "$(dpkg-query -W -f='${Version}' ros-humble-moveit-core)" "冲突"
  warn+=("机器上有 apt 版 moveit-core：和源码版同时 source 会打架，建议只用一种（docs/install.md 第 3 节）")
fi
if [ -r "$MOVEIT_WS/src/moveit2/moveit_core/package.xml" ]; then
  mv=$(grep -m1 "<version>" "$MOVEIT_WS/src/moveit2/moveit_core/package.xml" | sed 's/[^0-9.]//g')
  case "$mv" in
    2.5.*) row "MoveIt 版本" "2.5.9" "$mv" "OK" ;;
    2.[0-4].*) row "MoveIt 版本" "2.5.9" "$mv" "偏低"; warn+=("MoveIt $mv 偏旧，本项目在 2.5.9 上验证") ;;
    *) row "MoveIt 版本" "2.5.9" "${mv:-未知}" "偏高（危险）"
       fatal+=("MoveIt $mv：2.6+ 的 API 有变化，本项目未验证。建议 checkout 到 humble 分支的 2.5.9") ;;
  esac
fi

# ---------------------------------------------------------------- 5. 系统库
echo
echo "[5] 系统库"
check_lib() {
  local pkg="$1" want="$2"
  local got; got=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null | sed 's/+.*//;s/-.*//')
  if [ -z "$got" ]; then row "$pkg" "$want" "缺失" "缺失"; fatal+=("缺 $pkg"); return; fi
  if same_upstream "$got" "$want"; then row "$pkg" "$want" "$got" "OK"
  else
    local same; same=$(printf '%s\n%s\n' "$want" "$got" | sort -V | head -1)
    if [ "$same" = "$want" ]; then row "$pkg" "$want" "$got" "偏高"
    else row "$pkg" "$want" "$got" "偏低"; fi
  fi
}
check_lib "libeigen3-dev" "3.4.0"
check_lib "libboost-dev" "1.74.0"
check_lib "libtinyxml2-dev" "9.0.0"
check_lib "libconsole-bridge-dev" "1.0.1"
check_lib "libocct-foundation-dev" "7.5.1"
check_lib "libvtk9-dev" "9.1.0"
check_lib "qtbase5-dev" "5.15.3"

# ---------------------------------------------------------------- 6. Python 依赖
echo
echo "[6] Python 依赖（训练/策略，装在仓库内 .pylibs，可与系统隔离）"
PYLIBS="${RL_PYLIBS:-$PWD/.pylibs}"
if [ -d "$PYLIBS" ]; then
  row ".pylibs" "存在" "$PYLIBS" "OK"
  # 用 import 探真实版本：.pylibs 里可能残留多个 dist-info，元数据会报最新的，
  # 而实际导入的是另一份（开发机就踩过：元数据说 sb3 2.9.0，实际 2.3.2）。
  py_deps=(
    "torch|2.6.0"
    "stable_baselines3|2.3.2"
    "gymnasium|0.29.1"
    "numpy|1.26.4"
    "onnx|1.16.2"
    "onnxruntime|1.23.2"
    "pandas|2.3.3"
  )
  for entry in "${py_deps[@]}"; do
    mod="${entry%%|*}"; want="${entry##*|}"
    got=$(MPLCONFIGDIR=/tmp/mpl PYTHONPATH="$PYLIBS" python3 -c "
import importlib
try:
    m = importlib.import_module('$mod')
    print(getattr(m, 'AVERSION', getattr(m, '__version__', '')))
except Exception:
    print('')
" 2>/dev/null | tail -1)
    if [ -z "$got" ]; then row "$mod" "$want" "导入失败" "缺失"; warn+=(".pylibs 里 $mod 导入失败")
    elif same_upstream "$got" "$want"; then row "$mod" "$want" "$got" "OK"
    else
      local same; same=$(printf '%s\n%s\n' "$want" "$got" | sort -V | head -1)
      [ "$same" = "$want" ] && row "$mod" "$want" "$got" "偏高" \
                            || row "$mod" "$want" "$got" "偏低"
    fi
  done
  # 重复安装检测
  dups=$(ls -d "$PYLIBS"/*.dist-info 2>/dev/null | sed 's|.*/||;s|-[0-9].*||' | sort | uniq -d)
  if [ -n "$dups" ]; then
    row "重复安装的包" "无" "$(echo "$dups" | tr '\n' ' ')" "需清理"
    warn+=(".pylibs 里有多份同名包（$(echo "$dups" | tr '\n' ' ')）：导入结果不确定，建议按 docs/install.md 第 5.3 节重装")
  fi
else
  row ".pylibs" "存在" "缺失" "缺失（可跳过）"
  warn+=("没有 .pylibs：只影响 RL（训练/策略节点），纯 OMPL/直线规划不受影响")
fi

# ---------------------------------------------------------------- 7. 本仓库
echo
echo "[7] 本仓库"
if [ -f src/rosim_planning/package.xml ]; then
  row "仓库根" "-" "$PWD" "OK"
else
  row "仓库根" "-" "当前目录不像仓库根" "错误"
  fatal+=("请在仓库根目录运行：bash scripts/check_env.sh")
fi
[ -d install/rosim_planning ] && row "已编译(install/)" "存在" "存在" "OK" \
  || { row "已编译(install/)" "存在" "缺失" "待编译"; warn+=("还没 colcon build，见 docs/install.md 第 4 节"); }

# ---------------------------------------------------------------- 结论
echo
echo "=============================================================="
if [ ${#fatal[@]} -eq 0 ]; then
  echo " 结论：没有致命问题。"
else
  echo " 结论：有 ${#fatal[@]} 项必须先处理："
  i=1; for m in "${fatal[@]}"; do echo "   $i) $m"; i=$((i+1)); done
fi
if [ ${#warn[@]} -gt 0 ]; then
  echo
  echo " 提醒（${#warn[@]} 项，不阻塞）："
  i=1; for m in "${warn[@]}"; do echo "   $i) $m"; i=$((i+1)); done
fi
echo "=============================================================="
[ ${#fatal[@]} -eq 0 ] && exit 0 || exit 1
