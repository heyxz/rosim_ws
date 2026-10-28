#!/usr/bin/env bash
# 诊断 VSCode 在这个仓库里崩溃（SIGSEGV in uv_loop_interrupt）的问题
#
#   bash scripts/vscode_doctor.sh
#
# 已知根因：仓库里 .pylibs/build/install/log 文件太多，VSCode 递归监听吃光
# inotify 配额 → fileWatcher 进程被杀 → 主进程 SIGSEGV。
set +u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$ROOT"

echo "== 1. inotify 配额 =="
w=$(cat /proc/sys/fs/inotify/max_user_watches)
i=$(cat /proc/sys/fs/inotify/max_user_instances)
echo "  max_user_watches   = $w     (建议 ≥ 524288)"
echo "  max_user_instances = $i     (建议 ≥ 1024)"
if [ "$w" -lt 524288 ]; then
  echo "  ⚠ 配额偏低，执行："
  echo "    sudo sysctl -w fs.inotify.max_user_watches=524288 fs.inotify.max_user_instances=1024"
  echo "    echo 'fs.inotify.max_user_watches=524288' | sudo tee /etc/sysctl.d/99-inotify.conf"
  echo "    echo 'fs.inotify.max_user_instances=1024' | sudo tee -a /etc/sysctl.d/99-inotify.conf"
fi

echo "== 2. 大目录（会被递归监听） =="
for d in .pylibs build install log .scratch dsh-bridge/node_modules .git; do
  [ -e "$d" ] && printf "  %-28s %6s 文件\n" "$d" "$(find "$d" -type f 2>/dev/null | wc -l)"
done

echo "== 3. 工作区排除配置 =="
if [ -f .vscode/settings.json ] && grep -q 'watcherExclude' .vscode/settings.json; then
  echo "  ✅ .vscode/settings.json 已排除 .pylibs/build/install/log"
else
  echo "  ❌ 缺少排除配置（应包含 files.watcherExclude）"
fi

echo "== 4. compile_commands.json（C++ 跳转/补全） =="
n=$(find build -maxdepth 2 -name compile_commands.json 2>/dev/null | wc -l)
if [ "$n" -gt 0 ]; then
  echo "  ✅ 找到 $n 个"
  find build -maxdepth 2 -name compile_commands.json | head -3 | sed 's/^/     /'
else
  echo "  ⚠ 没有。生成命令："
  echo "    source /opt/ros/humble/setup.bash && source ~/ws_moveit/install/setup.bash"
  echo "    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
fi

echo "== 4.5 GUI 服务能否唤起 VSCode（右上角按钮） =="
if grep -q "^DISPLAY=" ~/.config/dsh-web.env 2>/dev/null; then
  echo "  ✅ dsh-web.env 里配了 DISPLAY（$(grep '^DISPLAY=' ~/.config/dsh-web.env | head -1)）"
else
  echo "  ❌ dsh-web.env 没有 DISPLAY → 服务里 spawn 的 GUI 程序起不了窗口"
  echo "     修复：echo 'DISPLAY=:1' >> ~/.config/dsh-web.env"
  echo "           echo 'XAUTHORITY=/run/user/1000/gdm/Xauthority' >> ~/.config/dsh-web.env"
  echo "           systemctl --user restart dsh-web"
fi
pid=$(systemctl --user show -p MainPID --value dsh-web.service 2>/dev/null)
if [ -n "$pid" ] && [ -r "/proc/$pid/environ" ]; then
  if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q '^DISPLAY='; then
    echo "  ✅ 运行中的 dsh-web 进程已有 DISPLAY（按钮可用）"
  else
    echo "  ⚠ 运行中的 dsh-web 进程仍无 DISPLAY —— 需要重启服务才生效"
  fi
fi

echo "== 5. 最近的 VSCode 崩溃日志 =="
L=$(ls -dt ~/.config/Code/logs/*/ 2>/dev/null | head -1)
[ -n "$L" ] && { grep -aiE "fileWatcher|crash|SIGSEGV|ENOSPC" "$L/main.log" 2>/dev/null | tail -4 | sed 's/^/  /'; }
ls -t /var/crash/*code*.crash 2>/dev/null | head -2 | sed 's/^/  崩溃转储: /'

cat <<'TIP'

== 建议的打开方式 ==
  code --disable-gpu "$(pwd)"     # 先关 GPU 加速试一次
  仍崩的话，把 .vscode/settings.json 里的排除项确认生效后重开
TIP
