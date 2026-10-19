#!/usr/bin/env bash
# 一键启动：仿真服务端 + 浏览器调试页
set -e
systemctl --user restart rosim-server.service
printf "等待服务端就绪"
for _ in $(seq 1 60); do
  if curl -sf -m 1 http://127.0.0.1:8765/ >/dev/null 2>&1; then echo " ✅"; break; fi
  printf "."; sleep 0.5
done
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
( google-chrome --app="http://127.0.0.1:8765/?v=$(date +%s)" >/dev/null 2>&1 & )
echo "调试页: http://127.0.0.1:8765/"
