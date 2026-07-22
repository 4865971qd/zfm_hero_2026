#!/bin/bash
#
# MCU 串口桥接 — 开机自启，独占 /dev/ttyACM1。
#
# 做什么：
#   1. 打开 /dev/ttyACM1 @115200
#   2. 创建虚拟串口 /tmp/ttyACM_mcu（自瞄项目通过它和 MCU 通信）
#   3. 双向透明转发所有数据
#   4. 解析 MCU 发来的 16 字节包，提取 mode 字节写入 /tmp/mcu_mode
#
# 运行方式：
#   ./mcu_bridge.sh          手动启动
#   ~/.config/autostart/     开机自启（配 .desktop 指向这里）

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BRIDGE_PY="$SCRIPT_DIR/mcu_bridge.py"
PID_FILE="/tmp/mcu_bridge.pid"
LOG_FILE="/home/zfm/zfm2026-log/mcu_bridge.log"

mkdir -p "$(dirname "$LOG_FILE")"

# 防止重复启动
if [[ -f "$PID_FILE" ]]; then
    old_pid=$(cat "$PID_FILE")
    if kill -0 "$old_pid" 2>/dev/null; then
        echo "[mcu_bridge] Already running (PID $old_pid)"
        exit 0
    fi
fi

set +e
echo "[mcu_bridge] Starting at $(date)" | tee -a "$LOG_FILE"
set -e
python3 "$BRIDGE_PY" >> "$LOG_FILE" 2>&1 &
echo $! > "$PID_FILE"
echo "[mcu_bridge] Started (PID $!)"

# 等虚拟串口就绪
for i in $(seq 1 20); do
    if [[ -e /tmp/ttyACM_mcu ]]; then
        echo "[mcu_bridge] Virtual serial /tmp/ttyACM_mcu ready"
        exit 0
    fi
    sleep 0.5
done
echo "[mcu_bridge] WARNING: /tmp/ttyACM_mcu not created after 10s, continuing anyway"
