#!/bin/bash
#
# RM 双项目看门狗
#
# 开机默认：只启动自瞄，不启动图传
# 收到 mode=99 → 杀自瞄，启动图传
# 收到 mode=0~9 → 杀图传，启动自瞄
# 当前项目崩溃 → 自动重启同一项目
#
# ~/.config/autostart/ 里配 .desktop 指向这个脚本即可

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# ============================================================================
# 工作区路径 — 按你的环境改
# ============================================================================
AIM_WS="/home/zfm/zfm_ws_final/zfm_autoaim"
AIM_LAUNCH="ros2 launch rm_bringup bringup.launch.py"
AIM_KILL="rm_bringup|component_container|robot_state_publisher|armor_detector|armor_solver|serial_driver|camera_driver"

DEPLOY_WS="/home/zfm/zfm_ws_final/zfm_client"
DEPLOY_LAUNCH="ros2 launch bringup onboard_sniper.launch.py"
DEPLOY_KILL="hik_camera|video_encoder|serial_bridge"

MODE_FILE="/tmp/mcu_mode"
LOG_DIR="/home/zfm/zfm2026-log/watchdog"
PID_FILE="/tmp/rm_watchdog.pid"

POLL_INTERVAL=1     # mode 文件轮询间隔（秒）
CHECK_INTERVAL=3    # 进程存活检查间隔（秒）
COOLDOWN=5          # 崩溃重启冷却（秒）

mkdir -p "$LOG_DIR"

# 防止重复启动
if [[ -f "$PID_FILE" ]]; then
    old_pid=$(cat "$PID_FILE")
    if kill -0 "$old_pid" 2>/dev/null; then
        echo "[watchdog] Already running (PID $old_pid)"
        exit 0
    fi
fi
echo $$ > "$PID_FILE"

# ============================================================================

log() {
    set +e
    echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_DIR/watchdog.log"
    set -e
}

get_mode() {
    if [[ -f "$MODE_FILE" ]]; then
        cat "$MODE_FILE" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

# ============================================================================
# 进程管理
# ============================================================================

kill_all() {
    local patterns="$1"
    local label="$2"

    # 先优雅地杀 ros2 launch 进程组
    local pids
    pids=$(ps aux | grep "ros2 launch" | grep -v grep | grep -v watchdog | awk '{print $2}') || true
    for pid in $pids; do
        kill -TERM -"$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in $pids; do
        kill -KILL -"$pid" 2>/dev/null || true
    done

    # 再清理残余节点
    ps aux | grep -E "$patterns" | grep -v grep | grep -v watchdog \
        | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true

    sleep 2
    log "[kill] $label stopped"
}

start_project() {
    local ws="$1"
    local cmd="$2"
    local label="$3"
    local logfile="$LOG_DIR/${label}.log"

    log "[start] $label"
    if [[ ! -d "$ws" ]]; then
        log "[start] ERROR: workspace not found: $ws"
        return 1
    fi
    cd "$ws" || { log "[start] ERROR: cannot cd to $ws"; return 1; }
    set +u
    source /opt/ros/humble/setup.bash
    source install/setup.bash 2>/dev/null || true
    set -u

    setsid bash -c "$cmd" >> "$logfile" 2>&1 &
    local pid=$!
    log "[start] $label PID=$pid  log=$logfile"
    echo "$pid"
}

is_alive() {
    local patterns="$1"

    # 检查 ros2 launch 进程
    ps aux | grep "ros2 launch" | grep -v grep | grep -v watchdog > /dev/null 2>&1 && return 0

    # 检查关键节点
    ps aux | grep -E "$patterns" | grep -v grep | grep -v watchdog > /dev/null 2>&1 && return 0

    return 1
}

# ============================================================================
# 等待 mcu_bridge 就绪
# ============================================================================

wait_for_bridge() {
    log "[init] Waiting for mcu_bridge (/tmp/ttyACM_mcu)..."

    # 桥接没跑就先自动拉起
    if ! pgrep -f "mcu_bridge.py" > /dev/null 2>&1; then
        log "[init] mcu_bridge not running, starting..."
        bash "${SCRIPT_DIR}/mcu_bridge.sh"
    fi

    for i in $(seq 1 60); do
        if [[ -e /tmp/ttyACM_mcu ]]; then
            log "[init] mcu_bridge ready (waited ${i}s)"
            return 0
        fi
        sleep 1
    done
    log "[init] WARNING: /tmp/ttyACM_mcu not ready after 60s, starting anyway"
}

# ============================================================================
# 主逻辑
# ============================================================================

main() {
    log "===== RM Watchdog start (PID $$) ====="
    log "  autoaim: $AIM_WS"
    log "  deploy:  $DEPLOY_WS"
    log "  default: autoaim (mode=0)"

    # 等桥接就绪再继续
    wait_for_bridge

    # 确保有初始 mode
    echo "0" > "$MODE_FILE" 2>/dev/null || true

    local current="none"       # aim / deploy / none
    local launch_pid=""
    local last_restart=0
    local last_check=0

    # 开机默认启动自瞄
    start_project "$AIM_WS" "$AIM_LAUNCH" "autoaim" || true
    current="aim"
    last_restart=$(date +%s)

    while true; do
        sleep "$POLL_INTERVAL"

        # ---- 确保 mcu_bridge 一直活着 ----
        if ! pgrep -f "mcu_bridge.py" > /dev/null 2>&1 || [[ ! -e /tmp/ttyACM_mcu ]]; then
            log "[watch] mcu_bridge missing, restarting..."
            bash "${SCRIPT_DIR}/mcu_bridge.sh"
            for i in $(seq 1 30); do
                if [[ -e /tmp/ttyACM_mcu ]]; then
                    log "[watch] mcu_bridge restarted (waited ${i}s)"
                    break
                fi
                sleep 1
            done
        fi

        # ---- 读 MCU 指令 ----
        local mode
        mode=$(get_mode)

        # 映射：mode_flag=1 自定义客户端，mode_flag=0 自瞄
        local want="aim"
        [[ "$mode" == "1" ]] && want="deploy"

        # ---- 模式切换 ----
        if [[ "$want" != "$current" ]]; then
            log "========================================="
            log "SWITCH: $current -> $want (mode=$mode)"

            if [[ "$current" == "aim" ]]; then
                kill_all "$AIM_KILL" "autoaim"
            elif [[ "$current" == "deploy" ]]; then
                kill_all "$DEPLOY_KILL" "deploy"
            fi

            sleep 1

            if [[ "$want" == "aim" ]]; then
                start_project "$AIM_WS" "$AIM_LAUNCH" "autoaim" || true
                current="aim"
            else
                start_project "$DEPLOY_WS" "$DEPLOY_LAUNCH" "deploy" || true
                current="deploy"
            fi

            last_restart=$(date +%s)
            last_check=$(date +%s)
            continue
        fi

        # ---- 崩溃检测（每 CHECK_INTERVAL 秒一次） ----
        local now
        now=$(date +%s)
        if [[ $((now - last_check)) -ge $CHECK_INTERVAL ]]; then
            last_check=$now

            local patterns
            [[ "$current" == "aim" ]] && patterns="$AIM_KILL" || patterns="$DEPLOY_KILL"

            if ! is_alive "$patterns"; then
                local elapsed=$((now - last_restart))

                if [[ $elapsed -lt $COOLDOWN ]]; then
                    log "[watch] $current dead, cooldown (${elapsed}s < ${COOLDOWN}s), wait..."
                    sleep $((COOLDOWN - elapsed))
                fi

                log "========================================="
                log "CRASH: $current died, restarting..."
                log "========================================="

                kill_all "$patterns" "$current (crashed)"
                sleep 1

                if [[ "$current" == "aim" ]]; then
                    start_project "$AIM_WS" "$AIM_LAUNCH" "autoaim" || true
                else
                    start_project "$DEPLOY_WS" "$DEPLOY_LAUNCH" "deploy" || true
                fi

                last_restart=$(date +%s)
            fi
        fi
    done
}

# ============================================================================

cleanup() {
    log "Signal received, cleaning up..."
    kill_all "$AIM_KILL" "autoaim" 2>/dev/null || true
    kill_all "$DEPLOY_KILL" "deploy" 2>/dev/null || true
    rm -f "$PID_FILE"
    log "Watchdog stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

main
