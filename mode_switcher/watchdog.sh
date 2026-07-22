#!/bin/bash
#
# RM 看门狗 — 只做一件事：读 mode 文件，切换项目
#
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# 自瞄项目
AIM1_WS="/home/zfm/zfm_ws_final/zfm_autoaim"
AIM1_LAUNCH="ros2 launch rm_bringup bringup.launch.py"
AIM1_KILL="rm_bringup|component_container|robot_state_publisher|armor_detector|armor_solver|serial_driver|camera_driver"

AIM2_WS="/home/zfm/zfm_ws_final/autoaim"
AIM2_LAUNCH="./build/autoaim configs/standard.yaml"
AIM2_KILL="[b]uild/autoaim"

AIM_SELECT="${SCRIPT_DIR}/aim_select.conf"

# 图传项目
DEPLOY_WS="/home/zfm/zfm_ws_final/zfm_client"
#DEPLOY_LAUNCH="ros2 launch bringup onboard_sniper.launch.py"
DEPLOY_LAUNCH="ros2 launch bringup local_sniper.launch.py"
DEPLOY_KILL="component_container|decoder_node|hik_camera|video_encoder|serial_bridge"

MODE_FILE="/tmp/mcu_mode"
LOG_DIR="/home/zfm/zfm2026-log/watchdog"
PID_FILE="/tmp/rm_watchdog.pid"
mkdir -p "$LOG_DIR"

if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Already running"
    exit 0
fi
echo $$ > "$PID_FILE"
trap 'rm -f $PID_FILE; exit 0' SIGINT SIGTERM

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_DIR/watchdog.log"; }

get_mode() {
    if [[ -f "$MODE_FILE" ]]; then cat "$MODE_FILE" 2>/dev/null || echo "0"
    else echo "0"; fi
}

select_aim() {
    local sel=1
    if [[ -f "$AIM_SELECT" ]]; then
        sel=$(grep -v '^\s*#' "$AIM_SELECT" | grep -o '^[0-9]\+' | head -1)
        [[ -z "$sel" ]] && sel=1
    fi
    case $sel in
        1) AIM_WS="$AIM1_WS"; AIM_LAUNCH="$AIM1_LAUNCH"; AIM_KILL="$AIM1_KILL"; AIM_LABEL="zfm_autoaim";;
        2) AIM_WS="$AIM2_WS"; AIM_LAUNCH="$AIM2_LAUNCH"; AIM_KILL="$AIM2_KILL"; AIM_LABEL="standalone_autoaim";;
        *) AIM_WS="$AIM1_WS"; AIM_LAUNCH="$AIM1_LAUNCH"; AIM_KILL="$AIM1_KILL"; AIM_LABEL="zfm_autoaim";;
    esac
}

kill_proj() {
    local pat="$1" label="$2"
    # ros2 launch 进程组
    local pids
    pids=$(ps aux | grep "ros2 launch" | grep -v grep | grep -v watchdog | awk '{print $2}') || true
    for p in $pids; do kill -TERM -"$p" 2>/dev/null || true; done
    sleep 1
    for p in $pids; do kill -KILL -"$p" 2>/dev/null || true; done
    # 按进程名杀
    # 杀终端窗口
    ps aux | grep "rm_term_" | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
    ps aux | grep -E "$pat" | grep -v grep | grep -v watchdog | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
    sleep 1
    log "[kill] $label"
}

start_proj() {
    local ws="$1" cmd="$2" label="$3"
    log "[start] $label"
    cd "$ws"
    set +u
    source /opt/ros/humble/setup.bash
    source install/setup.bash 2>/dev/null || true
    set -u

    if command -v gnome-terminal &>/dev/null && [[ -n "${DISPLAY:-}" ]]; then
        # 新终端窗口显示实时输出
        local scr="/tmp/rm_term_${label}.sh"
        cat > "$scr" <<- EOS
			#!/bin/bash
			echo -ne '\033]0;${label}\007'
			echo '=== ${label} ==='
			cd '${ws}'
			set +u
			source /opt/ros/humble/setup.bash
			source install/setup.bash 2>/dev/null
			set -u
			${cmd}
			echo ''
			echo '=== 已退出 (\$?) — 窗口即将关闭 ==='
			sleep 2
		EOS
        chmod +x "$scr"
        gnome-terminal --geometry=100x30 -- bash "$scr" &
    else
        setsid bash -c "$cmd" >> "$LOG_DIR/${label}.log" 2>&1 &
    fi
    log "[start] $label"
}

# ============================================================
# 主循环
# ============================================================

log "===== Watchdog start ====="

# 确保旧 bridge 死了
pkill -9 -f "mcu_bridge" 2>/dev/null || true
rm -f /tmp/mcu_bridge.pid /tmp/ttyACM_mcu /tmp/mcu_mode 2>/dev/null || true
sleep 1

# 启动 bridge
bash "${SCRIPT_DIR}/mcu_bridge.sh" 2>/dev/null || true

# 等 bridge 就绪
for i in $(seq 1 10); do
    if [[ -e /tmp/ttyACM_mcu ]]; then break; fi
    sleep 1
done

echo "0" > "$MODE_FILE" 2>/dev/null || true
select_aim

# 启动自瞄
kill_proj "$AIM_KILL" "$AIM_LABEL"
kill_proj "$DEPLOY_KILL" "deploy"
start_proj "$AIM_WS" "$AIM_LAUNCH" "$AIM_LABEL"
CUR="aim"

while true; do
    sleep 1

    # bridge 死了就重启
    if ! pgrep -f "mcu_bridge.py" > /dev/null 2>&1; then
        log "[loop] bridge dead, restarting"
        bash "${SCRIPT_DIR}/mcu_bridge.sh" 2>/dev/null || true
    fi

    # 读 mode
    mode=$(get_mode)
    want="aim"
    [[ "$mode" == "1" ]] && want="deploy"

    # 调试日志
    log "[loop] mode=$mode want=$want cur=$CUR"

    # 切换
    if [[ "$want" != "$CUR" ]]; then
        log "SWITCH: $CUR -> $want (mode=$mode)"

        if [[ "$want" == "aim" ]]; then
            kill_proj "$DEPLOY_KILL" "deploy"
            select_aim
            start_proj "$AIM_WS" "$AIM_LAUNCH" "$AIM_LABEL"
            CUR="aim"
        else
            kill_proj "$AIM_KILL" "$AIM_LABEL"
            start_proj "$DEPLOY_WS" "$DEPLOY_LAUNCH" "deploy"
            CUR="deploy"
        fi
    fi
done
