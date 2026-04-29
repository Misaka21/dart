#!/bin/bash
#
# dart2026 看门狗脚本
#
# 用法:
#   ./watchdog.sh              # 调试模式
#   ./watchdog.sh --match      # 比赛模式，强制内录
#

set -u

TIMEOUT=15
MAX_RETRY=100
SCREEN_NAME="dart2026"
PROCESS_NAME="dart2026"
RESOURCE_LOG_INTERVAL=5

ENABLE_REALTIME=true
NICE_LEVEL=-15
RT_PRIORITY=50
ENABLE_COREDUMP=true
# CPU_AFFINITY="0-3"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
LOG_DIR="$PROJECT_DIR/log"
EXECUTABLE="$BUILD_DIR/dart2026"
ARGS="$*"

RETRY_COUNT=0
SESSION_DIR=""
RESOURCE_LOG=""
LAST_RESOURCE_LOG_TIME=0

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

get_timestamp() {
    date '+%Y-%m-%d_%H-%M-%S'
}

create_session_dir() {
    local suffix=""
    if echo "$ARGS" | grep -q -- "--match\\|-m"; then
        suffix="_match"
    fi

    SESSION_DIR="$LOG_DIR/$(get_timestamp)$suffix"
    mkdir -p "$SESSION_DIR"
    log_msg "Session: $SESSION_DIR"
}

kill_screen() {
    if screen -list 2>/dev/null | grep -q "$SCREEN_NAME"; then
        log_msg "Killing existing screen session..."
        screen -S "$SCREEN_NAME" -X quit 2>/dev/null || true
        sleep 1
    fi
}

set_priority() {
    local pid="$1"
    [ -z "$pid" ] && return

    renice "$NICE_LEVEL" -p "$pid" >/dev/null 2>&1 && log_msg "  Nice set to $NICE_LEVEL"

    if $ENABLE_REALTIME; then
        if chrt -f -p "$RT_PRIORITY" "$pid" 2>/dev/null; then
            log_msg "  RT priority set to SCHED_FIFO:$RT_PRIORITY"
        elif chrt -r -p "$RT_PRIORITY" "$pid" 2>/dev/null; then
            log_msg "  RT priority set to SCHED_RR:$RT_PRIORITY"
        fi
    fi

    if [ -n "${CPU_AFFINITY:-}" ]; then
        taskset -cp "$CPU_AFFINITY" "$pid" >/dev/null 2>&1 && log_msg "  CPU affinity set to $CPU_AFFINITY"
    fi
}

start_screen() {
    log_msg "Starting dart2026 in screen session '$SCREEN_NAME'..."
    cd "$BUILD_DIR" || exit 1

    if $ENABLE_COREDUMP; then
        ulimit -c unlimited
        log_msg "Coredump enabled"
    fi

    local screen_log="$SESSION_DIR/screen.log"
    screen -dmS "$SCREEN_NAME" -L -Logfile "$screen_log" \
        bash -c "cd \"$BUILD_DIR\" && \"$EXECUTABLE\" $ARGS --log-dir \"$SESSION_DIR\" 2>&1; echo '[dart2026 exited with code '\$?']'; exec bash"

    sleep 3

    local pid
    pid=$(pgrep -x "$PROCESS_NAME" | head -1)
    if [ -n "$pid" ]; then
        log_msg "dart2026 PID: $pid"
        set_priority "$pid"
    fi
}

check_screen() {
    screen -list 2>/dev/null | grep -q "$SCREEN_NAME"
}

check_process() {
    pgrep -x "$PROCESS_NAME" >/dev/null
}

check_heartbeat() {
    [ -z "$SESSION_DIR" ] && return 0

    local heartbeat_file="$SESSION_DIR/heartbeat"
    [ ! -f "$heartbeat_file" ] && return 0

    local file_age
    file_age=$(($(date +%s) - $(stat -c %Y "$heartbeat_file" 2>/dev/null || echo 0)))
    [ "$file_age" -le "$TIMEOUT" ]
}

show_status() {
    local screen_ok=false
    local process_ok=false
    local heartbeat_ok=false
    local heartbeat_age="-"
    local heartbeat_file="$SESSION_DIR/heartbeat"

    check_screen && screen_ok=true
    check_process && process_ok=true

    if [ -f "$heartbeat_file" ]; then
        heartbeat_age=$(($(date +%s) - $(stat -c %Y "$heartbeat_file" 2>/dev/null || echo 0)))
        [ "$heartbeat_age" -le "$TIMEOUT" ] && heartbeat_ok=true
    fi

    echo "──────────────────────────────────────"
    echo "[$(date '+%H:%M:%S')] screen: $screen_ok  process: $process_ok  heartbeat: $heartbeat_ok (${heartbeat_age}s)"
    tail -n +4 "$heartbeat_file" 2>/dev/null | head -5 | sed 's/^/    /' || true
}

init_resource_log() {
    [ -z "$SESSION_DIR" ] && return
    RESOURCE_LOG="$SESSION_DIR/resources.csv"
    echo "timestamp,dart_cpu%,dart_rss_mb,dart_vsz_mb,sys_mem_used_mb,sys_mem_total_mb,cpu_temp_c" > "$RESOURCE_LOG"
}

log_resources() {
    local current_time
    current_time=$(date +%s)
    [ $((current_time - LAST_RESOURCE_LOG_TIME)) -lt "$RESOURCE_LOG_INTERVAL" ] && return
    LAST_RESOURCE_LOG_TIME=$current_time

    [ -z "$RESOURCE_LOG" ] && init_resource_log
    [ -z "$RESOURCE_LOG" ] && return

    local timestamp
    local dart_cpu=0
    local dart_rss=0
    local dart_vsz=0
    local sys_mem_used=0
    local sys_mem_total=0
    local cpu_temp=0
    timestamp=$(date '+%Y-%m-%d %H:%M:%S')

    local pid
    pid=$(pgrep -x "$PROCESS_NAME" | head -1)
    if [ -n "$pid" ]; then
        local ps_output
        ps_output=$(ps -p "$pid" -o %cpu=,rss=,vsz= 2>/dev/null | tr -s ' ')
        if [ -n "$ps_output" ]; then
            dart_cpu=$(echo "$ps_output" | awk '{print $1}')
            dart_rss=$(echo "$ps_output" | awk '{printf "%.1f", $2/1024}')
            dart_vsz=$(echo "$ps_output" | awk '{printf "%.1f", $3/1024}')
        fi
    fi

    local mem_info
    mem_info=$(free -m 2>/dev/null || true)
    if [ -n "$mem_info" ]; then
        sys_mem_total=$(echo "$mem_info" | awk '/^Mem:/ {print $2}')
        sys_mem_used=$(echo "$mem_info" | awk '/^Mem:/ {print $3}')
    fi

    if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
        cpu_temp=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "0")
    fi

    echo "$timestamp,$dart_cpu,$dart_rss,$dart_vsz,$sys_mem_used,$sys_mem_total,$cpu_temp" >> "$RESOURCE_LOG"
}

save_coredump() {
    $ENABLE_COREDUMP || return
    [ -z "$SESSION_DIR" ] && return

    find "$BUILD_DIR" /tmp -maxdepth 1 -name "core*" -mmin -5 2>/dev/null | while read -r core_file; do
        [ -f "$core_file" ] || continue
        mv "$core_file" "$SESSION_DIR/$(basename "$core_file")" 2>/dev/null || true
    done
}

bringup() {
    kill_screen
    pkill -TERM -x "$PROCESS_NAME" 2>/dev/null || true
    sleep 2
    pkill -9 -x "$PROCESS_NAME" 2>/dev/null || true
    sleep 1
    start_screen
}

restart() {
    log_msg "Restarting dart2026..."
    save_coredump

    RETRY_COUNT=$((RETRY_COUNT + 1))
    if [ "$RETRY_COUNT" -gt "$MAX_RETRY" ]; then
        log_msg "[FATAL] Max retry ($MAX_RETRY) reached!"
        exit 1
    fi

    log_msg "Retry $RETRY_COUNT / $MAX_RETRY"
    bringup
    sleep $((TIMEOUT * 2))
}

cleanup() {
    log_msg "Received exit signal, cleaning up..."
    kill_screen
    pkill -x "$PROCESS_NAME" 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

if [ ! -x "$EXECUTABLE" ]; then
    echo "Error: executable not found: $EXECUTABLE"
    echo "Please build first: cmake --build \"$BUILD_DIR\" --target dart2026 -j"
    exit 1
fi

mkdir -p "$LOG_DIR"
create_session_dir

WATCHDOG_LOG="$SESSION_DIR/watchdog.log"
exec > >(tee -a "$WATCHDOG_LOG") 2>&1

echo ""
echo "============================================"
echo "  dart2026 Watchdog"
echo "============================================"
log_msg "Project Dir: $PROJECT_DIR"
log_msg "Executable:  $EXECUTABLE"
log_msg "Args:        $ARGS"
log_msg "Timeout:     ${TIMEOUT}s"
log_msg "Max Retry:   $MAX_RETRY"
log_msg "Screen:      $SCREEN_NAME"
log_msg "Realtime:    $ENABLE_REALTIME (nice=$NICE_LEVEL, rt=$RT_PRIORITY)"
echo ""

bringup
sleep 3

CHECK_INTERVAL=2

while true; do
    show_status

    if ! check_screen; then
        log_msg "[ERROR] Screen session lost!"
        restart
        continue
    fi

    if ! check_process; then
        log_msg "[ERROR] dart2026 process not running!"
        restart
        continue
    fi

    if ! check_heartbeat; then
        log_msg "[ERROR] Heartbeat timeout!"
        restart
        continue
    fi

    log_resources
    RETRY_COUNT=0
    sleep "$CHECK_INTERVAL"
done
