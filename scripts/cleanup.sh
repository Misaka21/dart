#!/bin/bash
#
# 停止 dart2026 systemd 服务、watchdog、screen 和残留进程。
#

echo "[$(date '+%H:%M:%S')] Cleaning up dart2026..."

if systemctl is-active --quiet dart2026 2>/dev/null; then
    echo "Stopping systemd service..."
    sudo systemctl stop dart2026
fi

if screen -list 2>/dev/null | grep -q dart2026; then
    echo "Killing screen session..."
    screen -S dart2026 -X quit 2>/dev/null || true
fi

if pgrep -f "[s]cripts/watchdog.sh" >/dev/null; then
    echo "Killing watchdog..."
    pkill -f "[s]cripts/watchdog.sh" || true
fi

if pgrep -x dart2026 >/dev/null; then
    echo "Killing dart2026..."
    pkill -TERM -x dart2026 || true
    sleep 1
    pkill -9 -x dart2026 || true
fi

echo "[$(date '+%H:%M:%S')] Cleanup done."
echo ""
echo "进程检查:"
pgrep -a -f "[d]art2026|[w]atchdog.sh" || echo "  (无 dart2026 相关进程)"
