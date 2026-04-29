#!/bin/bash
#
# dart2026 systemd 服务安装脚本
#
# 用法: sudo ./install_service.sh
#

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 请使用 sudo 运行"
    echo "用法: sudo ./install_service.sh"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
SERVICE_TEMPLATE="$SCRIPT_DIR/dart2026.service"

REAL_USER="${SUDO_USER:-$USER}"
REAL_GROUP="$(id -gn "$REAL_USER")"

echo ""
echo "============================================"
echo "  dart2026 Service Installer"
echo "============================================"
echo "User:        $REAL_USER"
echo "Group:       $REAL_GROUP"
echo "Project Dir: $PROJECT_DIR"
echo ""

if [ ! -f "$SERVICE_TEMPLATE" ]; then
    echo "错误: 找不到 $SERVICE_TEMPLATE"
    exit 1
fi

if [ ! -x "$BUILD_DIR/dart2026" ]; then
    echo "警告: 可执行文件不存在 $BUILD_DIR/dart2026"
    echo "请先编译: cmake --build $BUILD_DIR --target dart2026 -j"
    read -r -p "是否继续安装? [y/N] " reply
    if [[ ! "$reply" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

chmod +x "$SCRIPT_DIR/watchdog.sh"
chmod +x "$SCRIPT_DIR/cleanup.sh"

TMP_SERVICE="/tmp/dart2026.service"
sed -e "s|User=dart|User=$REAL_USER|g" \
    -e "s|Group=dart|Group=$REAL_GROUP|g" \
    -e "s|/home/dart/dart2026|$PROJECT_DIR|g" \
    "$SERVICE_TEMPLATE" > "$TMP_SERVICE"

cp "$TMP_SERVICE" /etc/systemd/system/dart2026.service
chmod 644 /etc/systemd/system/dart2026.service

LIMITS_FILE="/etc/security/limits.d/dart2026.conf"
cat > "$LIMITS_FILE" << EOF
# dart2026 实时优先级权限
$REAL_USER    -    rtprio    99
$REAL_USER    -    nice      -20
$REAL_USER    -    memlock   unlimited
EOF
chmod 644 "$LIMITS_FILE"

systemctl daemon-reload
systemctl enable dart2026.service

echo ""
echo "============================================"
echo "  安装完成"
echo "============================================"
echo ""
echo "使用方法:"
echo "  sudo systemctl start dart2026"
echo "  sudo systemctl stop dart2026"
echo "  sudo systemctl restart dart2026"
echo "  sudo systemctl status dart2026"
echo "  journalctl -u dart2026 -f"
echo ""
echo "调试:"
echo "  screen -r dart2026"
echo "  $SCRIPT_DIR/watchdog.sh --match"
echo "  $SCRIPT_DIR/watchdog.sh"
echo ""

read -r -p "是否立即启动服务? [y/N] " reply
if [[ "$reply" =~ ^[Yy]$ ]]; then
    systemctl start dart2026
    systemctl status dart2026 --no-pager
fi
