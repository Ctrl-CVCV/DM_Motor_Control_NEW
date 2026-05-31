#!/usr/bin/env bash
set -euo pipefail

# Install script for enabling test.py as a systemd service.
# Run with sudo: sudo ./install_autostart.sh

SCRIPT_PATH="$(realpath "$(dirname "$0")/test.py")"
WORKDIR="$(dirname "$SCRIPT_PATH")"
OWNER="${SUDO_USER:-$(whoami)}"
SERVICE_NAME="rdk_test.service"
TEMPLATE="$(dirname "$0")/rdk_test.service.template"

if [ ! -f "$TEMPLATE" ]; then
  echo "模板文件未找到: $TEMPLATE"
  exit 1
fi

SERVICE_TMP="/tmp/$SERVICE_NAME"

sed \
  -e "s|{{USER}}|$OWNER|g" \
  -e "s|{{WORKDIR}}|$WORKDIR|g" \
  -e "s|{{SCRIPT}}|$SCRIPT_PATH|g" \
  "$TEMPLATE" > "$SERVICE_TMP"

sudo mv "$SERVICE_TMP" "/etc/systemd/system/$SERVICE_NAME"
sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE_NAME"

echo "已安装并启动服务: $SERVICE_NAME"
echo "查看状态: sudo systemctl status $SERVICE_NAME"
