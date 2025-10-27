#!/usr/bin/env bash
set -e

# === 設定ファイルの場所 ===
ENV_FILE="$(dirname "$0")/../config/wifi_ap.env"

# === .env から設定を読み込む ===
if [ ! -f "$ENV_FILE" ]; then
  echo "[ERROR] Wi-Fi設定ファイルが見つかりません: $ENV_FILE"
  exit 1
fi

# .envを読み込む
# コメント行と空行を無視
export $(grep -v '^#' "$ENV_FILE" | xargs)

# === 変数確認 ===
echo "[INFO] SSID: ${SSID:-undefined}"
echo "[INFO] INTERFACE: ${INTERFACE:-undefined}"
echo "[INFO] ORIGINAL_CONNECTION: ${ORIGINAL_CONNECTION:-undefined}"

# === 終了時のクリーンアップ ===
cleanup() {
    echo "[INFO] Cleaning up: restoring Wi-Fi connection..."
    nmcli connection down Hotspot || true
    if [ -n "$ORIGINAL_CONNECTION" ]; then
        nmcli connection up "$ORIGINAL_CONNECTION" || true
    fi
}
trap cleanup EXIT INT TERM

# === AP起動 ===
echo "[INFO] Starting AP '${SSID}' on ${INTERFACE}..."
nmcli dev wifi hotspot ifname "$INTERFACE" ssid "$SSID" password "$PASSWORD" || {
    echo "[ERROR] Failed to start AP. Exiting."
    exit 1
}

# === AP起動確認 ===
echo "[INFO] Waiting for AP to become active..."
for i in {1..10}; do
    if nmcli connection show --active | grep -q Hotspot; then
        echo "[INFO] ✅ AP is active."
        break
    fi
    sleep 1
done

# === ROS2 Launch 起動 ===
echo "[INFO] Launching ROS2 system..."
ros2 launch launch/full_system.launch.py use_ap:=false

