#!/bin/bash
# =============================================================
# M.A.S.S. Trap — Safe Flash Script
# Uploads firmware + LittleFS WITHOUT losing config/data
#
# This script:
#   1. Backs up config.json, garage.json, history.json, peers.json
#      from the device via HTTP API (if reachable)
#   2. Flashes firmware via USB
#   3. Flashes LittleFS via USB
#   4. Waits for device to boot into setup mode AP
#   5. Restores backed-up files via HTTP API
#
# Usage:
#   ./flash_safe.sh [device_ip]
#
# If device_ip is provided, config is backed up over the network
# before flashing. If omitted, uses any previously saved backup.
# =============================================================

set -e

DEVICE_IP="${1:-}"
BACKUP_DIR="$(cd "$(dirname "$0")" && pwd)/.flash_backup"
PIO="/Users/admin/.platformio-venv/bin/pio"

echo "========================================"
echo "M.A.S.S. Trap — Safe Flash"
echo "========================================"

# Step 1: Backup config from device (if reachable)
mkdir -p "$BACKUP_DIR"

if [ -n "$DEVICE_IP" ]; then
  echo ""
  echo "[1/5] Backing up device data from $DEVICE_IP..."

  for FILE in config.json garage.json history.json peers.json; do
    RESP=$(curl -s --connect-timeout 3 "http://${DEVICE_IP}/api/files?path=/${FILE}" 2>/dev/null)
    if [ -n "$RESP" ]; then
      # Extract the "content" field from the API response
      CONTENT=$(echo "$RESP" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('content',''))" 2>/dev/null)
      if [ -n "$CONTENT" ]; then
        echo "$CONTENT" > "$BACKUP_DIR/$FILE"
        echo "  OK  /${FILE} ($(wc -c < "$BACKUP_DIR/$FILE" | tr -d ' ') bytes)"
      else
        echo "  SKIP /${FILE} — empty or not found"
      fi
    else
      echo "  SKIP /${FILE} — device unreachable"
    fi
  done
elif [ -f "$BACKUP_DIR/config.json" ]; then
  echo ""
  echo "[1/5] Using existing backup from $BACKUP_DIR"
  ls -la "$BACKUP_DIR"/*.json 2>/dev/null
else
  echo ""
  echo "[1/5] WARNING: No device IP provided and no backup found!"
  echo "       Config will be LOST after uploadfs."
  echo "       Press Ctrl+C to abort, or Enter to continue..."
  read -r
fi

# Step 2: Flash firmware
echo ""
echo "[2/5] Flashing firmware via USB..."
$PIO run -e mass-trap -t upload

# Step 3: Flash LittleFS
echo ""
echo "[3/5] Flashing LittleFS via USB..."
$PIO run -e mass-trap -t uploadfs

# Step 4: Wait for device to boot
echo ""
echo "[4/5] Waiting for device to boot (20 seconds)..."
sleep 20

# Step 5: Restore config files
# The device is now in setup mode on 192.168.4.1 AP, OR on the network
# Try both paths
echo ""
echo "[5/5] Restoring config files..."

RESTORE_TARGET=""
# First check if device came back on the network (config recovery worked)
if [ -n "$DEVICE_IP" ]; then
  if curl -s --connect-timeout 3 "http://${DEVICE_IP}/api/version" > /dev/null 2>&1; then
    RESTORE_TARGET="$DEVICE_IP"
    echo "  Device found on network at $DEVICE_IP"
  fi
fi

# If not on network, try setup mode AP
if [ -z "$RESTORE_TARGET" ]; then
  if curl -s --connect-timeout 3 "http://192.168.4.1/api/version" > /dev/null 2>&1; then
    RESTORE_TARGET="192.168.4.1"
    echo "  Device found on setup AP at 192.168.4.1"
  fi
fi

if [ -z "$RESTORE_TARGET" ]; then
  echo "  Device not reachable. Manual restore needed."
  echo "  Backup files saved in: $BACKUP_DIR"
  echo ""
  echo "  To restore manually:"
  echo "    1. Connect to the device's setup AP"
  echo "    2. Run: curl -X POST -H 'X-API-Key: admin' \\"
  echo "       -H 'Content-Type: application/octet-stream' \\"
  echo "       --data-binary @$BACKUP_DIR/config.json \\"
  echo "       'http://192.168.4.1/api/files?path=/config.json'"
  echo "    3. Reboot the device"
  exit 1
fi

# Restore each file
for FILE in config.json garage.json history.json peers.json; do
  if [ -f "$BACKUP_DIR/$FILE" ]; then
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" \
      -X POST \
      -H "X-API-Key: admin" \
      -H "Content-Type: application/octet-stream" \
      --data-binary "@$BACKUP_DIR/$FILE" \
      "http://${RESTORE_TARGET}/api/files?path=/${FILE}" 2>/dev/null)

    if [ "$HTTP_CODE" = "200" ]; then
      echo "  OK  /${FILE} restored"
    else
      echo "  FAIL /${FILE} (HTTP ${HTTP_CODE})"
    fi
  fi
done

echo ""
echo "========================================"
echo "Flash complete! Device should reboot with restored config."
echo "========================================"
echo ""
echo "If device is still in setup mode, trigger a reboot:"
echo "  curl -X POST -H 'X-API-Key: admin' http://${RESTORE_TARGET}/api/reset"
