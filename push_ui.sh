#!/bin/bash
# =============================================================
# M.A.S.S. Trap — Remote LittleFS UI Pusher
# Uploads v2.5.0 web UI files to a device over HTTP
#
# Usage:
#   ./push_ui.sh <hostname> [api_key]
#
# Examples:
#   ./push_ui.sh masstrap-start.local
#   ./push_ui.sh masstrap-start.local admin
#   ./push_ui.sh 192.168.1.42 your-ota-password
# =============================================================

set -e

HOST="${1:?Usage: $0 <hostname> [api_key]}"
API_KEY="${2:-admin}"
DATA_DIR="$(cd "$(dirname "$0")/data" && pwd)"

# Files to push (new v2.5.0 UI — skip old index.html and config.html)
FILES=(
  style.css
  main.js
  dashboard.html
  history.html
  system.html
  console.html
  start_status.html
  speedtrap_status.html
)

echo "========================================"
echo "M.A.S.S. Trap — Remote UI Push v2.5.0"
echo "========================================"
echo "Target:  http://${HOST}"
echo "API Key: ${API_KEY:0:3}***"
echo "Source:  ${DATA_DIR}"
echo ""

# Pre-flight: check device is reachable
echo -n "Checking device... "
if ! curl -s --connect-timeout 3 "http://${HOST}/api/version" > /dev/null 2>&1; then
  echo "FAILED — cannot reach http://${HOST}/api/version"
  echo "Make sure the device is on and connected to your network."
  exit 1
fi
VERSION=$(curl -s "http://${HOST}/api/version")
echo "OK — ${VERSION}"
echo ""

TOTAL=${#FILES[@]}
SUCCESS=0
FAIL=0

for FILE in "${FILES[@]}"; do
  FILEPATH="${DATA_DIR}/${FILE}"
  if [ ! -f "$FILEPATH" ]; then
    echo "  SKIP  /${FILE} — file not found in data/"
    continue
  fi

  SIZE=$(wc -c < "$FILEPATH" | tr -d ' ')
  echo -n "  PUSH  /${FILE} (${SIZE} bytes)... "

  HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" \
    -X POST \
    -H "X-API-Key: ${API_KEY}" \
    -H "Content-Type: application/octet-stream" \
    --data-binary "@${FILEPATH}" \
    "http://${HOST}/api/files?path=/${FILE}")

  if [ "$HTTP_CODE" = "200" ]; then
    echo "OK"
    SUCCESS=$((SUCCESS + 1))
  else
    echo "FAILED (HTTP ${HTTP_CODE})"
    FAIL=$((FAIL + 1))
  fi
done

echo ""
echo "========================================"
echo "Done: ${SUCCESS}/${TOTAL} files pushed, ${FAIL} failed"
echo "========================================"

if [ "$FAIL" -gt 0 ]; then
  echo ""
  echo "Some files failed. Check:"
  echo "  - Is the API key correct? (default: admin)"
  echo "  - Does the device have enough LittleFS space?"
  exit 1
fi

echo ""
echo "UI updated! Refresh your browser to see changes."
echo "No reboot needed — files are served directly from LittleFS."
