#!/bin/bash
# Write Dynamic_Waypoint/dynamic_mission.txt (single mission file in this dir).
#
# Usage:
#   ./create_dynamic_mission.sh /path/to/waypoints.txt
#   ./create_dynamic_mission.sh   # copies from example if no arg (edit paths first)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SCRIPT_DIR/dynamic_mission.txt"
TMP="$SCRIPT_DIR/.dynamic_mission.txt.tmp"

if [ -n "${1:-}" ]; then
    SRC="$(realpath "$1")"
    if [ ! -f "$SRC" ]; then
        echo "ERROR: source not found: $SRC" >&2
        exit 1
    fi
else
    echo "Usage: $0 <waypoints.txt>" >&2
    exit 1
fi

# Enforce single mission file: remove previous dynamic mission before writing.
rm -f "$OUT" "$TMP"

cp "$SRC" "$TMP"
chmod 644 "$TMP"
mv "$TMP" "$OUT"

echo "Wrote mission to: $OUT"
wc -l "$OUT"
