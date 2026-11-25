#!/usr/bin/env bash
# The ESP-IDF build-only floor: how long the firmware takes to rebuild after a one-line renderer
# edit, before any flashing. The robot isn't touched.
#
#   host/live/idf-floor.sh [edits]     (default 8)
#
# Builds into host/build/idf-floor with its own sdkconfig, so build/ and sdkconfig stay as they
# are, with ccache off so repeated edits can't hit its cache. The firmware embeds the Wi-Fi
# networks from secrets/, so the build directory is deleted afterwards. Writes
# host/build/live/idf-floor.json.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="$REPO/host/build/idf-floor"
FILE="$REPO/main/status_view.cpp"
EDITS="${1:-8}"
export TMPDIR="$REPO/host/build/tmp" IDF_CCACHE_ENABLE=0 PYTHONDONTWRITEBYTECODE=1
mkdir -p "$TMPDIR" "$REPO/host/build/live"

if ! git -C "$REPO" diff --quiet -- main/status_view.cpp; then
  echo '{"status":"infra","error":"main/status_view.cpp has uncommitted changes"}'
  exit 4
fi
ORIGINAL="$(mktemp "$TMPDIR/status_view.XXXXXX")"
cp -p "$FILE" "$ORIGINAL"
cleanup() {
  cmp -s "$ORIGINAL" "$FILE" || cp "$ORIGINAL" "$FILE"
  rm -f "$ORIGINAL"
  rm -rf "$BUILD"
}
trap cleanup EXIT

. "$HOME/esp/v6.1.1/esp-idf/export.sh" >/dev/null 2>&1
idf() { idf.py -C "$REPO" -B "$BUILD" -D SDKCONFIG="$BUILD/sdkconfig" build >"$BUILD.log" 2>&1; }
ms() { python3 -c 'import time; print(int(time.time() * 1000))'; }

start=$(ms)
idf || { echo '{"status":"infra","error":"full build failed, see host/build/idf-floor.log"}'; exit 4; }
full=$(( $(ms) - start ))

times=()
for (( i = 0; i < EDITS; i++ )); do
  if (( i % 2 == 0 )); then
    sed 's/"BalaX", TITLE/"Balax", TITLE/' "$ORIGINAL" >"$FILE"
  else
    cp "$ORIGINAL" "$FILE"
  fi
  start=$(ms)
  idf || { echo "{\"status\":\"infra\",\"error\":\"incremental build $i failed\"}"; exit 4; }
  times+=( $(( $(ms) - start )) )
done
cp "$ORIGINAL" "$FILE"

sorted=($(printf '%s\n' "${times[@]}" | sort -n))
p50=${sorted[$(( (EDITS + 1) / 2 - 1 ))]}
p95=${sorted[$(( (EDITS * 95 + 99) / 100 - 1 ))]}
json="{\"schemaVersion\":1,\"status\":\"pass\",\"fullBuildMs\":$full,\"edits\":$EDITS,\"incrementalMs\":[$(IFS=,; echo "${times[*]}")],\"p50\":$p50,\"p95\":$p95}"
echo "$json" >"$REPO/host/build/live/idf-floor.json"
echo "$json"
