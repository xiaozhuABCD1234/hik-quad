#!/usr/bin/env bash
# Interactive chessboard capture for the 4-camera rig.
#
# Usage:
#   ./scripts/capture_calibration.sh [view_count] [output_dir]
#
# For every view: move the board, press Enter, the script grabs one synchronized
# frame set into output_dir/view_XXX/cam0.png ... cam3.png, then runs the
# detection check.  Finally it invokes hik_quad_calib.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="${ROOT}/build/release/apps/hik_quad_capture"
CALIB="${ROOT}/build/release/apps/hik_quad_calib"
COUNT="${1:-20}"
OUT="${2:-${ROOT}/data/calib_images}"
EXPOSURE="${HIK_EXPOSURE_US:-15000}"
GAIN="${HIK_GAIN_DB:-0}"
FPS="${HIK_FPS:-20}"
PANO="${HIK_PANO:-4096x1024}"

if [ ! -x "${APP}" ] || [ ! -x "${CALIB}" ]; then
  echo "error: build first: cmake --preset release && cmake --build --preset release" >&2
  exit 1
fi

mkdir -p "${OUT}"
echo "board: 8x11 inner corners, 100 mm squares"
echo "output: ${OUT}"
echo "views:  ${COUNT}"
echo
echo "Move the board between shots. Cover the centre, corners and all four edges."
echo "Press Ctrl-C to abort."
echo

for ((i = 0; i < COUNT; ++i)); do
  printf 'view %03d: place the board, then press Enter... ' "${i}"
  read -r _
  target="${OUT}/view_$(printf '%03d' "${i}")"
  # Free-run at 20 fps: the two cameras behind the 5 Gbps HUB can return
  # truncated frames at 30 fps; 20 fps is the verified-complete default.
  "${APP}" --frames 1 --fps "${FPS}" \
    --exposure "${EXPOSURE}" --gain "${GAIN}" --out "${target}"
  echo
done

echo
echo "== detection check =="
"${CALIB}" --check --input "${OUT}" || true

echo
echo "== calibration =="
"${CALIB}" --input "${OUT}" --output "${ROOT}/data/calib" \
  --pano "${PANO}" --fov 360 --horizontal
