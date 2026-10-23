#!/usr/bin/env bash
# Save the Gazebo GUI and RViz windows of the running demo as PNG files.
# (The OS blocks whole-desktop screenshots here; `xwd -id` of each X window is
# allowed.) A window drawn over another one appears in the other's picture.
# Usage: capture_windows.sh OUT_DIR [duration_s] [interval_s]
set -u
OUT=${1:?out dir}; DUR=${2:-400}; EVERY=${3:-25}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
end=$((SECONDS + DUR)); n=0
while [ $SECONDS -lt $end ]; do
    sleep "$EVERY"
    for pat in "RViz:rviz" "Gazebo Sim:gazebo"; do
        title=${pat%%:*}; tag=${pat##*:}
        id=$(xwininfo -root -tree 2>/dev/null | grep -E "\"$title\"|$title\"" | grep -v "mutter" | grep -E "rviz2|gz-sim-gui" | head -1 | awk '{print $1}')
        [ -z "$id" ] && continue
        f="$OUT/${tag}_$(printf '%04d' $n)"
        xwd -id "$id" -silent > "$f.xwd" 2>/dev/null && \
            python3 "$HERE/xwd2png.py" "$f.xwd" "$f.png" 2>/dev/null; rm -f "$f.xwd"
    done
    n=$((n + 1))
done
