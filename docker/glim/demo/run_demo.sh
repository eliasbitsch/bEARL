#!/bin/bash
# Live demo: M3DGR bag playback -> GLIM -> RViz (Windows 11: Docker Desktop + WSLg; Linux: X11).
#   bash docker/glim/demo/run_demo.sh [sequence] [rate]      e.g.  bash docker/glim/demo/run_demo.sh Varying-illu01 1.0
# Needs data/M3DGR/<seq>/glim_demo/ and glim_demo_tf.txt from docker/glim/demo/make_demo_bag.py.
set -e
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
SEQ="${1:-Varying-illu01}"
DATA="$ROOT/data/M3DGR/$SEQ"
[ -d "$DATA/glim_demo" ] || { echo "missing $DATA/glim_demo - run docker/glim/demo/make_demo_bag.py first"; exit 1; }
docker image inspect bearl/glim_demo >/dev/null 2>&1 || docker build -t bearl/glim_demo "$ROOT/docker/glim/demo"

if [ -d /run/desktop ] || [ -n "$WINDIR" ]; then   # Docker Desktop on Windows: use WSLg's display
  DISPLAY_ARGS=(-e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
                -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -v /run/desktop/mnt/host/wslg:/mnt/wslg)
else
  DISPLAY_ARGS=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
host() { if command -v cygpath >/dev/null; then cygpath -w "$1"; else echo "$1"; fi; }

MSYS_NO_PATHCONV=1 docker run --rm $([ -t 0 ] && echo -it) --name glim_demo --gpus all "${DISPLAY_ARGS[@]}" -e RATE="${2:-1.0}" \
  -v "$(host "$ROOT/docker/glim/config"):/glim/config:ro" \
  -v "$(host "$ROOT/docker/glim/demo"):/demo:ro" \
  -v "$(host "$DATA"):/data:ro" \
  bearl/glim_demo bash /demo/run_in_container.sh
