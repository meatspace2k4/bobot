#!/bin/sh
# Sim launch configurations.
#   viewer  normal view: HUD, head-camera inset, history chart, scene tabs, follow camera
#   dev     debugger view: MuJoCo's stock viewer and panels (for debugging)
#   twin    normal view opened on the live scene: mirrors / drives the real Beni (tools/nav/run_teleop.py)
#   bench   headless bench Beni on udp 127.0.0.1:19007 for beni_bridge.py / the beni SDK
# Needs: third_party/open-beni (git clone https://github.com/Mondo-Robotics/open-beni),
#        .venv-sim (python3 -m venv .venv-sim && .venv-sim/bin/pip install mujoco onnxruntime numpy),
#        captures/traces (tools/topic_trace.py output).
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
VENV="$ROOT/.venv-sim/bin"
OB="$ROOT/third_party/open-beni"
TR="$ROOT/captures/traces"
case "${1:-viewer}" in
  viewer) exec "$VENV/mjpython" "$ROOT/tools/sim/beni_view.py" --open-beni "$OB" --traces "$TR" ;;
  dev)    exec "$VENV/mjpython" "$ROOT/tools/sim/beni_view.py" --open-beni "$OB" --traces "$TR" --dev ;;
  twin)   exec "$VENV/mjpython" "$ROOT/tools/sim/beni_view.py" --open-beni "$OB" --traces "$TR" --start live ;;
  bench)  exec "$VENV/python" "$ROOT/tools/sim/beni_sim.py" --open-beni "$OB" ;;
  *) echo "usage: $0 [viewer|dev|twin|bench]" >&2; exit 2 ;;
esac
