#!/bin/bash
# Run the standalone (CMake) G1 binary -- this is the one with model preloading.
# NOT the ROS2 path (`ros2 run rl_sar ...`), which launches a stale colcon build
# without the preload code. No ROS sourcing needed; the binary is self-contained.
set -euo pipefail

cd "$(dirname "$0")"

IFACE="${1:-eth0}"
LOG="${2:-/tmp/sim.log}"

echo "[run_g1] launching ./cmake_build/bin/rl_real_g1 on '$IFACE' (log -> $LOG)"
./cmake_build/bin/rl_real_g1 "$IFACE" 2>&1 | tee "$LOG"
