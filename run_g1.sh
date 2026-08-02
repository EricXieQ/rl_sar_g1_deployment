#!/bin/bash
# Run the standalone (CMake) G1 binary -- this is the one with model preloading.
# NOT the ROS2 path (`ros2 run rl_sar ...`), which launches a stale colcon build
# without the preload code. No ROS sourcing needed; the binary is self-contained.
set -euo pipefail

cd "$(dirname "$0")"

IFACE="${1:-eth0}"
LOG="${2:-/tmp/sim.log}"

# A leftover controller from a previous run keeps publishing to rt/lowcmd. Two
# publishers send the robot contradictory motor commands, overload protection
# trips, and the new instance looks like it hangs at startup. Refuse to launch
# rather than let that happen silently -- killing is left to the operator, since
# SIGKILLing a live controller mid-motion is its own hazard.
# -x matches the process NAME exactly, not the command line: `pgrep -f` would
# also hit any shell, editor or grep that merely mentions the binary's path.
if stale=$(pgrep -ax rl_real_g1); then
    echo "[run_g1] ABORT: rl_real_g1 is already running:" >&2
    echo "$stale" | sed 's/^/    /' >&2
    echo >&2
    echo "  Make sure the robot is safe, then: pkill -x rl_real_g1" >&2
    echo "  Verify it is gone with: pgrep -ax rl_real_g1" >&2
    exit 1
fi

echo "[run_g1] no stale controller -- ok"
echo "[run_g1] launching ./cmake_build/bin/rl_real_g1 on '$IFACE' (log -> $LOG)"
./cmake_build/bin/rl_real_g1 "$IFACE" 2>&1 | tee "$LOG"
