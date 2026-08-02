#!/usr/bin/env bash
# preflight.sh -- verify everything is ready BEFORE putting the robot on the gantry.
# Each check corresponds to a failure that has actually cost a session.
#
#   ~/qxie2/rl_sar/scripts/preflight.sh

RL=/home/unitree/qxie2/rl_sar
ok=0; bad=0
pass(){ echo "  [ OK ] $1"; ok=$((ok+1)); }
fail(){ echo "  [FAIL] $1"; echo "         -> $2"; bad=$((bad+1)); }

echo "=============================================================="
echo " PRE-FLIGHT"
echo "=============================================================="

# 1. no stale controller -------------------------------------------------------
# Two rl_real_g1 instances both publish rt/lowcmd; the robot gets contradictory
# commands, protection trips (red eyes) and the new instance appears to hang.
echo
echo "1. controller"
if ps -eo pid,cmd | grep -q "[r]l_real_g1"; then
    fail "a stale rl_real_g1 is running" "kill it: $(ps -eo pid,cmd | grep '[r]l_real_g1' | awk '{print "kill "$1}')"
else
    pass "no stale rl_real_g1"
fi

# 2. network -------------------------------------------------------------------
# eth0 must hold exactly ONE address. A second subnet makes the Jetson advertise
# two DDS locators and the robot intermittently loses rt/lowstate.
echo
echo "2. network"
naddr=$(ip -4 -br addr show eth0 | tr ' ' '\n' | grep -c '/')
if [ "$naddr" -eq 1 ]; then
    pass "eth0 has one address ($(ip -4 -br addr show eth0 | awk '{print $3}'))"
else
    fail "eth0 has $naddr addresses -- DDS will advertise multiple locators" \
         "remove the extra: sudo nmcli connection modify unitree1 -ipv4.addresses <addr>"
fi
if ping -c1 -W1 192.168.123.161 >/dev/null 2>&1; then
    pass "robot boards reachable"
else
    fail "robot boards unreachable" "check the robot is powered and eth0 is connected"
fi
if ping -c1 -W1 192.168.123.50 >/dev/null 2>&1; then
    pass "Vicon PC reachable at 192.168.123.50"
else
    fail "Vicon PC not reachable at 192.168.123.50" \
         "check its cable to the G1; run 'ipconfig /release && ipconfig /renew' on it"
fi

# 3. robot publishing state ----------------------------------------------------
# rt/lowstate only flows once the robot is in DEVELOPMENT mode (purple/zero-torque
# is not enough). Counting distinct ticks avoids counting one message twice.
echo
echo "3. robot state (rt/lowstate)"
rate=$(timeout 20 /usr/bin/python3 -u -c "
import time
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    ChannelFactoryInitialize(0,'eth0')
    s=ChannelSubscriber('rt/lowstate',LowState_); s.Init()
    if s.Read(3000) is None: print('0'); raise SystemExit
    u=set(); t0=time.time()
    while time.time()-t0<2:
        m=s.Read(100)
        if m is not None: u.add(getattr(m,'tick',None))
    print(int(len(u)/2))
except Exception: print('0')
" 2>/dev/null | tail -1)
if [ "${rate:-0}" -gt 100 ]; then
    pass "publishing at ~${rate} Hz (robot is in development mode)"
else
    fail "rt/lowstate not arriving" "put the robot in DEVELOPMENT mode (purple -> dev), then re-run"
fi

# 4. Vicon stream --------------------------------------------------------------
echo
echo "4. Vicon (OSC on :7000)"
if ss -lun 2>/dev/null | grep -q ":7000"; then
    f=$(ls -t "$RL"/logs/oscpose_*.csv 2>/dev/null | head -1)
    if [ -n "$f" ]; then
        a=$(wc -l < "$f"); sleep 3; b=$(wc -l < "$f")
        if [ "$b" -gt "$a" ]; then
            pass "listener running and receiving (+$((b-a)) rows/3s)"
        else
            fail "listener running but NO data arriving" \
                 "on the Vicon PC: Nexus Live, OSC enabled -> 192.168.123.50, port 7000, markers ticked"
        fi
    else
        pass "listener running (no file yet)"
    fi
else
    fail "no listener on port 7000" \
         "start it: cd $RL/scripts/vicon && python3 osc_listener.py"
fi

# 5. disk ----------------------------------------------------------------------
echo
echo "5. disk"
avail=$(df -BG --output=avail "$RL/logs" | tail -1 | tr -dc '0-9')
if [ "${avail:-0}" -ge 5 ]; then
    pass "${avail} GB free (a 10 min session uses ~1 GB)"
else
    fail "only ${avail} GB free" "clear space in $RL/logs"
fi

echo
echo "=============================================================="
if [ "$bad" -eq 0 ]; then
    echo " READY -- $ok checks passed"
    echo
    echo " terminal 1:  cd $RL/scripts/vicon && python3 osc_listener.py"
    echo " terminal 2:  cd $RL && RL_RECORD=1 ./cmake_build/bin/rl_real_g1 eth0"
    echo
    echo " during: '1' then '5' per rep, one at a time"
    echo " exit:   press '0' (passive), let it settle, THEN Ctrl+C"
else
    echo " NOT READY -- $bad problem(s) above"
fi
echo "=============================================================="
exit $bad
