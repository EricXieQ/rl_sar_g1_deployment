#!/usr/bin/env python3
"""
live_temps.py -- watch G1 motor temperatures during a capture session.

The ASAP paper broke two G1 robots collecting real-world data ("highly dynamic
motions cause rapid overheating of joint motors"), so this matters for long runs.

    /usr/bin/python3 ~/qxie2/rl_sar/scripts/live_temps.py
    ... --every 15        # seconds between samples (default 20)
    ... --warn 65 --stop 80

Each motor reports two temperatures (rotor and driver board); the higher is used.
Read-only -- it subscribes to rt/lowstate and touches nothing.
"""
import argparse, sys, time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

NAMES = ["L_HipP","L_HipR","L_HipY","L_Knee","L_AnkP","L_AnkR",
         "R_HipP","R_HipR","R_HipY","R_Knee","R_AnkP","R_AnkR",
         "WaistY","WaistR","WaistP",
         "L_ShldP","L_ShldR","L_ShldY","L_Elb","L_WrR","L_WrP","L_WrY",
         "R_ShldP","R_ShldR","R_ShldY","R_Elb","R_WrR","R_WrP","R_WrY"]


def temps(msg):
    out = []
    for i in range(29):
        v = msg.motor_state[i].temperature
        out.append(max(v) if hasattr(v, "__len__") else v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--every", type=float, default=20.0)
    ap.add_argument("--warn", type=float, default=65.0, help="take a break above this")
    ap.add_argument("--stop", type=float, default=80.0, help="stop and cool above this")
    ap.add_argument("--iface", default="eth0")
    args = ap.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init()
    if sub.Read(3000) is None:
        print("no rt/lowstate -- is the robot in development mode?")
        return

    print("motor temperatures  (warn %.0f C, stop %.0f C)   Ctrl+C to quit\n"
          % (args.warn, args.stop))
    print("%-9s %6s %6s   %-38s %s" % ("time", "max", "median", "hottest three", "trend"))
    print("-" * 84)

    t0 = time.time()
    first_max = None
    prev_max = None
    try:
        while True:
            m = sub.Read(2000)
            if m is None:
                print("  (lost rt/lowstate)")
                time.sleep(args.every); continue
            t = temps(m)
            pairs = sorted(zip(NAMES, t), key=lambda x: -x[1])
            mx = pairs[0][1]
            med = sorted(t)[len(t) // 2]
            if first_max is None:
                first_max = mx
            trend = "" if prev_max is None else ("+%.0f" % (mx - prev_max) if mx >= prev_max
                                                 else "%.0f" % (mx - prev_max))
            prev_max = mx

            flag = ""
            if mx >= args.stop:
                flag = "   *** STOP AND COOL ***"
            elif mx >= args.warn:
                flag = "   <-- take a break"

            hot = "  ".join("%s %.0f" % (n, v) for n, v in pairs[:3])
            el = time.time() - t0
            print("%-9s %5.0fC %5.0fC   %-38s %-5s (since start %+.0f)%s"
                  % ("%.0fm%02.0fs" % (el // 60, el % 60), mx, med, hot, trend,
                     mx - first_max, flag))
            time.sleep(args.every)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
