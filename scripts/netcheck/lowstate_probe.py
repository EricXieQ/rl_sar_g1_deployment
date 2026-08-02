#!/usr/bin/env python3
"""Measure /lowstate arrival jitter and eth0 packet load -- a pre-flight check
for the robot's control link.

READ-ONLY. Subscribes to /lowstate and publishes nothing, so it cannot command
the robot and is safe to run at any time, including alongside rl_real_g1.

Record a baseline once, then re-run whenever you change the network (new cable,
new device on the switch, Vicon streaming, campus uplink) and compare:

    ./probe.sh baseline 60
    #  ... change one thing ...
    ./probe.sh with-vicon 60          # auto-compares against baseline

Which metrics to trust
----------------------
  max_gap_ms      the number that matters. lowstate arrives every ~1 ms; a
                  worst-case gap of tens of ms means the control link stalled.
  eth0_rx_pps     ground truth for what the NIC actually received. If this
                  holds steady, the wire is fine.
  nominal_gap_ms  median inter-arrival. A shift here means delivery genuinely
                  slowed, as opposed to the probe missing samples.
  rate_hz         how many messages *this script* captured. Python drops
                  samples under CPU load, so a drop here with eth0_rx_pps
                  unchanged is a probe artifact, NOT packet loss.

Single large outliers are common -- one 9 ms gap in 60,000 samples is usually
unrelated background noise. Re-run before believing it.
"""
import argparse
import json
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_hg.msg import LowState

RESULTS = Path(__file__).parent / "lowstate_results.json"


def net_counters(iface="eth0"):
    """RX counters for iface straight out of /proc/net/dev."""
    with open("/proc/net/dev") as f:
        for line in f:
            if line.strip().startswith(iface + ":"):
                rx = line.split(":", 1)[1].split()
                return {
                    "rx_bytes": int(rx[0]),
                    "rx_packets": int(rx[1]),
                    "rx_drop": int(rx[3]),
                    "multicast": int(rx[7]),
                }
    raise RuntimeError(f"interface {iface} not found in /proc/net/dev")


class Probe(Node):
    def __init__(self, secs):
        super().__init__("lowstate_probe")
        # BEST_EFFORT matches both best-effort and reliable publishers.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.stamps = []
        self.secs = secs
        self.create_subscription(LowState, "/lowstate", self._cb, qos)
        self.t0 = time.perf_counter()

    def _cb(self, _msg):
        self.stamps.append(time.perf_counter())

    def elapsed(self):
        return time.perf_counter() - self.t0


def summarize(stamps, net_before, net_after, wall):
    if len(stamps) < 2:
        return {"error": f"only {len(stamps)} messages -- is the robot powered?"}

    gaps = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
    gaps_sorted = sorted(gaps)
    nominal = statistics.median(gaps)

    def pct(p):
        return gaps_sorted[min(len(gaps_sorted) - 1, int(len(gaps_sorted) * p))]

    return {
        "messages": len(stamps),
        "duration_s": round(wall, 2),
        "rate_hz": round(len(stamps) / wall, 1),
        "nominal_gap_ms": round(nominal, 3),
        "mean_gap_ms": round(statistics.mean(gaps), 3),
        "p99_gap_ms": round(pct(0.99), 3),
        "p999_gap_ms": round(pct(0.999), 3),
        "max_gap_ms": round(max(gaps), 3),
        "stalls_over_3x": sum(1 for g in gaps if g > 3 * nominal),
        "stalls_over_10x": sum(1 for g in gaps if g > 10 * nominal),
        "eth0_rx_pps": round((net_after["rx_packets"] - net_before["rx_packets"]) / wall, 1),
        "eth0_rx_mbps": round(
            (net_after["rx_bytes"] - net_before["rx_bytes"]) * 8 / 1e6 / wall, 2
        ),
        "eth0_mcast_pps": round((net_after["multicast"] - net_before["multicast"]) / wall, 1),
        "eth0_rx_drop": net_after["rx_drop"] - net_before["rx_drop"],
    }


ROWS = ["max_gap_ms", "p999_gap_ms", "p99_gap_ms", "nominal_gap_ms", "rate_hz",
        "stalls_over_3x", "stalls_over_10x", "eth0_rx_pps", "eth0_rx_mbps"]


def judge(base, run):
    """Return verdict lines for one run against the baseline.

    Deliberately does NOT flag on rate_hz alone -- that measures what this
    script captured, which sags under CPU load even when the link is perfect.
    Real packet loss shows up on the NIC counter too.
    """
    out = []
    nominal = run["nominal_gap_ms"]

    if run["max_gap_ms"] > 20:
        out.append(f"SEVERE: {run['max_gap_ms']:.1f} ms worst-case stall. "
                   "Do not run policies in this configuration.")
    elif run["max_gap_ms"] > 10 * nominal:
        out.append(f"WARN: worst gap {run['max_gap_ms']:.1f} ms is >10x nominal "
                   f"({nominal:.2f} ms). Re-run before believing it -- single "
                   "outliers are usually unrelated background noise.")

    if run["stalls_over_10x"] > base["stalls_over_10x"]:
        n = run["stalls_over_10x"] - base["stalls_over_10x"]
        out.append(f"{n} new severe stalls (>10x nominal). Repeated stalls are "
                   "real in a way a single outlier is not.")

    rate_drop = run["rate_hz"] < 0.95 * base["rate_hz"]
    pps_drop = run["eth0_rx_pps"] < 0.95 * base["eth0_rx_pps"]
    if rate_drop and pps_drop:
        out.append("Capture rate AND NIC packet rate both dropped -- real packet loss.")
    elif rate_drop:
        slowed = run["nominal_gap_ms"] > 1.05 * base["nominal_gap_ms"]
        why = ("delivery genuinely slowed (nominal_gap_ms rose)"
               if slowed else "probe-side only; the link is fine")
        out.append(f"Capture rate down {base['rate_hz']:.0f} -> {run['rate_hz']:.0f} Hz "
                   f"but NIC rate held -- {why}.")

    return out or ["No degradation detected."]


def compare(results):
    base = results.get("baseline")
    if not base or "error" in base or len(results) < 2:
        return

    print("\n" + "=" * 68)
    print("COMPARISON vs baseline")
    print("=" * 68)

    for label, run in results.items():
        if label == "baseline" or "error" in run:
            continue
        print(f"\n  baseline  ->  {label}")
        for k in ROWS:
            b, n = base.get(k), run.get(k)
            if b is None or n is None:
                continue
            delta = "" if not b else f"   ({n / b:.1f}x)"
            print(f"    {k:<18} {b:>10}  ->  {n:>10}{delta}")
        print("\n    verdict: " + "\n             ".join(judge(base, run)))
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label", help="name for this run, e.g. 'baseline' or 'with-vicon'")
    ap.add_argument("secs", nargs="?", type=float, default=60.0)
    ap.add_argument("--iface", default="eth0")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.secs)
    print(f"[{args.label}] sampling /lowstate for {args.secs:.0f}s ...")

    net_before = net_counters(args.iface)
    t_start = time.perf_counter()
    try:
        while node.elapsed() < args.secs:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    wall = time.perf_counter() - t_start
    net_after = net_counters(args.iface)

    summary = summarize(node.stamps, net_before, net_after, wall)
    node.destroy_node()
    rclpy.shutdown()

    print(f"\n--- {args.label} ---")
    for k, v in summary.items():
        print(f"  {k:<18} {v}")

    results = {}
    if RESULTS.exists():
        results = json.loads(RESULTS.read_text())
    results[args.label] = summary
    RESULTS.write_text(json.dumps(results, indent=2))
    print(f"\nsaved -> {RESULTS}")

    compare(results)


if __name__ == "__main__":
    main()
