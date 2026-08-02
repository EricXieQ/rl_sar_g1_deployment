#!/usr/bin/env python3
"""
live_count.py -- live rep counter for a capture session.

Watches the newest rollout CSV and reports complete dab repetitions as they land,
plus a running total across all sessions and progress toward a target.

    /usr/bin/python3 ~/qxie2/rl_sar/scripts/live_count.py            # target 400
    /usr/bin/python3 ~/qxie2/rl_sar/scripts/live_count.py --target 500

Reads incrementally (remembers the byte offset), so it stays cheap even when the
rollout grows to tens of megabytes. Ctrl+C to stop -- it does not touch the data.
"""
import argparse, csv, glob, math, os, time, sys

LOGS = "/home/unitree/qxie2/rl_sar/logs"
MIN_STEPS = 250          # a full dab is ~298; shorter means it was cut off
TAU_ALIVE = 8.0          # mean |tau| below this = motors were not driving


def count_reps(path, upto=None):
    """Complete + alive asap_dab segments in one rollout file."""
    try:
        rows = list(csv.reader(l.replace("\x00", "") for l in open(path, errors="ignore")))
    except OSError:
        return 0, 0, None
    if len(rows) < 2:
        return 0, 0, None
    h = rows[0]
    d = [r for r in rows[1:] if len(r) == len(h)]
    if not d:
        return 0, 0, None
    I = {c: i for i, c in enumerate(h)}
    tau = [i for c, i in I.items() if c.startswith("tau_est_")]

    segs, cur = [], None
    for k, r in enumerate(d):
        s = r[I["state"]]
        if cur is None or s != cur[0]:
            if cur:
                segs.append(cur)
            cur = [s, k, k]
        cur[2] = k
    if cur:
        segs.append(cur)

    good = 0
    for s, a, b in segs:
        if s != "asap_dab" or (b - a + 1) < MIN_STEPS:
            continue
        m = []
        for r in d[a:b + 1]:
            try:
                m.append(math.sqrt(sum(float(r[i]) ** 2 for i in tau)))
            except ValueError:
                pass
        if m and sum(m) / len(m) > TAU_ALIVE:
            good += 1
    try:
        dur = float(d[-1][I["t_wall"]]) - float(d[0][I["t_wall"]])
    except ValueError:
        dur = 0.0
    return good, len(d), dur


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=int, default=400)
    ap.add_argument("--every", type=float, default=10.0, help="seconds between updates")
    # Sessions before 30 Jul 2026 pre-date the ankle fine-tune and the Nexus subject
    # fix, so they contain the wind-up and the 180-degree marker flips. They are NOT
    # usable training data and must not count toward the target.
    ap.add_argument("--since", default="20260730", help="ignore sessions before this YYYYMMDD")
    args = ap.parse_args()

    # everything except the live file is fixed -- count it once
    allf = sorted(glob.glob(os.path.join(LOGS, "rollout_*.csv")))
    files = [f for f in allf if os.path.basename(f).split("_")[1] >= args.since]
    live = max(allf, key=os.path.getmtime) if allf else None
    if live is None:
        print("no rollout files found"); return
    past = sum(count_reps(f)[0] for f in files if f != live)

    print("live file : %s" % os.path.basename(live))
    print("previous  : %d reps from %d earlier session(s) since %s" % (past, len(files) - 1, args.since))
    print("excluded  : %d pre-fix session(s) (ankle wind-up / marker flips)" % (len(allf) - len(files)))
    print("target    : %d\n" % args.target)
    print("%-9s %7s %7s %8s %9s %s" % ("elapsed", "reps", "TOTAL", "to go", "rate/min", "eta"))
    print("-" * 62)

    t0 = time.time()
    try:
        while True:
            n, steps, dur = count_reps(live)
            total = past + n
            togo = max(0, args.target - total)
            rate = n / (dur / 60.0) if dur and dur > 30 else 0.0
            eta = ("%.0f min" % (togo / rate)) if rate > 0.2 else "--"
            bar_done = int(28 * min(1.0, total / args.target))
            bar = "#" * bar_done + "." * (28 - bar_done)
            sys.stdout.write("\r%-9s %7d %7d %8d %9.1f %-8s [%s]" %
                             ("%.0fs" % (dur or 0), n, total, togo, rate, eta, bar))
            sys.stdout.flush()
            if total >= args.target:
                print("\n\n*** target of %d reached ***" % args.target)
            time.sleep(args.every)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
