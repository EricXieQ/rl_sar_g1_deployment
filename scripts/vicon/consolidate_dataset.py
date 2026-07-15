#!/usr/bin/env python3
"""
consolidate_dataset.py -- build ONE training-ready ASAP delta-action dataset from
all the (rollout, vicon) capture pairs.

For each pair it:
  1. cleans NUL bytes / truncated rows,
  2. merges rollout + vicon on the SHARED Jetson clock (offset 0 -- reliable to
     ~20ms; the stomp cross-correlation was too weak to trust here),
  3. keeps only ALIVE, COMPLETE asap_dab reps (mean |tau_est| > TAU_ALIVE and
     length >= MIN_DAB_STEPS) -- drops loco, dead-robot tails, and cut reps,
  4. tags each with run_id / rep_id / rep_step and concatenates.

Output: consolidated_dab_dataset.csv  (+ printed summary).
Run:  /usr/bin/python3 consolidate_dataset.py
"""
import csv, glob, os, math, subprocess, sys, tempfile

LOGDIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "logs"))
MERGE  = os.path.join(os.path.dirname(__file__), "merge_rollout_vicon.py")
OBJECT = "pelvis_g1"
TAU_ALIVE = 8.0        # mean torque-norm threshold: a rep the motors actually drove
MIN_DAB_STEPS = 250    # a "complete" dab (full reps are ~297-298 steps)
OUT = os.path.join(LOGDIR, "consolidated_dab_dataset.csv")


def clean(src, dst):
    rows = list(csv.reader((l.replace("\x00", "") for l in open(src, errors="ignore"))))
    n = len(rows[0]); good = [r for r in rows if len(r) == n]
    csv.writer(open(dst, "w", newline="")).writerows(good)
    return len(good) - 1


def first_twall(f):
    r = csv.reader((l.replace("\x00", "") for l in open(f, errors="ignore")))
    h = next(r); i = h.index("t_wall")
    for row in r:
        if len(row) == len(h):
            try: return float(row[i])
            except ValueError: pass
    return None


def pair_files():
    rolls = [(f, first_twall(f)) for f in sorted(glob.glob(os.path.join(LOGDIR, "rollout_*.csv")))]
    vics  = [(f, first_twall(f)) for f in sorted(glob.glob(os.path.join(LOGDIR, "vicon_*.csv")))]
    pairs = []
    for rf, rt in rolls:
        best = None
        for vf, vt in vics:
            if vt is None or rt is None: continue
            if vt <= rt + 5 and (best is None or abs(rt - vt) < abs(rt - best[1])):
                best = (vf, vt)
        if best: pairs.append((rf, best[0]))
    return pairs


def main():
    tmp = tempfile.mkdtemp()
    pairs = pair_files()
    out_rows = []
    header = None
    rep_id = 0
    summary = []
    for run_id, (rf, vf) in enumerate(pairs, 1):
        rc, vc = os.path.join(tmp, "r.csv"), os.path.join(tmp, "v.csv")
        clean(rf, rc); clean(vf, vc)
        merged = os.path.join(tmp, "m.csv")
        res = subprocess.run([sys.executable, MERGE, "--rollout", rc, "--vicon", vc,
                              "--object", OBJECT, "--out", merged, "--manual-offset", "0"],
                             capture_output=True, text=True)
        if not os.path.exists(merged):
            summary.append((os.path.basename(rf), 0, 0, "merge failed: " + res.stderr[-120:]))
            continue
        rows = list(csv.reader(open(merged)))
        h = rows[0]; d = rows[1:]
        I = {c: k for k, c in enumerate(h)}
        tau = [k for k, c in enumerate(h) if c.startswith("tau_est_")]
        def tn(r):
            try: return math.sqrt(sum(float(r[k]) ** 2 for k in tau))
            except ValueError: return 0.0
        # segment into contiguous state runs
        segs = []; cur = None
        for k, r in enumerate(d):
            s = r[I["state"]]
            if cur is None or s != cur[0]:
                if cur: segs.append(cur)
                cur = [s, k, k]
            cur[2] = k
        if cur: segs.append(cur)
        reps_this = 0
        for s, a, b in segs:
            if s != "asap_dab" or (b - a + 1) < MIN_DAB_STEPS:
                continue
            seg = d[a:b + 1]
            if sum(tn(r) for r in seg) / len(seg) <= TAU_ALIVE:
                continue                      # dead-robot / not driven
            rep_id += 1; reps_this += 1
            for rs, r in enumerate(seg):
                if header is None:
                    header = ["run_id", "rep_id", "rep_step"] + h
                out_rows.append([run_id, rep_id, rs] + r)
        cov = ""
        for line in res.stdout.splitlines():
            if "coverage" in line: cov = line.split("coverage")[-1].strip()
        summary.append((os.path.basename(rf), reps_this, len(out_rows), cov))

    if header is None:
        print("No alive DAB reps found."); return
    with open(OUT, "w", newline="") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(out_rows)

    print("=== consolidated ASAP DAB dataset ===")
    print("%-32s %5s" % ("run", "reps"))
    for name, reps, _, cov in summary:
        print("  %-30s %3d   (cov %s)" % (name, reps, cov))
    print("\ntotal DAB reps : %d" % rep_id)
    print("total rows     : %d  (~%.0f transitions)" % (len(out_rows), len(out_rows)))
    print("output         : %s" % OUT)
    print("columns        : run_id, rep_id, rep_step + full merged state/action/base pose")


if __name__ == "__main__":
    main()
