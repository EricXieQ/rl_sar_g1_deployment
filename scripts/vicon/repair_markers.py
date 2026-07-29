#!/usr/bin/env python3
"""
repair_markers.py -- automatically un-swap mislabelled mocap markers.

Vicon's labeller confuses markers that are close together (on the G1 pelvis the
left/right pair is only ~75mm apart vs ~170mm front/back), especially when the
arm occludes one during a dab. The result is a frame where labels are permuted,
which breaks the rigid-body fit and flips the computed orientation ~180 deg.

Because the cluster IS rigid, the true inter-marker distances are known. For any
frame whose geometry doesn't match, we try every permutation of the labels and
keep the one that best matches the reference geometry. This is deterministic and
runs in seconds -- no manual Nexus relabelling.

Reference geometry is taken from a clean capture (--ref), or from the most
self-consistent frames of the input itself.

  python3 repair_markers.py --in oscmarkers_X.csv --ref oscmarkers_clean.csv \
                            --out oscmarkers_X_repaired.csv

Outputs the repaired marker CSV plus a report of what was fixed/dropped.
"""
import argparse, csv, itertools, math, os
from collections import defaultdict


def load(path):
    frames = defaultdict(dict)
    meta = {}
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                fr = int(row["frame"])
                frames[fr][row["marker"]] = (float(row["x"]), float(row["y"]), float(row["z"]))
                meta[fr] = (row["t_wall"], row.get("subject", ""))
            except (ValueError, KeyError):
                pass
    return frames, meta


def dist(a, b):
    return math.dist(a, b) * 1000.0     # mm


def ref_geometry(frames, names):
    """Median pairwise distances over the most self-consistent frames."""
    pairs = list(itertools.combinations(names, 2))
    per = {p: [] for p in pairs}
    for fr, m in frames.items():
        if len(m) < len(names):
            continue
        for p in pairs:
            per[p].append(dist(m[p[0]], m[p[1]]))
    out = {}
    for p in pairs:
        v = sorted(per[p])
        if not v:
            return None
        out[p] = v[len(v) // 2]         # median
    return out


def geo_error(assign, ref, names):
    """RMS mm error of an assignment against the reference distances."""
    err = 0.0; n = 0
    for a, b in itertools.combinations(names, 2):
        if a in assign and b in assign:
            err += (dist(assign[a], assign[b]) - ref[(a, b)]) ** 2
            n += 1
    return math.sqrt(err / n) if n else float("inf")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--ref", default=None, help="clean capture to take reference geometry from")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tol", type=float, default=8.0, help="mm tolerance for 'good' geometry")
    ap.add_argument("--max-move", type=float, default=40.0, help="max mm a marker may move between frames")
    args = ap.parse_args()

    frames, meta = load(args.inp)
    all_names = sorted({n for m in frames.values() for n in m})
    print("input: %d frames, markers: %s" % (len(frames), ", ".join(all_names)))

    ref = None
    if args.ref:
        rf, _ = load(args.ref)
        ref = ref_geometry(rf, all_names)
        print("reference geometry from %s" % os.path.basename(args.ref))
    if ref is None:
        ref = ref_geometry(frames, all_names)
        print("reference geometry from the input itself")
    for p, v in sorted(ref.items()):
        print("   %-16s %7.1f mm" % ("-".join(p), v))

    good = repaired = unfixable = partial = 0
    out_rows = []
    prev_frame = None
    for fr in sorted(frames):
        m = frames[fr]
        t_wall, subj = meta[fr]
        present = sorted(m)
        if len(present) < 3:
            partial += 1
            for nm, p in m.items():
                out_rows.append([t_wall, fr, subj, nm, p[0], p[1], p[2], "partial"])
            continue

        e0 = geo_error(m, ref, all_names)
        if e0 <= args.tol and (prev_frame is None or
                max((dist(m[n], prev_frame[n]) for n in m if n in prev_frame), default=0) <= args.max_move):
            good += 1
            for nm, p in m.items():
                out_rows.append([t_wall, fr, subj, nm, p[0], p[1], p[2], "ok"])
            prev_frame = dict(m)
            continue

        # Try every relabelling. Geometry alone is ambiguous for a near-symmetric
        # cluster (a 180-deg-rotated labelling fits just as well), so among the
        # geometrically-valid candidates pick the one whose marker positions stay
        # CLOSEST TO THE PREVIOUS FRAME -- the pelvis cannot teleport in 10 ms.
        pts = [m[n] for n in present]
        cands = []
        for perm in itertools.permutations(present):
            cand = {perm[i]: pts[i] for i in range(len(pts))}
            e = geo_error(cand, ref, all_names)
            if e <= args.tol:
                cands.append((e, cand))
        best, best_e = None, None
        if cands:
            if prev_frame:
                def continuity(c):
                    d = [dist(c[n], prev_frame[n]) for n in c if n in prev_frame]
                    return sum(d) / len(d) if d else 1e9
                best_e, best = min(((e, c) for e, c in cands), key=lambda ec: continuity(ec[1]))
                if continuity(best) > args.max_move:
                    best = None          # even the best candidate jumped too far
            else:
                best_e, best = min(cands, key=lambda ec: ec[0])
        if best is not None:
            repaired += 1
            for nm, p in best.items():
                out_rows.append([t_wall, fr, subj, nm, p[0], p[1], p[2], "repaired"])
            prev_frame = dict(best)
        else:
            unfixable += 1
            for nm, p in m.items():
                out_rows.append([t_wall, fr, subj, nm, p[0], p[1], p[2], "bad"])

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_wall", "frame", "subject", "marker", "x", "y", "z", "status"])
        for r in out_rows:
            w.writerow(r[:4] + ["%.6f" % v for v in r[4:7]] + [r[7]])

    tot = len(frames)
    print("\n--- repair report (tolerance %.1f mm) ---" % args.tol)
    print("  already good : %6d (%5.1f%%)" % (good, 100 * good / tot))
    print("  REPAIRED     : %6d (%5.1f%%)  <- relabelled to match rigid geometry" % (repaired, 100 * repaired / tot))
    print("  unfixable    : %6d (%5.1f%%)  <- geometry wrong even after relabelling" % (unfixable, 100 * unfixable / tot))
    print("  <3 markers   : %6d (%5.1f%%)" % (partial, 100 * partial / tot))
    print("  => usable    : %6d (%5.1f%%)" % (good + repaired, 100 * (good + repaired) / tot))
    print("\nwrote %s" % args.out)


if __name__ == "__main__":
    main()
