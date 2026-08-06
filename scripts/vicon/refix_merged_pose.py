#!/usr/bin/env python3
"""Re-solve Vicon pose in an already-merged capture, with the centring fix applied.

Session 1 was merged before the Kabsch centring bug was found (see the fix in
osc_listener.py), and without --max-rms, so its 3-marker frames carried a ~29mm
position artifact straight into the training CSV. Re-capturing is impossible and
re-running the whole pipeline needs the raw rollout CSV, which was not kept -- but
the raw MARKERS were, and that is enough: re-solve pose from them, resample onto
the merged file's own timestamps, and rewrite only the pose-derived columns.

Everything else in the merged file (actions, dof, tau, IMU) is untouched.

Emits a corrected merged CSV; run clean_dataset.py on it as normal.

  python3 refix_merged_pose.py \
      --merged  merged_20260730_060409.csv \
      --markers oscmarkers_20260730_060345.csv \
      --out     merged_20260730_fixed.csv
"""
import argparse
import math

import numpy as np
import pandas as pd

MARKERS = ["PELVLB", "PELVLF", "PELVRB", "PELVRF"]


# ----------------------------------------------------------------- quaternions
def R_to_q(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [.25 * s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s]
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2]) * 2
        q = [(R[2,1]-R[1,2])/s, .25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s]
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2]) * 2
        q = [(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, .25*s, (R[1,2]+R[2,1])/s]
    else:
        s = math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1]) * 2
        q = [(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, .25*s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def q_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def quat_mul(a, b):
    w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def slerp(q0, q1, u):
    q0 = q0/np.linalg.norm(q0); q1 = q1/np.linalg.norm(q1)
    d = float(np.dot(q0, q1))
    if d < 0: q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u*(q1-q0); return q/np.linalg.norm(q)
    th = math.acos(np.clip(d, -1, 1)); s = math.sin(th)
    return (math.sin((1-u)*th)/s)*q0 + (math.sin(u*th)/s)*q1


# ----------------------------------------------------------------- pose solve
def solve_stream(markers_csv, max_rms):
    """Kabsch pose per frame, WITH the reference subset re-centred (the fix)."""
    d = pd.read_csv(markers_csv).drop_duplicates(["frame", "marker"])
    cnt = d.groupby("frame").marker.nunique()
    f0 = cnt[cnt == 4].index[0]
    first = d[d.frame == f0].set_index("marker")[["x", "y", "z"]]
    names = sorted(first.index)
    refc = first.loc[names].to_numpy().mean(0)
    ref = {n: first.loc[n].to_numpy() - refc for n in names}

    T, POS, QUAT, NM, RMS = [], [], [], [], []
    for _, sub in d.groupby("frame"):
        marks = {r.marker: np.array([r.x, r.y, r.z]) for r in sub.itertuples()}
        common = [n for n in names if n in marks]
        if len(common) < 3:
            continue                       # 2 markers cannot fix orientation
        cur = np.array([marks[n] for n in common])
        cc = cur.mean(0)
        P = np.array([ref[n] for n in common])
        # THE FIX: re-centre the reference subset. Without this, P keeps the mean
        # of the FULL 4-marker reference while Q is centred on the 3 visible ones,
        # and Kabsch is handed two clouds about different origins -- worth ~29mm.
        p0 = P.mean(0)
        Pc = P - p0
        Q = cur - cc
        H = Pc.T @ Q
        U, _, Vt = np.linalg.svd(H)
        R = (U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt).T
        rms = np.sqrt((np.linalg.norm((R @ Pc.T).T - Q, axis=1) ** 2).mean()) * 1000
        if max_rms is not None and rms > max_rms:
            continue
        T.append(sub.t_wall.iloc[0])
        POS.append(cc - R @ p0)            # centroid -> body origin
        QUAT.append(R_to_q(R)); NM.append(len(common)); RMS.append(rms)
    o = np.argsort(T)
    return (np.array(T)[o], np.array(POS)[o], np.array(QUAT)[o],
            np.array(NM)[o], np.array(RMS)[o])


def interp_pose(tq, tv, pv, qv):
    n = len(tq)
    pos = np.full((n, 3), np.nan); quat = np.full((n, 4), np.nan)
    cover = np.zeros(n, int)
    j = np.searchsorted(tv, tq)
    for k in range(n):
        jj = j[k]
        if jj <= 0 or jj >= len(tv):
            continue
        t0, t1 = tv[jj-1], tv[jj]
        if t1 <= t0:
            continue
        u = (tq[k]-t0)/(t1-t0)
        pos[k] = pv[jj-1] + u*(pv[jj]-pv[jj-1])
        quat[k] = slerp(qv[jj-1], qv[jj], u)
        cover[k] = 1
    return pos, quat, cover


def derive_vel(t, pos, quat, cover):
    n = len(t)
    lw = np.full((n,3), np.nan); aw = np.full((n,3), np.nan)
    lb = np.full((n,3), np.nan); ab = np.full((n,3), np.nan)
    for k in range(n):
        if cover[k] == 0:
            continue
        km, kp = max(k-1, 0), min(k+1, n-1)
        if cover[km] == 0 or cover[kp] == 0 or t[kp] <= t[km]:
            continue
        h = t[kp]-t[km]
        lw[k] = (pos[kp]-pos[km])/h
        dq = quat_mul(quat_conj(quat[km]), quat[kp])
        dq = dq/(np.linalg.norm(dq) or 1.0)
        ang = 2.0*math.acos(float(np.clip(dq[0], -1, 1)))
        ax = dq[1:]; s = np.linalg.norm(ax)
        wb = (ax/s)*(ang/h) if s > 1e-9 else np.zeros(3)
        R = q_to_R(quat[k])
        aw[k] = R@wb; ab[k] = wb; lb[k] = R.T@lw[k]
    return lw, aw, lb, ab


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--merged", required=True)
    ap.add_argument("--markers", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-rms", type=float, default=5.0,
                    help="drop Vicon frames above this fit residual BEFORE "
                         "interpolating, so pose is interpolated ACROSS a bad "
                         "frame rather than contaminated by it")
    args = ap.parse_args()

    mg = pd.read_csv(args.merged)
    tq = mg.t_wall.to_numpy()
    tv, pv, qv, nm, rms = solve_stream(args.markers, args.max_rms)
    print(f"pose samples kept {len(tv)} (3-marker {int((nm==3).sum())}), "
          f"residual median {np.median(rms):.2f} mm")

    pos, quat, cov = interp_pose(tq, tv, pv, qv)
    lw, aw, lb, ab = derive_vel(tq, pos, quat, cov)

    old = mg[["base_pos_x", "base_pos_y", "base_pos_z"]].to_numpy()
    out = mg.copy()
    out[["base_pos_x","base_pos_y","base_pos_z"]] = pos
    out[["base_quat_w","base_quat_x","base_quat_y","base_quat_z"]] = quat
    out[["base_lin_vel_w_x","base_lin_vel_w_y","base_lin_vel_w_z"]] = lw
    out[["base_ang_vel_w_x","base_ang_vel_w_y","base_ang_vel_w_z"]] = aw
    out[["base_lin_vel_b_x","base_lin_vel_b_y","base_lin_vel_b_z"]] = lb
    out[["base_ang_vel_b_x","base_ang_vel_b_y","base_ang_vel_b_z"]] = ab
    out["vicon_cover"] = cov
    out.to_csv(args.out, index=False)

    d = np.linalg.norm(pos - old, axis=1) * 1000
    d = d[~np.isnan(d)]
    print(f"wrote {args.out}")
    print(f"  rows changed >0.1mm: {int((d>0.1).sum())} of {len(d)}   max {d.max():.2f} mm")
    print(f"  rows without coverage: {int((cov==0).sum())}")


if __name__ == "__main__":
    main()
