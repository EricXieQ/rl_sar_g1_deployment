#!/usr/bin/env python3
"""
merge_rollout_vicon.py  --  align an rl_sar rollout CSV with a Vicon CSV and
emit one merged, per-policy-step trajectory for ASAP delta-model training.

Run OFFLINE (needs numpy):  /usr/bin/python3 merge_rollout_vicon.py ...

INPUTS
  --rollout logs/rollout_YYYYmmdd_HHMMSS.csv   (from rl_sar, RL_RECORD=1)
  --vicon   logs/vicon_YYYYmmdd_HHMMSS.csv     (from vicon_listener.py)

WHAT IT DOES
  1. Both files carry a `t_wall` in the SAME Jetson clock (the listener runs on
     the robot). That aligns them to within network/pipeline latency (a few ms).
  2. To remove that residual, it cross-correlates a shared physical event:
       rollout  -> |IMU linear accel| (lin_acc_0..2), high-passed
       vicon    -> |2nd derivative of pelvis position|, high-passed
     The lag at the correlation peak is the offset applied to the Vicon clock.
     If the rollout has `sync_mark==1` rows (key '.' in rl_sar, pressed at the
     stomp), the search is restricted to a window around them for robustness.
  3. It resamples the (shifted) Vicon pose onto every rollout step: position by
     linear interp, orientation by slerp; then differentiates for base linear &
     angular velocity, in BOTH world frame and base(pelvis) frame.

OUTPUT  (--out merged.csv): every rollout column, plus
  base_pos_{x,y,z}         pelvis position, world frame (m)
  base_quat_{w,x,y,z}      pelvis orientation from mocap, world frame
  base_lin_vel_w_{x,y,z}   world-frame linear velocity (m/s)
  base_ang_vel_w_{x,y,z}   world-frame angular velocity (rad/s)
  base_lin_vel_b_{x,y,z}   base-frame linear velocity  (ASAP obs convention)
  base_ang_vel_b_{x,y,z}   base-frame angular velocity
  vicon_cover              1 if the step fell inside Vicon coverage, else 0
Rows outside Vicon coverage get NaN pose and vicon_cover=0.
"""
import argparse
import csv
import sys
import numpy as np


# ----------------------------------------------------------------------- io
def load_csv(path):
    with open(path, "r", newline="") as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r if row]
    return header, rows


def col(header, rows, name, cast=float):
    i = header.index(name)
    return np.array([cast(row[i]) for row in rows])


def cols_prefixed(header, rows, prefix):
    idx = [k for k, h in enumerate(header) if h.startswith(prefix)]
    idx.sort(key=lambda k: int(header[k].split("_")[-1]))
    out = np.array([[float(row[k]) for k in idx] for row in rows])
    return out


# ------------------------------------------------------------------ quaternion
def quat_normalize(q):
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def slerp(q0, q1, u):
    """q0,q1: (4,) wxyz; u in [0,1]. Shortest-path slerp."""
    q0 = q0 / (np.linalg.norm(q0) or 1.0)
    q1 = q1 / (np.linalg.norm(q1) or 1.0)
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1 = -q1
        d = -d
    if d > 0.9995:                       # nearly parallel -> lerp
        q = q0 + u * (q1 - q0)
        return q / (np.linalg.norm(q) or 1.0)
    th0 = np.arccos(d)
    s = np.sin(th0)
    return (np.sin((1 - u) * th0) / s) * q0 + (np.sin(u * th0) / s) * q1


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)]])


# -------------------------------------------------------------- interpolation
def interp_pose(t_q, t_v, pos_v, quat_v):
    """Interpolate Vicon pose onto query times t_q. Returns pos, quat, cover."""
    n = len(t_q)
    pos = np.full((n, 3), np.nan)
    quat = np.full((n, 4), np.nan)
    cover = np.zeros(n, dtype=int)
    if len(t_v) < 2:
        return pos, quat, cover
    j = np.searchsorted(t_v, t_q)
    for k in range(n):
        jj = j[k]
        if jj <= 0 or jj >= len(t_v):
            continue                       # outside coverage
        t0, t1 = t_v[jj - 1], t_v[jj]
        if t1 <= t0:
            continue
        u = (t_q[k] - t0) / (t1 - t0)
        pos[k] = pos_v[jj - 1] + u * (pos_v[jj] - pos_v[jj - 1])
        quat[k] = slerp(quat_v[jj - 1], quat_v[jj], u)
        cover[k] = 1
    return pos, quat, cover


# --------------------------------------------------------------- correlation
def resample_uniform(t, y, t0, t1, fs):
    grid = np.arange(t0, t1, 1.0 / fs)
    if len(grid) < 4:
        return grid, np.zeros_like(grid)
    return grid, np.interp(grid, t, y)


def highpass(y, fs, fc=1.0):
    """Cheap high-pass: subtract a moving average (window ~1/fc)."""
    w = max(1, int(fs / fc))
    if w >= len(y):
        return y - y.mean()
    ker = np.ones(w) / w
    base = np.convolve(y, ker, mode="same")
    return y - base


def estimate_offset(t_roll, acc_roll, t_vic, acc_vic, fs=200.0,
                    win=None, win_center=None):
    """Return dt_seconds to ADD to the Vicon clock so it aligns with rollout,
    plus a normalized peak score in [0,1]."""
    t0 = max(t_roll[0], t_vic[0])
    t1 = min(t_roll[-1], t_vic[-1])
    if win is not None and win_center is not None:
        t0 = max(t0, win_center - win)
        t1 = min(t1, win_center + win)
    if t1 - t0 < 0.5:
        return 0.0, 0.0
    g, a = resample_uniform(t_roll, acc_roll, t0, t1, fs)
    _, b = resample_uniform(t_vic, acc_vic, t0, t1, fs)
    a = highpass(a, fs)
    b = highpass(b, fs)
    a = (a - a.mean())
    b = (b - b.mean())
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0, 0.0
    corr = np.correlate(a, b, mode="full") / (na * nb)
    lags = np.arange(-(len(b) - 1), len(a))
    k = int(np.argmax(corr))
    lag_samples = float(lags[k])
    # parabolic interpolation around the peak for sub-sample (sub-grid) lag
    if 0 < k < len(corr) - 1:
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        denom = (y0 - 2 * y1 + y2)
        if abs(denom) > 1e-12:
            lag_samples += 0.5 * (y0 - y2) / denom
    # corr peaks when a[n] ~ b[n - lag]; the Vicon event is `lag` samples LATER
    # than the rollout event -> advance the Vicon clock by dt (negative).
    dt = lag_samples / fs
    return dt, float(corr[k])


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rollout", required=True)
    ap.add_argument("--vicon", required=True)
    ap.add_argument("--object", default=None, help="Vicon object name to use")
    ap.add_argument("--out", default="merged.csv")
    ap.add_argument("--fs", type=float, default=200.0, help="correlation resample rate (Hz)")
    ap.add_argument("--win", type=float, default=3.0,
                    help="+/- seconds around sync_mark to search (if marks present)")
    ap.add_argument("--manual-offset", type=float, default=None,
                    help="skip cross-correlation; add this many seconds to Vicon clock")
    # Quality floor. The rigid-fit residual (rms_mm, written by osc_listener.py) says
    # how well the markers still formed a rigid body that frame. The distribution is
    # bimodal -- the bulk sits below ~5mm and a small tail jumps to ~30mm. Dropping the
    # tail BEFORE interpolation means the pose is interpolated across a bad frame
    # instead of being contaminated by it.
    ap.add_argument("--max-rms", type=float, default=None,
                    help="drop Vicon frames whose rms_mm exceeds this (mm)")
    args = ap.parse_args()

    rh, rr = load_csv(args.rollout)
    vh, vr = load_csv(args.vicon)

    t_roll = col(rh, rr, "t_wall")
    lin_acc = cols_prefixed(rh, rr, "lin_acc_")
    acc_roll = np.linalg.norm(lin_acc, axis=1)
    sync_mark = col(rh, rr, "sync_mark", int) if "sync_mark" in rh else np.zeros(len(rr), int)

    # pick the Vicon object
    obj = col(vh, vr, "object", str)
    names, counts = np.unique(obj, return_counts=True)
    target = args.object or names[int(np.argmax(counts))]
    sel = obj == target
    if sel.sum() < 2:
        sys.exit("ERROR: Vicon object '%s' not found. Available: %s"
                 % (target, ", ".join(names)))
    print("[merge] using Vicon object '%s' (%d samples)" % (target, sel.sum()))

    if args.max_rms is not None and "rms_mm" in vh:
        rms = col(vh, vr, "rms_mm")
        keep = (rms <= args.max_rms)
        dropped = int((~keep & sel).sum())
        sel = sel & keep
        print("[merge] rms filter <= %.1f mm: dropped %d frames (%.2f%%)"
              % (args.max_rms, dropped, 100.0 * dropped / max(1, len(rms))))

    t_vic = col(vh, vr, "t_wall")[sel]
    pos_v = np.stack([col(vh, vr, "x")[sel], col(vh, vr, "y")[sel],
                      col(vh, vr, "z")[sel]], axis=1)
    quat_v = quat_normalize(np.stack([col(vh, vr, "qw")[sel], col(vh, vr, "qx")[sel],
                                      col(vh, vr, "qy")[sel], col(vh, vr, "qz")[sel]], axis=1))
    order = np.argsort(t_vic)
    t_vic, pos_v, quat_v = t_vic[order], pos_v[order], quat_v[order]

    # Vicon acceleration magnitude (finite diff x2) for correlation
    dt_v = np.gradient(t_vic)
    dt_v[dt_v == 0] = 1e-6
    vel_v = np.gradient(pos_v, axis=0) / dt_v[:, None]
    acc_v = np.linalg.norm(np.gradient(vel_v, axis=0) / dt_v[:, None], axis=1)

    # --- time offset ------------------------------------------------------
    if args.manual_offset is not None:
        dt, score = args.manual_offset, float("nan")
        print("[merge] manual offset = %+.4f s (correlation skipped)" % dt)
    else:
        win_center = None
        if sync_mark.any():
            win_center = float(t_roll[sync_mark == 1][0])
            print("[merge] sync_mark found at t_wall=%.3f -> searching +/-%.1fs"
                  % (win_center, args.win))
        dt, score = estimate_offset(t_roll, acc_roll, t_vic, acc_v, fs=args.fs,
                                    win=args.win if win_center else None,
                                    win_center=win_center)
        print("[merge] estimated Vicon offset = %+.4f s  (corr peak = %.3f)" % (dt, score))
        if score < 0.2:
            print("[merge] WARNING: weak correlation (%.3f). Check that a sharp "
                  "shared motion (stomp) exists, or pass --manual-offset." % score)

    t_vic_aligned = t_vic + dt

    # --- resample pose onto rollout steps --------------------------------
    pos, quat, cover = interp_pose(t_roll, t_vic_aligned, pos_v, quat_v)

    # base velocities via finite diff over rollout steps
    n = len(t_roll)
    lin_w = np.full((n, 3), np.nan)
    ang_w = np.full((n, 3), np.nan)
    lin_b = np.full((n, 3), np.nan)
    ang_b = np.full((n, 3), np.nan)
    for k in range(n):
        if cover[k] == 0:
            continue
        km, kp = max(k - 1, 0), min(k + 1, n - 1)
        if cover[km] == 0 or cover[kp] == 0 or t_roll[kp] <= t_roll[km]:
            continue
        h = t_roll[kp] - t_roll[km]
        lin_w[k] = (pos[kp] - pos[km]) / h
        # angular velocity from relative rotation q_km^-1 * q_kp
        dq = quat_mul(quat_conj(quat[km]), quat[kp])
        dq = dq / (np.linalg.norm(dq) or 1.0)
        angle = 2.0 * np.arccos(np.clip(dq[0], -1.0, 1.0))
        axis = dq[1:]
        s = np.linalg.norm(axis)
        omega_body = (axis / s) * (angle / h) if s > 1e-9 else np.zeros(3)
        R = quat_to_R(quat[k])
        ang_w[k] = R @ omega_body          # world-frame angular velocity
        ang_b[k] = omega_body              # base-frame angular velocity
        lin_b[k] = R.T @ lin_w[k]          # base-frame linear velocity

    # --- write merged CSV -------------------------------------------------
    extra = (["base_pos_x", "base_pos_y", "base_pos_z",
              "base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z",
              "base_lin_vel_w_x", "base_lin_vel_w_y", "base_lin_vel_w_z",
              "base_ang_vel_w_x", "base_ang_vel_w_y", "base_ang_vel_w_z",
              "base_lin_vel_b_x", "base_lin_vel_b_y", "base_lin_vel_b_z",
              "base_ang_vel_b_x", "base_ang_vel_b_y", "base_ang_vel_b_z",
              "vicon_cover"])
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(rh + extra)
        for k in range(n):
            block = list(pos[k]) + list(quat[k]) + list(lin_w[k]) + list(ang_w[k]) \
                + list(lin_b[k]) + list(ang_b[k]) + [cover[k]]
            w.writerow(rr[k] + ["" if (isinstance(v, float) and np.isnan(v)) else
                                ("%.6f" % v if isinstance(v, float) else v) for v in block])

    cov = 100.0 * cover.mean()
    print("[merge] wrote %s  (%d rows, Vicon coverage %.1f%%)" % (args.out, n, cov))
    if cov < 90.0:
        print("[merge] NOTE: <90%% coverage -- rollout extends beyond Vicon capture "
              "or objects were occluded. Uncovered rows have empty pose fields.")


if __name__ == "__main__":
    main()
