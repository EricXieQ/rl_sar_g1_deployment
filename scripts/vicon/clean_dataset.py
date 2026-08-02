#!/usr/bin/env python3
"""
clean_dataset.py -- turn a merged capture into a training-ready ASAP dataset.

Takes merged_*.csv (one row per policy step, state/action + Vicon base pose) and:

  1. keeps only `asap_dab` segments that are COMPLETE (>= --min-steps) and ALIVE
     (mean |tau_est| above threshold, i.e. the motors were actually driving --
     this drops reps recorded after a firmware power-cut),
  2. drops steps without Vicon coverage,
  3. rotates the Vicon base orientation and velocities into the URDF pelvis frame
     using the measured calibration (default = the 2026-07-30 fit),
  4. re-references position so each rep starts at the origin with zero heading
     (optional, --recenter) so reps are comparable regardless of where in the
     capture volume they happened,
  5. tags every row with rep_id / rep_step and writes one clean CSV.

Usage:
  /usr/bin/python3 clean_dataset.py --in merged_X.csv --out dab_clean.csv
  ... --cal-rpy 4.51,5.99,-3.71     # calibration in degrees (Vicon -> URDF)
  ... --no-recenter                 # keep absolute world coordinates
"""
import argparse, csv, math

import numpy as np


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def q_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def R_to_q(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s;                z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q) or 1.0
    return -q if q[0] < 0 else q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--state", default="asap_dab")
    ap.add_argument("--min-steps", type=int, default=250)
    ap.add_argument("--tau-alive", type=float, default=8.0)
    ap.add_argument("--cal-rpy", default="4.51,5.99,-3.71",
                    help="Vicon->URDF calibration, degrees roll,pitch,yaw")
    ap.add_argument("--no-recenter", action="store_true")
    args = ap.parse_args()

    with open(args.inp) as f:
        rows = list(csv.DictReader(f))
    hdr = list(rows[0].keys())
    print("input: %d rows, %d columns" % (len(rows), len(hdr)))

    r, p, y = [math.radians(float(v)) for v in args.cal_rpy.split(",")]
    Rcal = rpy_to_R(r, p, y)
    print("calibration (Vicon->URDF): roll %.2f pitch %.2f yaw %.2f deg"
          % tuple(float(v) for v in args.cal_rpy.split(",")))

    tau_cols = [c for c in hdr if c.startswith("tau_est_")]

    # ---- segment into contiguous runs of the target state -------------------
    segs, cur = [], None
    for i, row in enumerate(rows):
        s = row.get("state", "")
        if cur is None or s != cur[0]:
            if cur:
                segs.append(cur)
            cur = [s, i, i]
        cur[2] = i
    if cur:
        segs.append(cur)

    kept, dropped_short, dropped_dead = [], 0, 0
    for s, a, b in segs:
        if s != args.state:
            continue
        n = b - a + 1
        if n < args.min_steps:
            dropped_short += 1
            continue
        tau = []
        for row in rows[a:b + 1]:
            try:
                tau.append(math.sqrt(sum(float(row[c]) ** 2 for c in tau_cols)))
            except ValueError:
                pass
        if not tau or (sum(tau) / len(tau)) <= args.tau_alive:
            dropped_dead += 1
            continue
        kept.append((a, b))

    print("segments of '%s': kept %d | dropped %d short | dropped %d dead"
          % (args.state, len(kept), dropped_short, dropped_dead))

    extra = ["rep_id", "rep_step",
             "base_quat_urdf_w", "base_quat_urdf_x", "base_quat_urdf_y", "base_quat_urdf_z",
             "base_lin_vel_urdf_b_x", "base_lin_vel_urdf_b_y", "base_lin_vel_urdf_b_z",
             "base_ang_vel_urdf_b_x", "base_ang_vel_urdf_b_y", "base_ang_vel_urdf_b_z",
             "base_pos_rel_x", "base_pos_rel_y", "base_pos_rel_z"]
    out_rows, no_cov = [], 0

    for rep, (a, b) in enumerate(kept, 1):
        seg = rows[a:b + 1]
        seg = [r_ for r_ in seg if r_.get("vicon_cover") == "1"]
        no_cov += (b - a + 1) - len(seg)
        if not seg:
            continue

        # reference pose of this rep (for optional re-centering)
        try:
            p0 = np.array([float(seg[0]["base_pos_x"]), float(seg[0]["base_pos_y"]),
                           float(seg[0]["base_pos_z"])])
            q0 = [float(seg[0]["base_quat_w"]), float(seg[0]["base_quat_x"]),
                  float(seg[0]["base_quat_y"]), float(seg[0]["base_quat_z"])]
            R0 = Rcal @ q_to_R(q0)
            yaw0 = math.atan2(R0[1, 0], R0[0, 0])
            Ryaw = rpy_to_R(0, 0, -yaw0)
        except (ValueError, KeyError):
            continue

        for k, row in enumerate(seg):
            try:
                qv = [float(row["base_quat_w"]), float(row["base_quat_x"]),
                      float(row["base_quat_y"]), float(row["base_quat_z"])]
                pos = np.array([float(row["base_pos_x"]), float(row["base_pos_y"]),
                                float(row["base_pos_z"])])
                lw = np.array([float(row["base_lin_vel_w_x"]), float(row["base_lin_vel_w_y"]),
                               float(row["base_lin_vel_w_z"])])
                aw = np.array([float(row["base_ang_vel_w_x"]), float(row["base_ang_vel_w_y"]),
                               float(row["base_ang_vel_w_z"])])
            except (ValueError, KeyError):
                continue

            # Vicon body frame -> URDF pelvis frame.
            # Rcal (= C) is the fitted mean of R_imu^T R_vicon, so it maps a vector
            # FROM the Vicon body frame INTO the URDF body frame, which means the
            # orientation converts by RIGHT-multiplying its transpose:
            #     R_urdf = R_vicon @ C^T
            # (verified empirically: this form gives 2.6 deg median agreement with
            # the IMU, vs 8.2 deg uncorrected and 16.5 deg for C @ R_vicon.)
            Rq = q_to_R(qv) @ Rcal.T
            q_urdf = R_to_q(Rq)
            # Body-frame velocities are derived from the corrected orientation rather
            # than rotated by Rcal -- Rcal relates the two BODY frames, not the world
            # frames, so applying it to world-frame vectors would be wrong.
            lv = Rq.T @ lw
            av = Rq.T @ aw

            rel = pos - p0
            if not args.no_recenter:
                rel = Ryaw @ rel
                q_urdf = R_to_q(Ryaw @ Rq)
                lv = Ryaw @ lv
                av = Ryaw @ av

            new = dict(row)
            new.update({
                "rep_id": rep, "rep_step": k,
                "base_quat_urdf_w": "%.6f" % q_urdf[0], "base_quat_urdf_x": "%.6f" % q_urdf[1],
                "base_quat_urdf_y": "%.6f" % q_urdf[2], "base_quat_urdf_z": "%.6f" % q_urdf[3],
                "base_lin_vel_urdf_b_x": "%.6f" % lv[0], "base_lin_vel_urdf_b_y": "%.6f" % lv[1],
                "base_lin_vel_urdf_b_z": "%.6f" % lv[2],
                "base_ang_vel_urdf_b_x": "%.6f" % av[0], "base_ang_vel_urdf_b_y": "%.6f" % av[1],
                "base_ang_vel_urdf_b_z": "%.6f" % av[2],
                "base_pos_rel_x": "%.6f" % rel[0], "base_pos_rel_y": "%.6f" % rel[1],
                "base_pos_rel_z": "%.6f" % rel[2],
            })
            out_rows.append(new)

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=hdr + extra)
        w.writeheader()
        w.writerows(out_rows)

    reps = len({r_["rep_id"] for r_ in out_rows})
    print("dropped %d rows without Vicon coverage" % no_cov)
    print("re-centering: %s" % ("OFF (absolute world coords)" if args.no_recenter
                                else "ON (each rep starts at origin, zero heading)"))
    print("\nwrote %s" % args.out)
    print("  %d reps, %d transitions, %d columns" % (reps, len(out_rows), len(hdr) + len(extra)))


if __name__ == "__main__":
    main()
