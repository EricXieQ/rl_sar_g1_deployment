#!/usr/bin/env python3
"""Score a G1 dab session, robot logs and MuJoCo logs alike, on the simulator's bar.

What this is for
----------------
The stage (c) repair (ASAP docs/stagec_repair_2026-09.md and
docs/stagec_lean_fix_2026-09.md) judged dab policies in MuJoCo on five things:
feet planted, endpoint heading, minimum height, peak torso roll and lateral
shift in the first 2.2 s, plus the ankle push and falls. This script computes
the same quantities, with the same windows and thresholds, from the logs the
real robot writes, and optionally from the MuJoCo harness's trial folders, and
prints one table so that simulator and robot numbers sit side by side.

It needs only numpy and pandas.

Input files
-----------
Give it any mix of these; each is recognised by its header or contents.

1. The dab diagnostic CSV written by RLFSMStateRLASAPDab (fsm_g1.hpp):
   /tmp/rl_sar_dab_log.csv on the robot, one file per repetition, overwritten
   on the next one. Columns: time_ms (policy time), wallclock_ms, tag
   (pre_<state>, loco_last, asap), q0..q28 (measured joint position, SDK
   order), tgt0..tgt28 (PD target), kp0..kp28. It carries no IMU, so it yields
   only the ankle push and whether the dab ran to its end. The policy is not
   in the file: pass --policy, or put the policy name in the path
   (logs/hw/asap_dab_v6recipe_s29/rep03.csv works).

2. The rollout recorder CSV written by RL_Real::RecordRollout (rl_real_g1.cpp,
   enabled with RL_RECORD=1): rl_sar/logs/rollout_<date>_<time>.csv, one file
   per process, every policy tick of every state. Columns: t_mono, t_wall,
   step, state (the config name, which is the policy), episode_time, sync_mark,
   action_*, dof_pos_*, dof_vel_*, base_quat_0..3 (IMU, w x y z), ang_vel_*,
   lin_acc_*, target_dof_pos_*, tau_est_*. Repetitions are the runs of
   consecutive rows whose state is a dab policy. The merged and cleaned files
   of the capture releases (rep_id, base_pos_x/y/z and base_quat_w/x/y/z from
   Vicon) are the same format with extra columns and are accepted too.

3. MuJoCo harness trial folders (--sim DIR ...): each holds events.jsonl,
   state.csv (the observer: pelvis pose, foot clearance and contact forces
   every 10 ms) and dab_log.csv (the diagnostic above). These are the folders
   under data/asap-stagec-repair-g1/logs/ and data/asap-lean-fix-k1/logs/.

What the robot can and cannot supply
------------------------------------
Pelvis roll, pitch and yaw come from the IMU quaternion in the rollout CSV
(the G1's low-state IMU sits in the pelvis, the same body the simulator
scores). Endpoint heading and peak roll are therefore direct. Height and
lateral shift need a position source: they are computed when the file carries
Vicon columns (base_pos_x/y/z with base_quat_w/x/y/z, as the merged capture
files do; the calibrated base_quat_urdf_* of the cleaned files is preferred
for the start heading when present) and printed as n/a otherwise. Note that the Vicon height is the
height of the marker rigid body's origin, not MuJoCo's base_z, so compare
heights between policies within one session rather than against the
simulator's 0.74 m line.

The G1 has no foot force sensors, so "feet planted" is inferred. The primary
inference is kinematic: the height of the lowest corner of each foot below
the pelvis, from the leg forward kinematics (link offsets and foot contact
spheres of g1_29dof.xml, checked against the MuJoCo observer to 0.5 mm)
rotated into the gravity frame by the IMU. With both feet on a flat floor the
two heights are equal; a foot that lifts shows as a height difference, and a
foot that only rocks onto its heel or toe edge does not. A lift event is a
difference of at least LIFT_CM (1.0 cm) for at least 80 ms after the first
0.5 s, the simulator's support-loss duration; the largest difference after
0.5 s is printed as "max foot rise" for the finer comparison between policies.
Limits: it cannot see a foot that unloads without leaving the floor (the
simulator's support loss counts those), it cannot see a foot that slides,
IMU pitch error and floor or foot compliance shift it by a few millimetres.
On the 120 MuJoCo trials of the repair and reset-fix labs the rule flagged
none of the simulator's 50 borderline support losses (peak clearance 0.7 to
2.1 cm, under 1 cm for most of each event) and none of the 230 planted feet,
so it is a detector of clear lifts and steps, not of the simulator's
sub-centimetre unloadings; those show up, if at all, in the max foot rise
column (median 0.06 cm for planted feet in MuJoCo, 1.3 cm for feet that lost
support).

tau_est on the ankles is reported as a second, weaker proxy: an ankle-quiet
event is both ankle pitch and roll torque estimates below QUIET_NM (0.5 N m)
for at least 80 ms after 0.5 s. It is weak because a planted ankle carries
zero torque whenever the centre of pressure passes under the joint: in the
404 recorded v6 repetitions of 2 August 2026 (release capture-20260802),
whose feet stayed planted, the left ankle was quiet for 80 ms in 185
repetitions and the right in 47. Knee torque is near zero through the dab
hold (knees straight) and says nothing about load. So read the ankle-quiet
column as context, not as a feet-planted score, and expect v6 to show many.

For reference, v6 on the robot in those two captures (69 and 404
repetitions, scored by this script): feet planted 69/69 and 402/404, max
foot rise median 0.2 and 0.3 cm, absolute heading median 2.1 and 2.5
degrees (one turn over 15 in 473), peak roll median 1.7 and 1.6 degrees,
lateral median 1.5 and 2.3 cm, Vicon height median 0.68 m, ankle push
+17.6 / +19.1 and +13.5 / +13.3 degrees left / right (the robot's ankles lag
their command far more than MuJoCo's, whose v6 push is +1 to +5 degrees).

Falls: the FSM aborts the dab when IMU roll or pitch exceeds 30 degrees; the
script applies the same rule to the rollout rows and also flags a repetition
whose dab ended before 5.9 s of policy time (aborted, by the FSM or by the
operator).

The quantities (same windows as tools/summarize.py and torso_roll.py)
-------------------------------------------------------------------
heading      pelvis yaw at the end of the dab minus at its start, degrees,
             positive counterclockwise from above; a turn is more than 15.
peak roll    largest change of pelvis roll from the dab start over policy
             time 0 to 2.2 s, degrees, positive when the right side drops.
lateral      largest pelvis displacement across the start heading over 0 to
             2.2 s, cm, positive to the robot's left (n/a without Vicon).
height       minimum pelvis height over the dab, m (n/a without Vicon).
push L / R   commanded minus measured ankle pitch averaged over the first arm
             raise, 0.88 to 1.72 s, degrees, positive toes down, and the same
             times the deployed ankle gain of 20 N m/rad.
lift L / R   foot-height-difference lift events (robot) or support loss of
             80 ms or more after 0.5 s (simulator), per side.
max foot rise largest lowest-corner height difference between the feet
             after 0.5 s, cm (kinematic, both robot and simulator).
quiet L / R  ankle-quiet events from tau_est (robot only).
fall         IMU roll or pitch over 30 degrees, or the harness's fall alarm;
             "aborted" when the dab ended early without one.

The simulator bar (docs/stagec_lean_fix_2026-09.md), per policy: feet planted
in at least 9 of 10 starts; no turn over 15 degrees and median absolute
heading within 5 degrees; median minimum height at least 0.74 m; median
absolute peak roll at most 4.5 degrees; median absolute peak lateral shift at
most 4.5 cm. On the robot the height line does not transfer (see above); use
v6's own session numbers as the reference for every column.

Usage
-----
  # robot session: rollout CSVs (policy from the state column) and per-rep
  # diagnostic CSVs (policy from the folder name)
  python3 scripts/score_dab_session.py logs/rollout_*.csv logs/hw_2026-09/*/rep*.csv

  # the same with the MuJoCo trials of the repair lab beside them
  python3 scripts/score_dab_session.py logs/rollout_*.csv \\
      --sim /home/eric/Project/humanoid/firstmate/data/asap-stagec-repair-g1/logs/s29_t*

  # only simulator trials, with a per-repetition CSV written out
  python3 scripts/score_dab_session.py --sim data/asap-lean-fix-k1/logs/s17_t* --csv per_rep.csv

  --policy NAME   policy label for diagnostic CSVs whose path does not name it
  (simulator trial labels v6, s29_R1 and s17_R1 are printed under the
  policy/g1 folder names asap_dab, asap_dab_v6recipe_s29 and _s17)
  --lift-cm X     lift threshold for the kinematic proxy (default 1.0)
  --quiet-nm X    ankle-quiet threshold (default 0.5)
"""
import argparse
import json
import os
import re
import sys

import numpy as np
import pandas as pd

# ----------------------------------------------------------------- constants
WIN_LEAN = (0.0, 2.2)      # policy time window for peak roll and lateral shift
WIN_RAISE = (0.88, 1.72)   # the first arm raise, window for the ankle push
POST_ENTRY = 0.5           # support loss and lifts are counted after this
MIN_EVENT_S = 0.0799       # 80 ms, the simulator's support-loss duration
DAB_END_S = 5.9            # a dab that ends before this was aborted
TURN_DEG = 15.0
FALL_DEG = 30.0            # the FSM's abort threshold on roll and pitch
ANKLE_KP = 20.0            # deployed ankle gain, N m/rad
LIFT_CM = 1.0
QUIET_NM = 0.5

# The laboratories' MuJoCo trial names for the policies shipped in policy/g1,
# so that simulator and robot rows of one policy share a label.
ALIASES = {"v6": "asap_dab", "asap_dab": "asap_dab",
           "s29_R1": "asap_dab_v6recipe_s29", "s29": "asap_dab_v6recipe_s29",
           "s17_R1": "asap_dab_v6recipe_s17", "s17": "asap_dab_v6recipe_s17"}

L_HIP_P, L_HIP_R, L_HIP_Y, L_KNEE, L_ANK_P, L_ANK_R = 0, 1, 2, 3, 4, 5
R_HIP_P, R_HIP_R, R_HIP_Y, R_KNEE, R_ANK_P, R_ANK_R = 6, 7, 8, 9, 10, 11

# Leg chain of g1_29dof.xml (rl_sar_zoo/g1_description/mjcf), pelvis to the sole.
# Each entry: (body offset in the parent frame, fixed rotation about y in rad,
# joint axis). The hip roll and knee bodies carry a -10 / +10 degree tilt.
TILT = 2.0 * np.arcsin(0.0873386)
LEG_CHAIN = [
    ((0.0, 0.064452, -0.1027), 0.0, "y"),          # hip pitch
    ((0.0, 0.052, -0.030465), -TILT, "x"),         # hip roll
    ((0.025001, 0.0, -0.12412), 0.0, "z"),         # hip yaw
    ((-0.078273, 0.0021489, -0.17734), TILT, "y"), # knee
    ((0.0, -9.4445e-05, -0.30001), 0.0, "y"),      # ankle pitch
    ((0.0, 0.0, -0.017558), 0.0, "x"),             # ankle roll
]
# The four foot contact spheres of the ankle roll body (radius 5 mm): their
# lowest points are the foot's corners.
FOOT_CORNERS = np.array([[-0.05, 0.025, -0.035], [-0.05, -0.025, -0.035],
                         [0.12, 0.03, -0.035], [0.12, -0.03, -0.035]])


# ------------------------------------------------------------- small helpers
def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_to_mat(w, x, y, z):
    """Rotation matrices (N, 3, 3) from unit quaternions given as arrays."""
    m = np.empty((len(w), 3, 3))
    m[:, 0, 0] = 1 - 2 * (y * y + z * z); m[:, 0, 1] = 2 * (x * y - z * w); m[:, 0, 2] = 2 * (x * z + y * w)
    m[:, 1, 0] = 2 * (x * y + z * w); m[:, 1, 1] = 1 - 2 * (x * x + z * z); m[:, 1, 2] = 2 * (y * z - x * w)
    m[:, 2, 0] = 2 * (x * z - y * w); m[:, 2, 1] = 2 * (y * z + x * w); m[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return m


def roll_from_wxyz(w, x, y, z):
    return np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))


def pitch_from_wxyz(w, x, y, z):
    return np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))


def yaw_from_wxyz(w, x, y, z):
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def foot_in_pelvis(q, side, points=None):
    """Points of one foot in the pelvis frame, from the six leg joints.

    q: (N, 29) joint positions in SDK order; points: (P, 3) in the ankle roll
    frame (default: the ankle roll origin). Returns (N, P, 3)."""
    if points is None:
        points = np.zeros((1, 3))
    base = L_HIP_P if side == "L" else R_HIP_P
    n = len(q)
    p = np.zeros((n, 3))
    R = np.tile(np.eye(3), (n, 1, 1))
    for k, (off, tilt, axis) in enumerate(LEG_CHAIN):
        off = np.array(off)
        if side == "R":
            off = off * np.array([1, -1, 1])
        p = p + np.einsum("nij,j->ni", R, off)
        if tilt:
            R = R @ rot("y", tilt)
        ang = q[:, base + k]
        c, s = np.cos(ang), np.sin(ang)
        J = np.tile(np.eye(3), (n, 1, 1))
        if axis == "x":
            J[:, 1, 1] = c; J[:, 1, 2] = -s; J[:, 2, 1] = s; J[:, 2, 2] = c
        elif axis == "y":
            J[:, 0, 0] = c; J[:, 0, 2] = s; J[:, 2, 0] = -s; J[:, 2, 2] = c
        else:
            J[:, 0, 0] = c; J[:, 0, 1] = -s; J[:, 1, 0] = s; J[:, 1, 1] = c
        R = R @ J
    return p[:, None, :] + np.einsum("nij,pj->npi", R, points)


def foot_heights_world(q, quat_wxyz):
    """Height of the lowest corner of each foot relative to the pelvis, in the
    gravity frame given by the pelvis quaternion. Returns (zL, zR), metres,
    negative below the pelvis. Using the lowest corner rather than the ankle
    makes a foot that rocks on its heel or toe edge read as still down."""
    Rp = quat_to_mat(*quat_wxyz)
    zl = np.einsum("nij,npj->npi", Rp, foot_in_pelvis(q, "L", FOOT_CORNERS))[:, :, 2].min(1)
    zr = np.einsum("nij,npj->npi", Rp, foot_in_pelvis(q, "R", FOOT_CORNERS))[:, :, 2].min(1)
    return zl, zr


def runs_of(mask, t):
    """[(t_start, t_end)] of the True runs of mask, using the sample times t."""
    out = []
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return out
    for g in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
        out.append((float(t[g[0]]), float(t[g[-1]])))
    return out


def longest_run(mask, t, dt):
    """Longest True run of mask in seconds, counting each sample as dt."""
    return max([b - a + dt for a, b in runs_of(mask, t)] or [0.0])


def peak_signed(x):
    """The element of x with the largest magnitude, keeping its sign."""
    x = np.asarray(x, float)
    if len(x) == 0 or np.all(np.isnan(x)):
        return np.nan
    i = np.nanargmax(np.abs(x))
    return float(x[i])


def lateral_cm(x, y, yaw0, i0):
    """Pelvis displacement across the start heading, cm, positive to the left."""
    dx, dy = x - x[i0], y - y[i0]
    return (-np.sin(yaw0) * dx + np.cos(yaw0) * dy) * 100.0


def push_over_raise(t, tgt, q):
    """Commanded minus measured ankle pitch over the first raise, degrees L, R."""
    m = (t >= WIN_RAISE[0]) & (t <= WIN_RAISE[1])
    if not m.any():
        return np.nan, np.nan
    pl = np.degrees((tgt[m, L_ANK_P] - q[m, L_ANK_P]).mean())
    pr = np.degrees((tgt[m, R_ANK_P] - q[m, R_ANK_P]).mean())
    return float(pl), float(pr)


def push_nm(deg):
    """Ankle push as a torque at the deployed gain, N m."""
    return np.nan if deg is None or np.isnan(deg) else float(np.radians(deg) * ANKLE_KP)


def policy_from_path(path):
    """A dab policy name found in the path, longest match first."""
    parts = re.split(r"[/\\_.\-]", path)
    cands = re.findall(r"asap_dab(?:_[A-Za-z0-9]+)*", path.replace("-", "_"))
    if cands:
        return max(cands, key=len)
    for p in parts:
        if re.fullmatch(r"(v6|s\d+(?:_R\d+)?)", p):
            return p
    return None


def blank():
    return {"heading_deg": np.nan, "peak_roll_deg": np.nan, "peak_pitch_deg": np.nan,
            "peak_lat_cm": np.nan, "min_height_m": np.nan,
            "push_L_deg": np.nan, "push_R_deg": np.nan,
            "lift_L": np.nan, "lift_R": np.nan, "max_dz_cm": np.nan,
            "quiet_L": np.nan, "quiet_R": np.nan,
            "fall": np.nan, "aborted": np.nan, "dab_end_s": np.nan}


# ------------------------------------------------------- the three readers
def score_rollout_segment(g, lift_cm, quiet_nm):
    """One repetition from rollout-recorder rows (already one dab segment)."""
    r = blank()
    t = g["episode_time"].to_numpy(float)
    n = len(g)
    if n < 5:
        return None
    r["dab_end_s"] = float(t[-1])
    r["aborted"] = int(t[-1] < DAB_END_S)

    q = np.stack([g[f"dof_pos_{i}"].to_numpy(float) for i in range(29)], 1)
    tgt = np.stack([g[f"target_dof_pos_{i}"].to_numpy(float) for i in range(29)], 1)
    r["push_L_deg"], r["push_R_deg"] = push_over_raise(t, tgt, q)

    w, x, y, z = [g[f"base_quat_{i}"].to_numpy(float) for i in range(4)]
    roll = np.degrees(roll_from_wxyz(w, x, y, z))
    pitch = np.degrees(pitch_from_wxyz(w, x, y, z))
    yaw = np.unwrap(yaw_from_wxyz(w, x, y, z))
    r["heading_deg"] = float(np.degrees(yaw[-1] - yaw[0]))
    m = (t >= WIN_LEAN[0]) & (t <= WIN_LEAN[1])
    r["peak_roll_deg"] = peak_signed(roll[m] - roll[0])
    r["peak_pitch_deg"] = peak_signed(pitch[m] - pitch[0])
    r["fall"] = int(bool(np.any(np.abs(roll) > FALL_DEG) or np.any(np.abs(pitch) > FALL_DEG)))

    # kinematic lift proxy
    zl, zr = foot_heights_world(q, (w, x, y, z))
    dz = (zl - zr) * 100.0
    post = t >= POST_ENTRY
    dt = float(np.median(np.diff(t))) if n > 1 else 0.02
    r["max_dz_cm"] = float(np.max(np.abs(dz[post]))) if post.any() else np.nan
    r["lift_L"] = int(longest_run(post & (dz >= lift_cm), t, dt) >= MIN_EVENT_S)
    r["lift_R"] = int(longest_run(post & (dz <= -lift_cm), t, dt) >= MIN_EVENT_S)

    # ankle-quiet proxy from tau_est
    if "tau_est_4" in g.columns:
        for side, ip, ir in (("L", L_ANK_P, L_ANK_R), ("R", R_ANK_P, R_ANK_R)):
            mag = np.hypot(g[f"tau_est_{ip}"].to_numpy(float), g[f"tau_est_{ir}"].to_numpy(float))
            r[f"quiet_{side}"] = int(longest_run(post & (mag < quiet_nm), t, dt) >= MIN_EVENT_S)

    # Vicon, when merged in
    vic = {"base_pos_x", "base_pos_y", "base_pos_z", "base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"}
    if vic.issubset(g.columns) and g["base_pos_z"].notna().any():
        px, py, pz = [g[f"base_pos_{c}"].to_numpy(float) for c in "xyz"]
        qcol = "base_quat_urdf_{}" if "base_quat_urdf_w" in g.columns else "base_quat_{}"
        vw, vx, vy, vz = [g[qcol.format(c)].to_numpy(float) for c in "wxyz"]
        yaw0 = yaw_from_wxyz(vw[0], vx[0], vy[0], vz[0])
        lat = lateral_cm(px, py, yaw0, 0)
        r["peak_lat_cm"] = peak_signed(lat[m])
        r["min_height_m"] = float(np.nanmin(pz))
    return r


def read_rollout(path, lift_cm, quiet_nm):
    df = pd.read_csv(path)
    rows = []
    if "rep_id" in df.columns:
        groups = [(str(k), g.sort_values("rep_step" if "rep_step" in g.columns else "step")) for k, g in df.groupby("rep_id")]
    else:
        is_dab = df["state"].astype(str).str.startswith("asap_dab")
        # a new segment starts when the state changes or episode_time restarts
        restart = df["episode_time"].diff().fillna(0) < 0
        seg = ((df["state"] != df["state"].shift()) | restart).cumsum()
        groups = [(f"seg{int(k):03d}", g) for k, g in df[is_dab].groupby(seg[is_dab])]
    for rep, g in groups:
        if len(g) < 25:      # less than half a second: a key bounce, not a dab
            continue
        r = score_rollout_segment(g, lift_cm, quiet_nm)
        if r is None:
            continue
        r.update({"policy": str(g["state"].iloc[0]), "rep": rep, "source": "robot rollout",
                  "file": os.path.basename(path)})
        rows.append(r)
    return rows


def read_diag(path, policy, lift_cm, quiet_nm):
    df = pd.read_csv(path)
    a = df[df["tag"] == "asap"]
    if len(a) < 5:
        return []
    t = a["time_ms"].to_numpy(float) / 1000.0
    q = np.stack([a[f"q{i}"].to_numpy(float) for i in range(29)], 1)
    tgt = np.stack([a[f"tgt{i}"].to_numpy(float) for i in range(29)], 1)
    r = blank()
    r["push_L_deg"], r["push_R_deg"] = push_over_raise(t, tgt, q)
    r["dab_end_s"] = float(t[-1])
    r["aborted"] = int(t[-1] < DAB_END_S)
    label = policy or policy_from_path(path) or "unknown"
    r.update({"policy": label, "rep": os.path.splitext(os.path.basename(path))[0],
              "source": "robot dab diag", "file": os.path.basename(path)})
    return [r]


def loco_joint_mapping():
    """joint_mapping of the locomotion policy, parsed without a yaml library."""
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.join(here, "..", "policy", "g1", "robomimic", "locomotion", "config.yaml")
    try:
        txt = open(cfg).read()
    except OSError:
        return None
    m = re.search(r"joint_mapping:\s*\[([^\]]*)\]", txt, re.S)
    if not m:
        return None
    return [int(v) for v in re.findall(r"-?\d+", m.group(1))]


def read_sim_trial(d, lift_cm, quiet_nm):
    """One MuJoCo harness trial folder, scored as tools/summarize.py does."""
    ev = [json.loads(l) for l in open(os.path.join(d, "events.jsonl")) if l.strip()]
    meta = next((e["meta"] for e in ev if "meta" in e), {})
    key = next((e["mono_s"] for e in ev if e.get("key") == "5"), None)
    a = pd.read_csv(os.path.join(d, "state.csv"))
    log = pd.read_csv(os.path.join(d, "dab_log.csv"))
    asap = log[log["tag"] == "asap"]
    if key is None or len(asap) < 5:
        return []
    r = blank()

    # align the observer clock to the handoff: the observer sample whose joints
    # best match the last locomotion snapshot, within 0.25 s of the key press
    mono = a["mono_s"].to_numpy(float)
    sel = np.where((mono >= key - 0.03) & (mono < key + 0.25))[0]
    if len(sel) == 0:
        sel = np.array([np.searchsorted(mono, key)])
    mapping = loco_joint_mapping()
    pre = log[log["tag"] == "loco_last"]
    if mapping is not None and len(pre):
        sdk = np.zeros(29)
        for i, j in enumerate(mapping):
            sdk[j] = float(pre.iloc[0][f"q{i}"])
        qa = np.stack([a[f"q{i}"].to_numpy(float) for i in range(29)], 1)
        err = ((qa[sel] - sdk) ** 2).sum(1)
        i0 = int(sel[np.argmin(err)])
    else:
        i0 = int(sel[0])
    wall = mono - mono[i0]
    lwall = asap["wallclock_ms"].to_numpy(float) / 1000.0
    lt = asap["time_ms"].to_numpy(float) / 1000.0
    phase = np.interp(wall, lwall, lt, left=0.0, right=lt[-1])
    use = (wall >= 0) & (wall <= lwall[-1])
    main = use & (phase >= POST_ENTRY)
    sim_t = a["sim_s"].to_numpy(float)

    yaw = np.unwrap(a["pelvis_yaw"].to_numpy(float))
    r["heading_deg"] = float(np.degrees(yaw[use][-1] - yaw[i0]))
    w, x, y, z = [a[c].to_numpy(float) for c in ("qw", "qx", "qy", "qz")]
    roll = np.degrees(roll_from_wxyz(w, x, y, z))
    pitch = np.degrees(pitch_from_wxyz(w, x, y, z))
    m = (wall >= 0) & (phase >= WIN_LEAN[0]) & (phase <= WIN_LEAN[1])
    r["peak_roll_deg"] = peak_signed(roll[m] - roll[i0])
    r["peak_pitch_deg"] = peak_signed(pitch[m] - pitch[i0])
    yaw0 = yaw_from_wxyz(w[i0], x[i0], y[i0], z[i0])
    lat = lateral_cm(a["base_x"].to_numpy(float), a["base_y"].to_numpy(float), yaw0, i0)
    r["peak_lat_cm"] = peak_signed(lat[m])
    r["min_height_m"] = float(a["base_z"].to_numpy(float)[use].min())
    for side in ("L", "R"):
        nocontact = main & (a[f"{side}contacts"].to_numpy(float) == 0)
        r[f"lift_{side}"] = int(longest_run(nocontact, sim_t, 0.01) >= MIN_EVENT_S)
    # the robot's kinematic proxy, computed the same way from the observer's
    # joints and pelvis quaternion, so the two columns are comparable
    qa = np.stack([a[f"q{i}"].to_numpy(float) for i in range(29)], 1)
    zl, zr = foot_heights_world(qa, (w, x, y, z))
    r["max_dz_cm"] = float(np.max(np.abs(zl - zr)[main]) * 100.0) if main.any() else np.nan

    tq = np.stack([asap[f"q{i}"].to_numpy(float) for i in range(29)], 1)
    tt = np.stack([asap[f"tgt{i}"].to_numpy(float) for i in range(29)], 1)
    r["push_L_deg"], r["push_R_deg"] = push_over_raise(lt, tt, tq)
    r["fall"] = int(any("Fall detected" in e.get("line", "") for e in ev))
    r["dab_end_s"] = float(lt[-1])
    r["aborted"] = int(lt[-1] < DAB_END_S and not r["fall"])
    pol = str(meta.get("policy", "")).rstrip("/").split("/")[-1] or os.path.basename(d.rstrip("/"))
    if pol.startswith("(shipped"):
        pol = "asap_dab"
    pol = ALIASES.get(pol, pol)
    r.update({"policy": pol, "rep": os.path.basename(d.rstrip("/")), "source": "mujoco",
              "file": os.path.basename(d.rstrip("/"))})
    return [r]


def sniff(path):
    with open(path) as f:
        head = f.readline()
    cols = set(head.strip().split(","))
    if {"tag", "time_ms", "tgt0"}.issubset(cols):
        return "diag"
    if {"state", "episode_time", "dof_pos_0"}.issubset(cols):
        return "rollout"
    return None


# ------------------------------------------------------------------ tables
def fmt(v, spec="{:+.1f}"):
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return "n/a"
    return spec.format(v)


def fmt_flag(v):
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return "n/a"
    return "yes" if int(v) else "no"


def per_rep_table(rows):
    print("Per repetition")
    print("policy | source | rep | heading deg | peak roll deg | peak pitch deg | lateral cm | min height m | push L / R deg | push L / R N m | lift L / R | max foot rise cm | quiet L / R | fall | ended s")
    print("---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:")
    for r in rows:
        nm = "{} / {}".format(fmt(push_nm(r["push_L_deg"]), "{:.1f}"), fmt(push_nm(r["push_R_deg"]), "{:.1f}"))
        fall = "fall" if r["fall"] == 1 else ("aborted" if r["aborted"] == 1 else ("no" if r["fall"] == 0 or r["aborted"] == 0 else "n/a"))
        print(" | ".join([
            r["policy"], r["source"], r["rep"],
            fmt(r["heading_deg"]), fmt(r["peak_roll_deg"]), fmt(r["peak_pitch_deg"]),
            fmt(r["peak_lat_cm"]), fmt(r["min_height_m"], "{:.3f}"),
            "{} / {}".format(fmt(r["push_L_deg"]), fmt(r["push_R_deg"])), nm,
            "{} / {}".format(fmt_flag(r["lift_L"]), fmt_flag(r["lift_R"])), fmt(r["max_dz_cm"], "{:.2f}"),
            "{} / {}".format(fmt_flag(r["quiet_L"]), fmt_flag(r["quiet_R"])),
            fall, fmt(r["dab_end_s"], "{:.2f}")]))
    print()


def med_abs(v):
    v = np.array([x for x in v if x is not None and not np.isnan(x)], float)
    return float(np.median(np.abs(v))) if len(v) else np.nan


def med(v):
    v = np.array([x for x in v if x is not None and not np.isnan(x)], float)
    return float(np.median(v)) if len(v) else np.nan


def count_known(v):
    v = [x for x in v if x is not None and not np.isnan(x)]
    return int(sum(int(x) for x in v)), len(v)


def per_policy_table(rows):
    print("Per policy and source (medians over repetitions; k/n counts repetitions with the quantity)")
    print("policy | source | n | feet planted | max foot rise, median cm | turns >15 | falls | aborted | abs heading, median (signed median) deg | abs peak roll, median (signed) deg | abs lateral, median cm | min height, median m | push L / R, median deg | push L / R, median N m | ankle quiet L / R")
    print("---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:")
    keys = sorted(set((r["policy"], r["source"]) for r in rows), key=lambda k: (k[1], k[0]))
    for pol, src in keys:
        rr = [r for r in rows if r["policy"] == pol and r["source"] == src]
        n = len(rr)
        lifts = [max(r["lift_L"], r["lift_R"]) if not (np.isnan(r["lift_L"]) or np.isnan(r["lift_R"])) else np.nan for r in rr]
        k, kn = count_known(lifts)
        planted = "{}/{}".format(kn - k, kn) if kn else "n/a"
        turns = count_known([abs(r["heading_deg"]) > TURN_DEG if not np.isnan(r["heading_deg"]) else np.nan for r in rr])
        falls = count_known([r["fall"] for r in rr])
        ab = count_known([r["aborted"] for r in rr])
        ql = count_known([r["quiet_L"] for r in rr]); qr = count_known([r["quiet_R"] for r in rr])
        pl, pr = med([r["push_L_deg"] for r in rr]), med([r["push_R_deg"] for r in rr])
        print(" | ".join([
            pol, src, str(n), planted, fmt(med([r["max_dz_cm"] for r in rr]), "{:.2f}"),
            "{}/{}".format(*turns) if turns[1] else "n/a",
            "{}/{}".format(*falls) if falls[1] else "n/a",
            "{}/{}".format(*ab) if ab[1] else "n/a",
            "{} ({})".format(fmt(med_abs([r["heading_deg"] for r in rr]), "{:.1f}"), fmt(med([r["heading_deg"] for r in rr]))),
            "{} ({})".format(fmt(med_abs([r["peak_roll_deg"] for r in rr]), "{:.1f}"), fmt(med([r["peak_roll_deg"] for r in rr]))),
            fmt(med_abs([r["peak_lat_cm"] for r in rr]), "{:.1f}"),
            fmt(med([r["min_height_m"] for r in rr]), "{:.3f}"),
            "{} / {}".format(fmt(pl), fmt(pr)),
            "{} / {}".format(fmt(push_nm(pl), "{:.1f}"), fmt(push_nm(pr), "{:.1f}")),
            "{}/{} / {}/{}".format(ql[0], ql[1], qr[0], qr[1]) if ql[1] else "n/a"]))
    print()
    print("feet planted: repetitions with no lift event (robot: lowest-corner foot height difference of {:.1f} cm or more for 80 ms after 0.5 s; "
          "simulator: no foot-floor contact for 80 ms after 0.5 s). max foot rise: the same height difference, largest after 0.5 s, "
          "kinematic in both. ankle quiet: repetitions with both ankle tau_est below {:.1f} N m for 80 ms after 0.5 s, a weak proxy "
          "that fires on about half of planted v6 repetitions.".format(LIFT_CM, QUIET_NM))


def main(argv=None):
    global LIFT_CM, QUIET_NM
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="\n".join(__doc__.split("Usage\n-----\n")[1:]))
    ap.add_argument("files", nargs="*", help="robot rollout CSVs and dab diagnostic CSVs")
    ap.add_argument("--sim", nargs="*", default=[], help="MuJoCo harness trial folders")
    ap.add_argument("--policy", default=None, help="policy label for diagnostic CSVs")
    ap.add_argument("--lift-cm", type=float, default=LIFT_CM)
    ap.add_argument("--quiet-nm", type=float, default=QUIET_NM)
    ap.add_argument("--csv", default=None, help="write the per-repetition rows to this CSV")
    ap.add_argument("--no-reps", action="store_true", help="print only the per-policy table")
    args = ap.parse_args(argv)
    LIFT_CM, QUIET_NM = args.lift_cm, args.quiet_nm

    rows = []
    for f in args.files:
        if os.path.isdir(f):
            if os.path.exists(os.path.join(f, "events.jsonl")):
                rows += read_sim_trial(f, args.lift_cm, args.quiet_nm)
            continue
        kind = sniff(f)
        if kind == "rollout":
            rows += read_rollout(f, args.lift_cm, args.quiet_nm)
        elif kind == "diag":
            rows += read_diag(f, args.policy, args.lift_cm, args.quiet_nm)
        else:
            print(f"skipping {f}: not a rollout or dab diagnostic CSV", file=sys.stderr)
    for d in args.sim:
        if os.path.exists(os.path.join(d, "events.jsonl")) and os.path.exists(os.path.join(d, "state.csv")):
            rows += read_sim_trial(d, args.lift_cm, args.quiet_nm)
        else:
            print(f"skipping {d}: not a harness trial folder", file=sys.stderr)
    if not rows:
        print("no repetitions found", file=sys.stderr)
        return 1
    rows.sort(key=lambda r: (r["source"], r["policy"], r["file"], r["rep"]))
    if not args.no_reps:
        per_rep_table(rows)
    per_policy_table(rows)
    if args.csv:
        pd.DataFrame(rows).to_csv(args.csv, index=False)
        print(f"\nper-repetition rows written to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
