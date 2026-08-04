#!/usr/bin/env python3
"""dab_clean_*.csv -> ASAP motion_lib pkl, with real actions baked in.

ASAP has no loader for real rollouts: `delta_dynamics_model.py` is dead code (it
calls env.get_input_dim/get_output_dim/parse_delta, none of which exist), and the
working path -- `train_delta_a.py` + `envs/delta_a/delta_a_open_loop.py` -- pulls
its reference action from `_motion_lib.get_motion_actions()`. So real data enters
as a motion pkl carrying an `action` key, which is what flips has_action=True in
motion_lib_base.py:384. Stock motion pkls do not have it.

Emits one motion entry per repetition into a single pkl, matching the layout
motion_lib expects (top-level dict keyed by motion name).

Schema, verified against
  data/motions/g1_29dof_anneal_23dof/TairanTestbed/singles/*.pkl

  root_trans_offset (T,3) f32    root translation
  root_rot          (T,4) f64    quaternion, XYZW (scipy convention)
  dof               (T,23) f32   joint angles, ASAP 23-DoF order
  pose_aa           (T,27,3) f32 axis-angle: [0]=root, [1..23]=dof*axis, [24..26]=0
  action            (T,23) f32   raw policy output -- the load-bearing key
  fps               scalar

`pose_aa[:, j+1] = dof[:, j] * axis_j` is not a guess: fk_batch recovers dof as
`pose.sum(dim=-1)[..., 1:]` (torch_humanoid_batch.py:216), which only works
because each Unitree joint is 1-DoF on a signed unit axis. Verified against the
reference dab pkl to 0.000e+00.

Usage:
  python3 export_asap_motion.py --in dab_clean_20260730.csv --out s1.pkl \
      [--tag s1] [--no-reref] [--z-offset 0.0]
"""
import argparse
import numpy as np
import pandas as pd

try:
    import joblib
except ImportError:
    raise SystemExit("needs joblib (same dependency ASAP's motion_lib uses)")

# ASAP 23-DoF order == the rl_sar 29-DoF order minus the six wrists.
# rl_sar idx: 19,20,21 = L wrist roll/pitch/yaw; 26,27,28 = R wrist roll/pitch/yaw.
CSV_DOF_IDX = list(range(0, 19)) + [22, 23, 24, 25]
assert len(CSV_DOF_IDX) == 23

# Joint axes read from g1_29dof_anneal_23dof_fitmotionONLY.xml, in body order.
DOF_AXIS = np.array([
    [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0],   # left leg
    [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0],   # right leg
    [0, 0, 1], [1, 0, 0], [0, 1, 0],                                     # waist yaw/roll/pitch
    [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0],                          # left arm
    [0, 1, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0],                          # right arm
], dtype=np.float64)
N_BODIES = 27          # 24 MJCF bodies + 3 extend_config bodies


def q_to_R(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def R_to_q_xyzw(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = .25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2]) * 2
        w, x, y, z = (R[2,1]-R[1,2])/s, .25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2]) * 2
        w, x, y, z = (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, .25*s, (R[1,2]+R[2,1])/s
    else:
        s = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1]) * 2
        w, x, y, z = (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, .25*s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def quat_xyzw_to_aa(q):
    """xyzw quaternion -> axis-angle, shortest rotation."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q = np.where(q[..., 3:4] < 0, -q, q)                 # shortest path
    w = np.clip(q[..., 3], -1.0, 1.0)
    ang = 2.0 * np.arccos(w)
    s = np.sqrt(np.maximum(1.0 - w*w, 0.0))
    small = s < 1e-8
    axis = np.where(small[..., None], np.array([1.0, 0.0, 0.0]), q[..., :3] / np.where(small, 1.0, s)[..., None])
    return axis * np.where(small, 0.0, ang)[..., None]


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default="real", help="prefix for motion names")
    ap.add_argument("--no-reref", action="store_true",
                    help="keep absolute world pose (default re-references each rep "
                         "to its own start position and heading, which is what makes "
                         "the two sessions' 26.5 deg yaw calibration gap irrelevant)")
    ap.add_argument("--marker-offset", default="0,0,-0.083",
                    help="offset from the URDF pelvis origin to the Vicon marker-cluster "
                         "centroid, expressed in the PELVIS BODY frame, as x,y,z metres. "
                         "Applied as pos_pelvis = pos_vicon - R @ d, so pelvis tilt no "
                         "longer swings the reported position. Default z=-0.083 was "
                         "derived by FK: pelvis height above the foot contact spheres "
                         "from the recorded joint angles, minus the Vicon height, over "
                         "600 standing-hold frames per session -- the two sessions agree "
                         "to 1mm. x and y are NOT identifiable from that fit (only the z "
                         "equation is available and R[2,:] barely varies while upright), "
                         "so they default to 0; pitch/roll the pelvis +-20deg while "
                         "planted during a calibration take to make them observable. "
                         "Pass 0,0,0 to disable.")
    args = ap.parse_args()
    d_off = np.array([float(v) for v in args.marker_offset.split(",")], dtype=np.float64)
    assert d_off.shape == (3,), "--marker-offset wants x,y,z"

    df = pd.read_csv(args.inp)
    # Per-rep dt, so the gaps between reps don't skew the median.
    dts = np.concatenate([np.diff(g.sort_values("rep_step").t_wall.to_numpy())
                          for _, g in df.groupby("rep_id")])
    measured = 1.0 / np.median(dts)
    # fps MUST be an integer. torch_humanoid_batch.py:225 does
    # `return_dict.fps = int(1/dt)`, which TRUNCATES -- a stored 48.88 becomes 48,
    # a 1.8% timebase error that accumulates to a 5-frame drift over a 6s rep.
    # Round instead: 49 sits 0.2% from the measured rate, an order of magnitude
    # inside the real per-step jitter (std ~0.53ms, ~2.7% of a step).
    fps = int(round(measured))
    err = abs(fps - measured) / measured
    print(f"  measured rate {measured:.3f} Hz -> fps={fps} ({100*err:.2f}% off, "
          f"jitter std {1000*np.std(dts):.2f} ms)")
    if err > 0.01:
        print(f"  WARNING: rounding error {100*err:.1f}% exceeds 1% -- resample before export")

    out = {}
    for rep, g in df.groupby("rep_id"):
        g = g.sort_values("rep_step")
        T = len(g)

        pos = g[["base_pos_x", "base_pos_y", "base_pos_z"]].to_numpy(np.float64)
        qw = g[["base_quat_urdf_w", "base_quat_urdf_x",
                "base_quat_urdf_y", "base_quat_urdf_z"]].to_numpy(np.float64)

        Rs = np.stack([q_to_R(q) for q in qw])

        # Marker-cluster centroid -> URDF pelvis origin. This must happen BEFORE
        # re-referencing, and it must go through R: the offset is fixed in the body
        # frame, so an 83mm lever arm swings the centroid ~15mm at 10 deg of pelvis
        # tilt -- motion the pelvis origin never underwent. A scalar z shift cannot
        # represent that.
        if np.any(d_off):
            pos = pos - np.einsum("tij,j->ti", Rs, d_off)

        if not args.no_reref:
            # Rotate the whole rep so it starts facing +x at the origin. Removes
            # the per-session yaw calibration (which drifts 26.5 deg between
            # sessions and 5.7 deg within one) from the data entirely. rz() leaves
            # z untouched, so height survives; only x,y are zeroed.
            yaw0 = np.arctan2(Rs[0][1, 0], Rs[0][0, 0])
            Ry = rz(-yaw0)
            z0 = pos[0, 2]
            Rs = Ry @ Rs
            pos = (Ry @ (pos - pos[0]).T).T
            pos[:, 2] += z0                     # restore true (offset-corrected) height

        root_rot = np.stack([R_to_q_xyzw(R) for R in Rs])

        dof = g[[f"dof_pos_{i}" for i in CSV_DOF_IDX]].to_numpy(np.float64)
        act = g[[f"action_{i}" for i in CSV_DOF_IDX]].to_numpy(np.float64)

        pose_aa = np.zeros((T, N_BODIES, 3), dtype=np.float64)
        pose_aa[:, 0, :] = quat_xyzw_to_aa(root_rot)
        pose_aa[:, 1:24, :] = dof[:, :, None] * DOF_AXIS[None, :, :]
        # bodies 24..26 are extend_config virtual bodies -- no DoF, stay zero.

        out[f"{args.tag}_rep{int(rep):04d}"] = {
            "root_trans_offset": pos.astype(np.float32),
            "root_rot": root_rot,                      # float64 xyzw
            "dof": dof.astype(np.float32),
            "pose_aa": pose_aa.astype(np.float32),
            "action": act.astype(np.float32),
            "fps": fps,
        }

    joblib.dump(out, args.out)

    # ---- self-check: the same identity fk_batch relies on --------------------
    k = next(iter(out))
    m = out[k]
    err = np.abs(m["pose_aa"].sum(-1)[:, 1:24] - m["dof"]).max()
    print(f"wrote {args.out}")
    print(f"  motions: {len(out)}   fps: {fps}   frames: {sum(len(v['dof']) for v in out.values())}")
    print(f"  self-check pose_aa.sum(-1)[1:24] == dof : max err {err:.3e}")
    print(f"  re-reference: {'OFF (absolute world)' if args.no_reref else 'ON (per-rep origin + heading)'}")
    if np.any(d_off):
        print(f"  marker->pelvis offset applied (body frame): "
              f"[{d_off[0]:+.4f}, {d_off[1]:+.4f}, {d_off[2]:+.4f}] m")
        if d_off[0] == 0.0 and d_off[1] == 0.0:
            print("        x,y are 0 because the FK fit cannot identify them while the robot")
            print("        is upright. Cross-check d_z against a tape measure from the marker")
            print("        plate to the pelvis origin -- a systematic disagreement means the")
            print("        hoist harness was carrying weight when the estimate was taken.")
    else:
        print("  WARNING: no marker->pelvis offset. root z is the marker-cluster centroid,")
        print("        ~83mm below the URDF pelvis origin. The delta model will try to")
        print("        explain that measurement offset as dynamics.")


if __name__ == "__main__":
    main()
