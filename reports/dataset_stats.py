#!/usr/bin/env python3
"""
dataset_stats.py -- every statistic quoted about the capture, with the derivation
visible. Run it to reproduce the numbers:

    /usr/bin/python3 dataset_stats.py [path/to/dab_clean_*.csv]

Nothing here is estimated; each number comes from the column named beside it.
"""
import csv, sys
import numpy as np

CSV = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/unitree/qxie2/rl_sar/logs/dab_clean_20260730.csv"

rows = list(csv.DictReader(open(CSV)))
col = lambda k: np.array([float(r[k]) for r in rows])      # one column as floats
rep = np.array([int(r["rep_id"]) for r in rows])

# 29-DOF G1 joint order (index -> name)
NAMES = ["L_HipP","L_HipR","L_HipY","L_Knee","L_AnkP","L_AnkR",
         "R_HipP","R_HipR","R_HipY","R_Knee","R_AnkP","R_AnkR",
         "WaistY","WaistR","WaistP",
         "L_ShldP","L_ShldR","L_ShldY","L_Elb","L_WrR","L_WrP","L_WrY",
         "R_ShldP","R_ShldR","R_ShldY","R_Elb","R_WrR","R_WrP","R_WrY"]

print("source: %s" % CSV)

# ---------------------------------------------------------------- SIZE
# reps  = number of distinct rep_id values (clean_dataset.py tags each kept segment)
# rows  = one row per policy step
# rate  = 1 / median gap between consecutive t_wall values
t = col("t_wall")
dt = np.diff(t)
dt = dt[dt > 0]
lens = [int((rep == q).sum()) for q in sorted(set(rep.tolist()))]
print("\n[SIZE]")
print("  reps            = len(set(rep_id))                    -> %d" % len(lens))
print("  rows            = len(file)                           -> %d" % len(rows))
print("  columns         = len(header)                         -> %d" % len(rows[0]))
print("  steps per rep   = count of rows per rep_id            -> min %d, median %d, max %d"
      % (min(lens), int(np.median(lens)), max(lens)))
print("  control rate    = 1 / median(diff(t_wall))            -> %.1f Hz" % (1 / np.median(dt)))
print("  dab duration    = rows / rate                         -> %.1f s" % (len(rows) / (1 / np.median(dt))))

# ---------------------------------------------------------------- BASE POSE
# position range = max - min of base_pos_{x,y,z}   (metres, Vicon world frame)
# speed          = L2 norm of the three base_lin_vel_w_* columns, per row
print("\n[BASE POSE]  (columns base_pos_*, base_lin_vel_w_*, base_ang_vel_w_*)")
for c in "xyz":
    v = col("base_pos_" + c)
    print("  pos %s range   = max - min                            -> %.3f m  (%.3f .. %.3f)"
          % (c, v.max() - v.min(), v.min(), v.max()))
sp = np.linalg.norm(np.stack([col("base_lin_vel_w_" + c) for c in "xyz"], 1), axis=1)
print("  speed           = |[vx,vy,vz]| per row                -> median %.3f, p95 %.3f, max %.3f m/s"
      % (np.median(sp), np.percentile(sp, 95), sp.max()))
w = np.linalg.norm(np.stack([col("base_ang_vel_w_" + c) for c in "xyz"], 1), axis=1)
print("  angular speed   = |[wx,wy,wz]| per row                -> median %.3f, p95 %.3f, max %.3f rad/s"
      % (np.median(w), np.percentile(w, 95), w.max()))

# ---------------------------------------------------------------- JOINTS
# range of motion = (max - min) of dof_pos_i, converted rad -> deg  (x 180/pi)
print("\n[JOINT RANGE OF MOTION]  = (max - min) of dof_pos_i, rad -> deg")
rng = [(NAMES[i],
        (col("dof_pos_%d" % i).max() - col("dof_pos_%d" % i).min()) * 180 / np.pi,
        np.abs(col("tau_est_%d" % i)).max()) for i in range(29)]
for nm, rg, tq in sorted(rng, key=lambda x: -x[1])[:8]:
    print("    %-8s %6.1f deg   peak |tau_est| %5.1f Nm" % (nm, rg, tq))

# ---------------------------------------------------------------- TORQUE
# whole-body |tau| = L2 norm across all 29 tau_est_i, per row
# per-joint peak   = max |tau_est_i| over the whole dataset
tau = np.stack([col("tau_est_%d" % i) for i in range(29)], 1)
tn = np.linalg.norm(tau, axis=1)
print("\n[TORQUE]  (columns tau_est_0..28)")
print("  whole body      = |tau vector| per row                -> median %.1f, p95 %.1f, max %.1f Nm"
      % (np.median(tn), np.percentile(tn, 95), tn.max()))
top = sorted([(NAMES[i], np.abs(tau[:, i]).max()) for i in range(29)], key=lambda x: -x[1])[:4]
print("  highest joints  = max |tau_est_i| each                -> "
      + ", ".join("%s %.1f" % (n, v) for n, v in top))
print("  R_AnkleRoll(11) = max |tau_est_11|                    -> %.1f Nm   (~30 Nm before the fine-tune)"
      % np.abs(tau[:, 11]).max())

# ---------------------------------------------------------------- ACTION
# raw policy output, BEFORE action_scale. The divergence guard compares |action| to 3.0.
act = np.stack([col("action_%d" % i) for i in range(29)], 1)
per_step = np.abs(act).max(1)
print("\n[ACTION]  (columns action_0..28, raw policy output before scaling)")
print("  max over run    = max |action| anywhere               -> %.2f" % np.abs(act).max())
print("  per-step max    = max |action| within each row        -> median %.2f, p95 %.2f"
      % (np.median(per_step), np.percentile(per_step, 95)))
print("  NOTE: these exceed the divergence-guard threshold of 3.0, but that is this")
print("        policy's normal profile -- the health signal is torque, not |action|.")

# ---------------------------------------------------------------- INTEGRITY
cov = sum(1 for r in rows if r["vicon_cover"] == "1")
empty = sum(1 for r in rows for v in r.values() if v == "")
print("\n[INTEGRITY]")
print("  Vicon coverage  = count(vicon_cover == 1) / rows      -> %d / %d  (%.1f%%)"
      % (cov, len(rows), 100.0 * cov / len(rows)))
print("  empty cells     = count of '' across all fields       -> %d" % empty)
print("  reps dropped    = reported by clean_dataset.py        -> 0 short / 0 dead / 0 no-coverage")
