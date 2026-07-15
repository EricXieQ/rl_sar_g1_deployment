# Vicon ↔ rl_sar real-world capture (ASAP)

Capture G1 per-joint **state + action** together with the **pelvis 6-DOF pose**
from Vicon, aligned on one timeline, for ASAP delta-model training.

Two logs, one shared Jetson clock (`t_wall`), stitched offline.

## 1. On the Vicon PC (Tracker 3.10, once)
- Enable **UDP Object Stream** (default port **51001**); point it at the Jetson IP
  (unicast) or broadcast on the mocap subnet.
- Build an **Object** from the pelvis marker cluster, name it e.g. `g1_pelvis`.
- Note the fixed transform between the Vicon object frame and the robot
  pelvis/IMU frame (align axes with the robot in a known standing pose).

## 2. On the robot Jetson — record both streams
Terminal A (Vicon listener, pure stdlib, same clock as rl_sar):
```bash
python3 scripts/vicon/vicon_listener.py --object g1_pelvis
# -> logs/vicon_<timestamp>.csv
```
Terminal B (rl_sar with rollout recording on):
```bash
RL_RECORD=1 ./cmake_build/bin/rl_real_g1   # NOT build/rl_sar/... (stale)
# -> logs/rollout_<timestamp>.csv
```
At the **start of each rollout**, deliver one sharp **stomp/tap** and press `.`
in the rl_sar terminal at that instant — it flags a `sync_mark` row used as the
cross-correlation anchor. (Vicon is passive; the robot never triggers it.)

## 3. Offline — merge + auto-sync
```bash
/usr/bin/python3 scripts/vicon/merge_rollout_vicon.py \
    --rollout logs/rollout_<ts>.csv \
    --vicon   logs/vicon_<ts>.csv \
    --object  g1_pelvis --out merged_<ts>.csv
```
Cross-correlates the IMU-accel spike vs the Vicon pelvis-accel spike (within
±3 s of the `sync_mark`), applies the sub-ms offset, resamples pose onto every
policy step, and appends `base_pos/quat` + world- & base-frame `lin/ang_vel`.
Use `--manual-offset <sec>` to bypass correlation.

## Notes
- `rollout_*.csv` logs `num_of_dofs` joint columns — the asap_dab config is
  **29 DOF**. Confirm that matches the physical robot before trusting the data.
- IMU gives drift-free orientation but no position — position comes only from
  Vicon; that's why both streams are required.
- Vicon `t_wall` is packet-arrival time on the Jetson (shared clock); the
  cross-correlation removes the residual network/pipeline latency.
