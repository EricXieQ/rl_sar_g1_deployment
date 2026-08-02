# ASAP real-world data collection on the Unitree G1 — status

Last updated **2 August 2026**. Everything here is measured, not estimated;
reproduce with `reports/dataset_stats.py` and `reports/validate.py`.

---

## 1. Where the project stands

**Goal.** ASAP closes the sim-to-real gap by fitting a *delta action model* to real
rollouts. Training it needs, at every control step, the joint state, the action the
policy actually sent, **and the floating-base pose** — which the robot cannot measure
itself, because an IMU gives orientation but not position. So: capture robot
state/action plus external motion capture of the pelvis, on one defensible timeline.

**Done.**
- Capture pipeline built, validated against an independent sensor, and documented
- **473 dab repetitions** collected across two clean sessions (~139k timesteps)
- Vicon→URDF calibration solved per session
- Three blocking faults diagnosed and fixed (§3)
- Both datasets published with dataset cards

**Not done.**
- The delta action model has not been fitted yet — that is the training PC's next step
- Only one skill (`wall_background_dab`) has been captured
- The per-joint sim-to-real error profile has not been measured (§6)

---

## 2. The data

| | session 1 | session 2 | total |
|---|---|---|---|
| Date (local) | Wed 29 Jul | Sat 2 Aug | |
| Repetitions | 69 | **404** | **473** |
| Timesteps | 20,556 | 120,265 | 140,821 |
| Transitions (`s_t → s_{t+1}`) | 20,487 | 119,861 | 140,348 |
| Dab motion | 7.0 min | 41.0 min | 48 min |
| Session length | 11 min | 65 min | |
| Locomotion retained | 3.7 min | ~19 min | ~23 min |
| Vicon coverage | 100 % | 100 % | |
| Genuine flips / mislabels | 0 / 0 | 0 / 0 | |
| Rigid-fit residual (median) | **0.40 mm** | 2.52 mm | |
| Calibration (roll, pitch, yaw) | +4.51, +5.99, −3.71 | +1.22, +4.61, +22.82 | |

Session 1 is the **cleaner** data per frame; session 2 is far larger. Weight
accordingly.

**Files** (`~/qxie2/rl_sar/logs/`, gitignored — published as release assets):

| file | what |
|---|---|
| `dab_clean_2026{0730,0802}.csv` | **the training datasets** — dab only, calibrated, `rep_id`-tagged |
| `merged_2026*.csv` | all policy steps incl. locomotion, before filtering |
| `oscmarkers_2026*.csv` | raw labelled markers — lets the pose be re-solved |
| `rollout_2026*.csv` | raw robot state/action |

Releases: `capture-20260730`, `capture-20260802` at
github.com/EricXieQ/rl_sar_g1_deployment

---

## 3. The three faults that had to be fixed first

Each presented the same way — the robot cutting power mid-motion — but they were
unrelated.

**1. Right ankle wind-up (policy).** `R_ankle_roll` was commanded to ~100° against a
~15° joint limit; the PD controller dumped ~30 Nm into a gap it could never close and
firmware protection cut power. Root cause: `ankle_roll` is an *under-constrained*
action dimension — with planted feet it barely affects the tracking reward, so during
training the output drifted. Simulation hides it (joint limits, contact and the force
cap absorb the command); only hardware pays. Fixed by a fine-tune penalising commanded
targets outside joint limits. Result: **~100° → 1.7–2.9°, ~30 Nm → 0.6–3.5 Nm, no
power cuts.**

**2. Marker mislabelling (mocap).** The pelvis rigid body flipped ~180°, corrupting
orientation — 655 flips, 17.8 % of frames. Causes: the G1's silver shell throws IR
reflections that spawn phantom markers, the arm occludes pelvis markers during the
dab, and the left/right markers sit only ~75 mm apart (vs ~170 mm front/back). Fixed
by moving to Nexus and redefining the subject: Nexus streams **labelled** markers, so
identity is fixed and cannot be swapped. **655 flips → 0.**

**3. Dual-subnet DDS (network).** `rt/lowstate` stopped intermittently and the robot
needed repeated power cycles, while discovery and ping both looked healthy. A second
IP had been added to the robot-facing interface, so the Jetson advertised **two DDS
locators** and the robot intermittently sent data to the unreachable one. Discovery is
multicast (always arrives), data is unicast (silently vanished) — which is why it was
invisible. Fixed by keeping one address on `eth0` and giving the Vicon PC an address
on the robot's own subnet via a DHCP reservation.

An automatic marker-repair step was written and **abandoned**: it made the pose worse
(39 → 134 orientation jumps), because on a near-symmetric cluster a 180°-rotated
labelling fits the rigid geometry just as well. Fixing the measurement beat
post-processing it.

---

## 4. How the pipeline works

```
Vicon Nexus (Vicon PC, 192.168.123.50)
   └─ OSC, 4 labelled markers, 100 Hz, mm
        │  direct ethernet, 192.168.123.x
        ▼
JETSON  — one clock: time.time()
   ├─ osc_listener.py     stamps on arrival → oscmarkers CSV (raw)
   │     └─ Kabsch rigid fit → oscpose CSV (pelvis 6-DOF, 100 Hz)
   └─ rl_sar              rt/lowstate (1000 Hz) + policy → RecordRollout()
                                        → rollout CSV (state+action, 48.9 Hz)
OFFLINE
   merge_rollout_vicon.py  match on t_wall, interpolate onto steps, --max-rms filter
   clean_dataset.py        keep complete live dab reps, apply calibration, tag reps
   validate.py             Vicon vs IMU cross-check
```

**The one design decision that matters:** both streams are timestamped by the *same
machine*, so only one clock exists. Alignment is timestamp matching — no NTP/PTP, no
offset estimation, no sync event. Verified: cross-correlating Vicon-derived and IMU
angular velocity peaks at **0 ms lag**.

**Pose from markers.** Position is the centroid of the four markers (which is why it is
immune to label swaps — relabelling does not change an average). Orientation is the
least-squares rotation mapping the reference marker arrangement onto the current one
(Umeyama/Kabsch, Horn quaternion + Jacobi eigensolve). The fit residual doubles as a
per-frame health check.

---

## 5. Validation — how we know the data is right

Vicon (external cameras) and the onboard IMU measure the same motion through
completely separate physical paths. Neither can influence the other, so agreement is
independent evidence.

| test | result |
|---|---|
| Angular velocity, pitch axis | **corr 0.924** |
| Angular velocity, yaw axis | **corr 0.936** |
| Gravity, measured two ways | **+9.727 vs +9.719 m/s²** (0.08 % apart) |
| Time alignment | **0 ms lag** |
| Repeatability, 69 reps overlaid | **1.4 mm** median spread |
| IMU gravity magnitude | 9.820 m/s² (true 9.81) |

**What this does *not* prove.** Dynamic *linear acceleration* could not be validated
directly: differentiating position twice at 50 Hz amplifies noise enormously (0.4 mm
of position noise becomes ~1 m/s² of spurious acceleration), so that comparison is
noise-dominated. Position is validated only indirectly, via gravity and repeatability.
State this before someone finds it.

---

## 6. What the ASAP paper actually did, and how we compare

> *"collecting over **400 real-world motion clips** — the minimum required to train the
> full 23-DoF delta action model in simulation — poses significant challenges. Our
> experiments involve highly dynamic motions that cause rapid overheating of joint
> motors, leading to hardware failures (**two Unitree G1 robots broke** during data
> collection). Given these constraints, we adopt a more sample-efficient approach by
> focusing exclusively on learning a **4-DoF ankle** delta action model rather than the
> full-body 23-DoF model… we collect **100 motion clips**… We execute the tracking
> policy **30 times for each task**… we also collect **10 minutes of locomotion data**."*

| | paper | us |
|---|---|---|
| Clips per motion | ~20–30 | **473** |
| Total clips | 100 (5 motions) | 473 (1 motion) |
| Motions | 5 | **1** |
| Locomotion | 10 min | ~23 min |
| Delta model scope | 4-DoF ankle | (undecided) |

**Two things follow.**

**(a) The 4-DoF model is the ankles — our problem joint.** The paper narrows to the
ankle because *"the G1 features a mechanical linkage design in the ankle which
introduces a significant sim-to-real gap difficult to bridge with conventional
modelling."* Our ankle wind-up was not a side quest; it is the phenomenon ASAP exists
to correct on this robot.

**(b) They never verify the other 19 DoF are fine.** Reason (1) for narrowing is
explicitly a *data budget*, not a measurement. The physical argument is sound — serial
revolute joints are well modelled, a parallel linkage is a structural modelling error —
but "expected" is not "measured".

**The cheap experiment that settles it:** replay the recorded actions through the
simulator and compare per-joint trajectories against the real ones. Where the tracking
error concentrates *is* the sim-to-real gap profile. It costs no robot time — the data
already exists — and it turns "we followed the paper" into "we verified the paper's
assumption on our hardware."

---

## 7. Known limitations

- **All 473 repetitions are the same motion, from broadly the same place.** Quantity is
  ample; **coverage is the limitation.** A model fitted here will be accurate on this
  trajectory and unproven away from it.
- **The calibration's yaw term is not reusable across sessions.** Roll and pitch are
  gravity-referenced and stable (±3° between sessions), but yaw moved **26.5° between
  sessions and 5.7° within one 65-minute run**. That is IMU yaw drift, not marker
  movement — Vicon's world frame is fixed and is the trustworthy heading source.
  **Solve yaw per session.**
- Base velocities are finite-differenced from interpolated pose, so noisier than
  position.
- Session 2 is noisier per frame than session 1 (2.52 mm vs 0.40 mm median residual).
- Only 9 of 20 mocap cameras work, limiting the usable capture volume.
- `rt/lowstate` publishes at **1000 Hz** but is sampled at the policy step (48.9 Hz);
  the dataset is at policy rate.

---

## 8. Next steps

**Immediately, no robot time needed**
1. **Fit the delta action model** on the 473 repetitions.
2. **Plot a learning curve** — fit at 100 / 200 / 300 / 473 clips and watch validation
   error. If it plateaus at 200, more of the same motion is wasted robot time and you
   can say so with evidence. This answers "is 473 enough?" far better than any number
   from the paper.
3. **Measure the per-joint sim-to-real error** (§6) to decide 4-DoF vs 23-DoF honestly.

**Next capture sessions**
4. **Add motions, not more dabs.** Four more policies are already deployed and keyed
   into the FSM: `robomimic/charleston`, `whole_body_tracking/dance_102`,
   `whole_body_tracking/gangnam_style`, `robomimic/locomotion`. Roughly 80 clips each
   would match the paper's breadth. Expect to debug each policy's own deployment quirks
   (the dab had the ankle wind-up), and watch motor temperatures — the dances are more
   dynamic than the dab.
5. **Vary conditions within a motion** — position and heading in the volume, action
   scale at 80–90 %, small perturbations. Otherwise the learning curve flattens because
   the data stopped containing anything new, not because the model learned enough.

**Open question for the training side**
6. Does the reference dab end heading-neutral? The robot yaws slightly per repetition;
   if the reference is neutral, a symmetry or yaw-drift term is worth adding to the
   fine-tune.

---

## 9. Running a session

**Pre-flight** — `~/qxie2/rl_sar/scripts/preflight.sh`
Verifies: no stale `rl_real_g1`, exactly one address on `eth0`, robot boards and Vicon
PC reachable, `rt/lowstate` publishing (i.e. robot in **development mode** — purple /
zero-torque is not enough), Vicon rows growing, disk free.

**Vicon PC** — nothing to run but Nexus. Confirm `ipconfig` shows `192.168.123.50`,
OSC enabled → `192.168.123.50` port 7000, subject + markers ticked, Live mode. If the
cameras have not moved, **verify** the calibration rather than re-wanding. If the
markers stayed on the robot, the subject and calibration carry over.

**Jetson** — two terminals:
```
cd ~/qxie2/rl_sar/scripts/vicon && python3 osc_listener.py
cd ~/qxie2/rl_sar && RL_RECORD=1 ./cmake_build/bin/rl_real_g1 eth0
```
Optional: `scripts/live_count.py` and `scripts/live_temps.py`.

**During** — `1` then `5` per repetition, one at a time. Change conditions every ~20.

**Stopping** — press **`0`** (passive), let it settle, **then** Ctrl+C. Abrupt Ctrl+C
mid-control faults the robot and forces a power cycle.

**After** — merge → clean → validate → publish:
```
merge_rollout_vicon.py --object pelvis --manual-offset 0 --max-rms 5.0
clean_dataset.py --no-recenter --cal-rpy <solved for THIS session>
validate.py
```

**Subject definition** (only if markers were re-mounted): origin `PELVRB`, primary
axis `PELVRF` (→ X forward), secondary axis `PELVLB` (→ Y left). Label left/right from
the **robot's** perspective — stand behind it facing the same way. Sanity check: if the
solved calibration comes out small (~5°) the labelling is right; near 180° yaw means
front/back are swapped.

---

## 10. Gotchas that cost time

- **Timezone.** The Jetson runs `Asia/Shanghai` (UTC+8), so filenames read ~12 hours
  ahead of Buffalo local time. `dab_clean_20260802` was recorded on **2 Aug local**;
  the 29 July session is named `20260730`. `t_wall` is UTC epoch and is unambiguous.
- **A stale `rl_real_g1`** makes a fresh instance appear to hang at the loop-start
  lines. `run_g1.sh` now refuses to launch in that state.
- **The robot leaves development mode** when rl_sar stops; re-enter it each run.
- **Never put two subnets on `eth0`** — see §3.
- `/tmp` gets cleaned; keep artefacts in `reports/`.
- `scipy` on this Jetson is broken against the installed numpy (`np.typeDict`). Use
  numpy directly.
