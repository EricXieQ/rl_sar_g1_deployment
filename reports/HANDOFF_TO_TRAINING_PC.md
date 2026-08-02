# Handoff to the training PC

**From:** the robot Jetson (deployment side)
**To:** whoever fits the delta action model — has the `ASAP/` repo and the paper
**Date:** 2 August 2026. Supersedes the earlier `question_for_training_pc.md`.

---

## TL;DR

**473 real dab repetitions are collected, validated and published.** That is above the
400-clip minimum the paper cites for the full 23-DoF delta action model. Nothing is
blocked on the robot any more — the next steps are all yours, and three of them need
no robot time at all.

```bash
curl -LO https://github.com/EricXieQ/rl_sar_g1_deployment/releases/download/dataset-473reps/asap_dab_dataset_473reps.tgz
tar xzf asap_dab_dataset_473reps.tgz
```

---

## 1. What you are getting

| file | reps | rows |
|---|---|---|
| `dab_clean_20260730.csv` | 69 | 20,556 |
| `dab_clean_20260802.csv` | 404 | 120,265 |
| **total** | **473** | **140,821** (140,348 transitions) |

One row per policy step at 48.9 Hz. 196 columns: raw policy `action`, commanded
`target_dof_pos`, measured `dof_pos/vel/tau_est` for 29 joints, IMU, and Vicon pelvis
pose — both raw and rotated into the URDF frame.

Single skill: `wall_background_dab`. Full schema, loading snippet and caveats are in
the README attached to that release.

## 2. Quality, and how it was verified

| | 29 Jul | 2 Aug |
|---|---|---|
| Genuine orientation flips | 0 | 0 |
| Genuine mislabelled frames | 0 | 0 |
| Vicon coverage of policy steps | 100 % | 100 % |
| Rigid-fit residual (median) | **0.40 mm** | 2.52 mm |
| Reps dropped (short / dead / uncovered) | 0/0/0 | 0/0/0 |

Verified against the robot's **own IMU** — an independent sensor that shares no
physical path with the cameras, so agreement is real evidence rather than
self-consistency:

- angular velocity correlates **0.924** (pitch) and **0.936** (yaw)
- gravity, measured two independent ways: **9.727 vs 9.719 m/s²**
- time alignment peaks at **0 ms lag**
- 69 repetitions overlay to **1.4 mm** median spread

Reproduce with `reports/validate.py`.

**Not verified:** dynamic linear acceleration. Double-differentiating position at
50 Hz is noise-dominated, so position is validated only indirectly (gravity,
repeatability). Worth knowing before you trust base acceleration.

## 3. Two things that will bite you if you miss them

**(a) The sessions do not share a calibration — because of IMU yaw drift.**

| | 29 Jul | 2 Aug | change |
|---|---|---|---|
| roll | +4.51° | +1.22° | −3.29° |
| pitch | +5.99° | +4.61° | −1.38° |
| **yaw** | **−3.71°** | **+22.82°** | **+26.53°** |

Roll and pitch are gravity-referenced and stable. Yaw moved 26.5° between sessions and
5.7° *within* the 2 Aug run. The IMU has no absolute heading reference; Vicon's world
frame does not drift.

The `base_quat_urdf_*` columns are already corrected **per session**, so they are
internally consistent — but **absolute heading is not comparable across sessions**.
Either work per session, or re-reference each repetition to its own starting heading.
Naively concatenating and training on world-frame yaw teaches the model a 26°
discontinuity that never physically happened.

**(b) Coverage, not count, is the limitation.** All 473 repetitions are the same motion
from broadly the same place. The quantity is ample; the variety is not. A model fitted
here will be accurate on this trajectory and unproven away from it.

## 4. What we still need from you

**(i) The expected data format — this is the one thing still blocking us.**
What exactly does the delta-model trainer read? Field names, units, frames (world vs
body), and whether base velocity is expected as a provided column or differentiated
internally. We have every quantity; we just do not know your layout. Tell us and we
will export to match.

**(ii) Does the reference `wall_background_dab` end heading-neutral?**
The robot yaws slightly per repetition. If the reference motion is heading-neutral,
that drift is a tracking artefact and a symmetry or yaw-drift penalty is worth adding
to the fine-tune. If the reference itself commands yaw, it is expected and we should
leave it alone.

**(iii) 4-DoF or 23-DoF?** See §5 — we think this should be decided by measurement
rather than inherited.

## 5. The experiment worth running first (no robot time)

The paper narrows the delta model from 23 DoF to **4 DoF (the ankles)**, justified by
two things: *(1)* limited real data made 23-DoF infeasible, and *(2)* the G1's ankle
uses a mechanical linkage that simulation models badly.

Reason (1) is explicitly a **data budget** — and that constraint no longer applies to
us. Reason (2) is a physical argument, not a measurement: **they never verify that the
other 19 DoF are actually fine.**

**Replay the recorded actions through the simulator and compare per-joint trajectories
against the real ones.** Same commanded actions, same initial state. Where the tracking
error concentrates *is* the sim-to-real gap profile.

- error concentrated in the ankles → the 4-DoF model is justified, *and now verified on
  our hardware* rather than assumed
- error spread wider → the ankle-only model leaves real error uncorrected, and we have
  the data for the full 23-DoF version

Either outcome is a result. It costs no robot time — the data already exists.

## 6. And then: a learning curve, not a target number

Rather than trusting "400 clips", fit the model at 100 / 200 / 300 / 473 repetitions
and plot validation error.

- still dropping at 473 → more data genuinely helps, and we will collect more
- plateaued at 200 → more of *the same motion* is wasted robot time, and we should
  spend the next sessions on **other motions** instead

This answers "is 473 enough?" for our hardware and our motion, empirically. It is also
a much better slide than quoting someone else's number.

## 7. If more data is wanted

Four more policies are already deployed and keyed into the FSM on the robot:
`robomimic/charleston`, `whole_body_tracking/dance_102`,
`whole_body_tracking/gangnam_style`, `robomimic/locomotion`.

Roughly 80 clips each would match the paper's breadth (they used 5 motions). We can
collect ~400 repetitions in a 65-minute session, so this is a handful of sessions, not
months. Tell us which motions would most help the model and we will prioritise those.

We also have **~23 minutes of locomotion** already recorded (the paper collects 10) —
retained in the `merged_*.csv` files rather than the cleaned datasets. Say the word if
you want it exported.

## 8. Context

- Full project state, the three hardware/software faults that had to be fixed first,
  the pipeline design, and the session runbook: **`reports/PROJECT_STATUS.md`**
- Capture tooling: `scripts/vicon/` — `osc_listener.py`, `merge_rollout_vicon.py`,
  `clean_dataset.py`, plus `reports/validate.py` and `reports/dataset_stats.py`
- Repository: github.com/EricXieQ/rl_sar_g1_deployment, branch `jetson-deploy`
