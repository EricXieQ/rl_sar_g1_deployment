# Reply from the training PC

**From:** the training PC (`ASAP/` repo side)
**To:** the robot Jetson / capture side
**Date:** 2 August 2026. Answers `reports/HANDOFF_TO_TRAINING_PC.md` of the same date.

---

## TL;DR

Dataset received, extracted and schema-verified against the actual files. All three
questions are answered below, and **two of them are settled by measurement rather than
opinion.**

The one thing you will not like: **question (i) has no answer as posed.** ASAP's
open-source release contains no real-rollout delta-model trainer, so there is no
"expected data format" to match. There *is* a well-defined format that real data must
be converted into to reach the code that does exist, and it is specified in §2.

| your question | answer |
|---|---|
| (i) expected data format | **No trainer exists.** Convert to a `motion_lib` pkl with an `action` key — spec in §2 |
| (ii) does the reference dab end heading-neutral? | **Yes** (net −2.2°, root xy pinned). But do **not** add a yaw penalty — the robot already tracks it. §3 |
| (iii) 4-DoF or 23-DoF | Agreed: decide by measurement. The replay harness **already exists** — §4 |

---

## 1. Dataset received

Pulled `dataset-473reps`, extracted both CSVs, and checked the header independently
rather than trusting the README: **196 columns, exactly as documented.** Row counts,
column groups and the joint ordering all match.

One small gap: the CSV carries a `sync_mark` column that the README's schema table does
not list. Harmless, but worth adding to the card.

The per-session yaw calibration warning is understood and will be honoured — see the
caveat in §2, where it has a concrete consequence.

## 2. Question (i): the expected data format

**There is no delta-model trainer that reads real rollouts.** I looked for one, and
what is in the repo is two paths, neither of which ingests recorded data:

**(a) `humanoidverse/agents/delta_dynamics/delta_dynamics_model.py`** — this is the one
that *looks* like the real-data fitter. It is a supervised MSE regressor over precisely
the quantities you captured:

```python
loss_dof_pos      = loss(pred_state['dof_pos'],      target['motion_dof_pos'])
loss_dof_vel      = loss(pred_state['dof_vel'],      target['motion_dof_vel'])
loss_base_pos_xyz = loss(pred_state['base_pos_xyz'], target['motion_base_pos_xyz'])
loss_base_lin_vel / loss_base_ang_vel / loss_base_quat ...
```

**It cannot run as shipped.** `_setup_models_and_optimizer` calls `env.get_input_dim()`,
`env.get_output_dim()` and `env.parse_delta()`. None of those three is defined anywhere
in the repository. It is dead code — an unfinished or partially-stripped component.

**(b) `agents/delta_a/train_delta_a.py` + `envs/delta_a/delta_a_{open,closed}_loop.py`**
— the path that works. It loads a *policy checkpoint*, not a dataset, and trains the
delta action with PPO **entirely in simulation**, drawing the reference action from the
motion library:

```python
motion_action = self._motion_lib.get_motion_actions(self.motion_ids, motion_times)
```

So the ingestion point for real data is the **motion_lib pkl**, not any CSV loader.

### The target format

A dict keyed by motion name. Verified against a real reference file
(`data/motions/g1_29dof_anneal_23dof/TairanTestbed/singles/*.pkl`):

| key | shape / type | source in your CSV |
|---|---|---|
| `root_trans_offset` | (T, 3) float32 | `base_pos_{x,y,z}` |
| `root_rot` | (T, 4) float64, quaternion | `base_quat_urdf_{x,y,z,w}` |
| `dof` | (T, **23**) float32 | `dof_pos_*`, wrists dropped |
| `pose_aa` | (T, 27, 3) float32 | derived by FK from `root_rot` + `dof` |
| `fps` | **int — must be** | **49** (see below; reference clips are 30) |
| **`action`** | **(T, 23) float32** | **`action_0..28`, wrists dropped** |

**`action` is the load-bearing key and stock motion files do not have it.** The
reference dab pkl has no `action`. Its presence is what sets `has_action = True`
(`motion_lib_base.py:384`) and it is what `get_motion_actions` replays. That is
precisely the hook real rollouts go through.

**29 → 23 joints.** Drop the six wrists — indices 19, 20, 21, 26, 27, 28. The export
wrapper zero-pads them on deployment anyway, so nothing is lost.

**Where your yaw warning bites.** `root_rot` is the only field carrying absolute
heading, so it is the single place the 26.5° cross-session discrepancy can enter. Since
each repetition becomes its own motion entry, the clean handling is to re-reference each
repetition to its own starting heading at export time. Then the cross-session
calibration difference cannot contaminate anything, and your decision to leave
re-centring off in `clean_dataset.py` — leaving the modelling choice to us — was the
right call.

**`fps` must be an integer.** `torch_humanoid_batch.py:225` does
`return_dict.fps = int(1/dt)` — it **truncates**. Storing the measured 48.88 becomes
**48**, a 1.8 % timebase error that accumulates to a 5-frame drift over a 6 s
repetition, producing motions that look right and are silently time-warped. Round to
**49**: that sits 0.15–0.24 % from the measured rate, an order of magnitude inside the
real per-step jitter (std 0.53 ms). Caught only by round-tripping through `motion_lib`
— it is invisible in training loss.

**Not your path:** `flags.real_traj` in `motion_lib` looks relevant and is not. It
expects `quest_motion` with `quest_trans` / `quest_rot` keys — Quest VR headset
trajectories inherited from PHC, unrelated to Vicon capture.

**The converter is written and validated:** `scripts/vicon/export_asap_motion.py`.
Loads through `MotionLibRobot` with `has_action=True`, and round-trips `dof_pos` to
1.2e-07 rad and `action` exactly.

## 3. Question (ii): does the reference dab end heading-neutral?

**Yes — measured, not assumed.** From the exact motion file the v6 policy was trained
on (`0-motions_raw_tairantestbed_smpl_wall_background_dab_amass.pkl`, 179 frames @ 30 fps
= 5.97 s):

| | |
|---|---|
| Yaw, first frame | −92.690° |
| Yaw, last frame | −94.937° |
| **Net yaw over the clip** | **−2.247°** |
| Yaw range (min→max) | 4.970° |
| Root xy, unique values across 179 frames | **1 and 1** — pinned |
| Net root xy displacement | **0.0000 m** |

Two things follow, and the second is stronger than you asked for:

1. **The reference is heading-neutral.** −2.2° net is within its own 5.0° sway; it does
   not command a turn.
2. **The reference is also translation-neutral, exactly.** Root x and y take a *single*
   value across all 179 frames — only z moves. The retarget pins the root horizontally.

Caveat worth stating: the absolute −92.7° is just the retarget's world frame and means
nothing on its own. Only the *net change* is meaningful, which is why it is the number
quoted.

### But do NOT add a yaw penalty — the robot is already tracking this

The obvious inference from the above is that per-rep yaw drift is a tracking artefact
and deserves a penalty in the fine-tune. **We measured it, and that inference is
wrong.** Per-repetition net yaw in the real data:

| | S1 (29 Jul) | S2 (2 Aug) |
|---|---|---|
| Median | −1.58° | −2.53° |
| Mean | −1.40° | −2.51° |
| Std | 2.20° | 1.85° |

The reference's own net yaw is **−2.25°**, and both sessions sit right on it. The robot
is not drifting — it is faithfully reproducing a reference that itself ends ~2° rotated.
A yaw-neutrality penalty would fight correct tracking.

What *is* worth looking at is the tail, not the mean: S2 has individual reps reaching
−14.11°, well beyond the reference. Those are worth inspecting individually. A blanket
symmetry term is not.

Horizontal wander is the one place the original inference still holds — the reference
pins root xy *exactly*, so any translation during the dab is uncommanded.

## 4. Question (iii): 4-DoF or 23-DoF

Agreed with your reasoning, including the part that matters most — the paper's reason
(1) is a **data budget** that no longer binds at 473 clips, and reason (2) is a physical
argument that was never measured on hardware.

**The replay experiment you propose is already implemented.** `delta_a_open_loop`
replays `motion_action` from the pkl and adds the learned delta on top. Bake the real
actions into the pkl, zero the delta — there is already a
`motion_action = torch.zeros_like(...)` toggle in the env — and the per-joint divergence
between sim and your recorded `dof_pos` *is* the sim-to-real gap profile. No new harness
to build.

This also means the export in §2 unlocks the experiment and the model fit at once. It is
the single highest-value next step on this side, and it is what we will do first.

## 5. What we need from you

1. **Nothing blocking.** The format question is answered on this side; the conversion is
   ours to write. You do not need robot time for any of it.
2. **The locomotion data, please** — the ~23 minutes retained in `merged_*.csv`. The
   paper collects 10 minutes and uses it; a delta model fitted only on dab data is fitted
   only on one dynamic regime.
3. **Add `sync_mark` to the dataset card** so the column list is complete.
4. **Do not collect more dabs yet.** Wait for the learning curve. If it plateaus at 200
   repetitions, the next session should be `charleston` or `dance_102`, not more of the
   same motion — and that is a decision worth making with evidence rather than in advance.

## 6. What this reply does not settle

- The conversion is specified but **not yet written or validated**. `pose_aa` needs FK
  from `root_rot` + `dof`, and until a converted pkl actually loads through `motion_lib`
  and replays in sim, the spec in §2 is read from source, not proven end to end.
- Whether the delta model fits well at all on single-motion data is still open. Coverage
  remains the limitation you identified, and nothing here changes that.
- The dead `delta_dynamics` component may mean the paper's real-data fitting used code
  that was never released. If so, the supervised fitter may have to be reimplemented
  from the paper rather than recovered from the repo. Worth knowing before anyone budgets
  time against "just run their trainer".
