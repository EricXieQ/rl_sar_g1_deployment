# asap_dab_v6recipe_s17

The stage (c) delta-off control of the repaired ASAP recipe, training seed 17.
NOT YET RUN ON HARDWARE. The validated real-robot dab policy is `../asap_dab`
(v6); this one is the second candidate for a hardware comparison, after
`../asap_dab_v6recipe_s29`, which met the MuJoCo bar in full where this one
missed it on two short foot unloadings.

## What it is

v6 (`DabTracking_grounded_v6/model_20000.pt`) fine-tuned for 3000 iterations in
IsaacGym with the repaired stage (c) recipe of ASAP branch
`fix/stagec-repair-2026-09` (`finetune_dab_stagec_v6_recipe.sh off 17`):
v6's own recipe restored (human wall-dab reference, locomotion-stance entries,
spawn noise, v6 domain randomization, low-height termination, v6 learning
rates), seeding re-enabled, the stay-near-v6 penalty
`penalty_ref_action_deviation` at weight 0.5, and no delta model at all (the
control arm, "R1"). Same recipe as the seed 29 policy, different seed. See
`docs/stagec_repair_2026-09.md` and `docs/stagec_lean_fix_2026-09.md` on that
ASAP branch for the full record.

## Where the checkpoint came from

```
/home/eric/Project/humanoid/firstmate/data/asap-stagec-repair-g1/checkpoints/s17_R1/model_3000.pt
sha256 b25870dfb95cf25b84461e7defe98283d4102e5c76368d94fe283cc6f4adfd5b
```
The training console (`training/s17_R1/console.log` of that laboratory)
records `Setting seed: 17`, `no delta_checkpoint given` and
`penalty_ref_action_deviation = -0.5`. The observation layout is v6's
(380-dim ASAP actor, 23 joints), so the same TorchScript wrapper applies.

`policy.pt` (sha256 d001d66fa2d7188c11406861a8224a691fdc3dd372b7829fe51e743df2ab3321)
was produced with `../asap_dab/export_wrapper.py --ckpt <that file> --out
policy.pt` and verified against the raw checkpoint weights on 256 random
observations (relative difference 1e-7, float32 rounding; wrist outputs zero)
and against the laboratory's own export (parameters identical, outputs
identical). `config.yaml` is `../asap_dab/config.yaml` with only the top-level
key renamed. One MuJoCo dab (`RL_DAB_POLICY` pointing here, `scene_29dof_hoist2`)
loaded the policy, ran the 5.94 s dab to its end and handed back to
locomotion without a fall alarm.

## MuJoCo scores (ten locomotion-to-dab handoffs, the harness of the repair)

| | v6 (same batch) | this policy |
|---|---:|---:|
| Feet planted (no support loss of 80 ms or more after 0.5 s) | 10/10 | 8/10 (two 80 ms left-foot unloadings under 1 cm) |
| Endpoint heading, median (range), deg | +1.4 (0.3 to 3.0) | +3.7 (1.4 to 9.1) |
| Turns over 15 deg / fall alarms | 0 / 0 | 0 / 0 |
| Minimum body height, median m | 0.754 | 0.750 |
| Peak pelvis roll in the first 2.2 s, median deg | -3.1 | -4.4 |
| Peak lateral shift in the first 2.2 s, median cm | -2.9 | +2.7 |
| Ankle push over the first raise, L / R, median deg | +1.8 / +5.6 | +2.7 / +3.4 |
| Meets the five-part bar | yes | no, on the feet line only (6/10 in the reset-fix batch re-scored with the lean rule) |

IsaacGym, delta off, 256 episodes from the locomotion stance: 0 steps, 0
turns, absolute heading median 1.4 deg (v6: 1.5). Its residual against v6 is
a short left-foot unloading at the border of the rule and a heading 2 to 3
deg further.

## How to run

```bash
RL_DAB_POLICY=asap_dab_v6recipe_s17 ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof_hoist2
RL_RECORD=1 RL_DAB_POLICY=asap_dab_v6recipe_s17 ./cmake_build/bin/rl_real_g1 eth0
```
Keys as for `../asap_dab`: `0` stand up, `1` locomotion, `5` dab, `p`
passive. Score the logs with `scripts/score_dab_session.py`; the session
plan is `reports/HW_SESSION_2026-09.md`.
