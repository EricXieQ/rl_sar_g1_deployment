# asap_dab_v6recipe_s29

The stage (c) delta-off control of the repaired ASAP recipe, training seed 29.
NOT YET RUN ON HARDWARE. The validated real-robot dab policy is `../asap_dab`
(v6); this one is a candidate for a hardware comparison against it, first in
the order (then `../asap_dab_v6recipe_s17`).

## What it is

v6 (`DabTracking_grounded_v6/model_20000.pt`) fine-tuned for 3000 iterations in
IsaacGym with the repaired stage (c) recipe of ASAP branch
`fix/stagec-repair-2026-09` (`finetune_dab_stagec_v6_recipe.sh off 29`):
v6's own recipe restored (human wall-dab reference, locomotion-stance entries,
spawn noise, v6 domain randomization, low-height termination, v6 learning
rates), seeding re-enabled, the stay-near-v6 penalty
`penalty_ref_action_deviation` at weight 0.5, and no delta model at all (the
control arm, "R1"). It therefore answers a narrower question than a delta arm:
does the repaired fine-tuning recipe on its own change anything on the robot.
See `docs/stagec_repair_2026-09.md` and `docs/stagec_lean_fix_2026-09.md` on
that ASAP branch for the full record.

## Where the checkpoint came from

```
/home/eric/Project/humanoid/firstmate/data/asap-stagec-repair-g1/checkpoints/s29_R1/model_3000.pt
sha256 4c37b187c010df485cbf951075a1701d19bdebb3228478fe4c91845e346328ac
```
The training console (`training/s29_R1/console.log` of that laboratory)
records `Setting seed: 29`, `no delta_checkpoint given` and
`penalty_ref_action_deviation = -0.5`. The observation layout is v6's
(380-dim ASAP actor, 23 joints), so the same TorchScript wrapper applies.

`policy.pt` (sha256 c2328aac11ca4bc609c69be714c53a7ebef62fc7715721e98b62195462c74a7a)
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
| Feet planted (no support loss of 80 ms or more after 0.5 s) | 10/10 | 10/10 |
| Endpoint heading, median (range), deg | +1.3 (0.2 to 4.4) | +4.5 (3.2 to 8.8) |
| Turns over 15 deg / fall alarms | 0 / 0 | 0 / 0 |
| Minimum body height, median m | 0.752 | 0.755 |
| Peak pelvis roll in the first 2.2 s, median deg | -3.0 | -2.4 |
| Peak lateral shift in the first 2.2 s, median cm | -3.2 | -2.7 |
| Ankle push over the first raise, L / R, median deg | +1.0 / +4.4 | +10.3 / +8.6 |
| Meets the five-part bar | yes | yes |

IsaacGym, delta off, 256 episodes from the locomotion stance: 0 steps, 0
turns, absolute heading median 1.7 deg (v6: 1.5). Its residual against v6 is
a heading 2 to 3 deg further and a larger ankle push.

## How to run

```bash
RL_DAB_POLICY=asap_dab_v6recipe_s29 ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof_hoist2
RL_RECORD=1 RL_DAB_POLICY=asap_dab_v6recipe_s29 ./cmake_build/bin/rl_real_g1 eth0
```
Keys as for `../asap_dab`: `0` stand up, `1` locomotion, `5` dab, `p`
passive. Score the logs with `scripts/score_dab_session.py`; the session
plan is `reports/HW_SESSION_2026-09.md`.
