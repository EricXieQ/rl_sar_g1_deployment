# Hardware session, September 2026: v6 against the repaired stage (c) controls

**Question.** Does the repaired stage (c) recipe (ASAP branch
`fix/stagec-repair-2026-09`: v6's recipe restored, seeded, stay-near-v6 penalty, no
delta) change anything on the real G1 compared with v6, for better or worse, scored the
way the simulator was: feet planted, heading, height, torso roll and lateral shift,
ankle push, not merely "did it fall".

**Policies**, in this order, all under `policy/g1/`:

| Order | `RL_DAB_POLICY` | What | MuJoCo, ten handoffs |
|---|---|---|---|
| 1 | `asap_dab` | v6, the validated reference (473 recorded reps) | planted 10/10, heading +1.3 deg, roll -3.0 deg, push +1 / +4 deg |
| 2 | `asap_dab_v6recipe_s29` | delta-off control, seed 29 (never on hardware) | planted 10/10, heading +4.5, roll -2.4, push +10 / +9 |
| 3 | `asap_dab_v6recipe_s17` | delta-off control, seed 17 (never on hardware) | planted 8/10 (two 80 ms unloadings under 1 cm), heading +3.7, roll -4.4, push +3 / +3 |

Do not use `asap_dab_ft`, `asap_dab_ftdr` (both fall in MuJoCo) or `asap_dab_w01`
(steps and turns). Each folder's README has the checkpoint path, sha256 and full scores.

## Before the robot

1. Put the two new folders on the Jetson checkout (`~/qxie2/rl_sar`): pull this branch,
   or `scp -r policy/g1/asap_dab_v6recipe_s29 policy/g1/asap_dab_v6recipe_s17
   unitree@<jetson>:~/qxie2/rl_sar/policy/g1/`. No rebuild: the FSM reads
   `RL_DAB_POLICY` at runtime and the binary preloads every `policy/g1/*/config.yaml`
   it finds; look for `[PRELOAD] cached g1/asap_dab_v6recipe_s29` at startup.
2. `scripts/preflight.sh` (no stale `rl_real_g1`, one address on eth0, robot in
   development mode, Vicon PC reachable if you are capturing).
3. Session folder on the Jetson: `mkdir -p ~/qxie2/rl_sar/logs/hw_2026-09/{asap_dab,asap_dab_v6recipe_s29,asap_dab_v6recipe_s17}`
   and a `notes.md` in it for the rep order and anything odd.
4. Vicon is optional but is the only source of height and lateral shift. If the cameras
   and markers are unchanged, verify the calibration rather than re-wanding, and run
   `scripts/vicon/osc_listener.py` in its own terminal as in
   `reports/PROJECT_STATUS.md` section 9.

## Repetitions and order

At least 10 per policy, interleaved. `RL_DAB_POLICY` is read from the process
environment, so changing policy means relaunching `rl_real_g1`; interleave in blocks of
five to bound the relaunches:

    block 1: asap_dab x5      block 2: v6recipe_s29 x5      block 3: v6recipe_s17 x5
    block 4: asap_dab x5      block 5: v6recipe_s29 x5      block 6: v6recipe_s17 x5

Thirty reps, six launches, about the pace of the August sessions (10 s per rep plus
the relaunch). If time allows, a seventh and eighth block (v6, then whichever control
looked closer) sharpen the medians. Keep the same floor spot and heading for every rep;
the capture sessions showed v6's heading drifts by a couple of degrees per rep, which
is inside the noise you are trying to see through.

## Per block

```bash
cd ~/qxie2/rl_sar
RL_RECORD=1 RL_DAB_POLICY=asap_dab_v6recipe_s29 ./run_g1.sh eth0 /tmp/g1_s29_block2.log
```
`RL_RECORD=1` turns on the rollout recorder (`logs/rollout_<date>_<time>.csv`, one
row per policy tick of every RL state, with the policy in the `state` column). Confirm
the console shows `[RECORD] rollout -> ...` at the first policy tick and, on the first dab,
`[WARNING] [DAB] policy overridden by RL_DAB_POLICY -> asap_dab_v6recipe_s29`.
For the v6 blocks leave `RL_DAB_POLICY` unset (it defaults to `asap_dab`).

Keys, as always: `0` stand up, `1` locomotion, let it settle 4 to 8 s standing in place,
`5` dab. The dab runs 5.94 s of policy time (6.09 s on the wall) and hands itself back
to locomotion. Then, after each rep, from a second terminal:

```bash
cp /tmp/rl_sar_dab_log.csv ~/qxie2/rl_sar/logs/hw_2026-09/asap_dab_v6recipe_s29/rep$(printf %02d N).csv
```
(`/tmp/rl_sar_dab_log.csv` is the FSM's per-tick diagnostic, joint q, target and kp;
it is overwritten by the next rep and `/tmp` is cleaned, so copy it every time.)
Number reps across the whole session per policy (block 2 writes rep01 to rep05, block 5
rep06 to rep10). End the block the way you end every run: locomotion settled, get down
(`9`) or passive (`p`) on the hoist, then Ctrl+C; never Ctrl+C mid-control. Then move
the block's rollout file next to its diag files:

```bash
mv ~/qxie2/rl_sar/logs/rollout_*.csv ~/qxie2/rl_sar/logs/hw_2026-09/asap_dab_v6recipe_s29/
```

## What to watch live

- The first rep of each new policy (`s29` in block 2, `s17` in block 3) is its first
  time on hardware: hoist line taut, spotter ready, and read the console's
  `[ASAPDab DIAG] === First-tick jolt analysis ===` table; a largest delta well above
  v6's usual entry step is a reason to stop and look before rep 2.
- `[ASAPDab] Fall detected (roll=..., pitch=...)` means the FSM's 30 degree rule fired
  and it handed control to locomotion, which usually recovers. Log it in `notes.md`;
  the scoring counts it as a fall. Two aborts of one policy end its block.
- Feet: a foot that lifts, shuffles or slides during the hold (3 to 4.4 s into the
  dab) is exactly what the simulator saw in the failed arms. Note the rep number and
  which foot. The scoring's kinematic proxy sees clear lifts, not slides or unloading.
- The right ankle roll (index 11): the July wind-up (command far over 15 degrees,
  about 30 N m, firmware power cut) has not recurred with v6 at full action scale, but
  the new policies push their ankles harder in MuJoCo (s29: +10 / +9 degrees); if
  `live_temps.py` shows an ankle climbing or the console prints a power-cut, stop.
- Loop timer: ticks of about 20.5 ms (48.8 Hz, 2.5 percent slow) are the known
  `loop.hpp` behaviour and are expected and unchanged. The 473 reference reps were
  recorded on the same clock, so do not fix it before this session; a dab taking 6.09 s
  on the wall is normal.
- Motor temperatures (`scripts/live_temps.py`) between blocks; the dab is gentle but
  thirty reps with relaunches take an hour.

## Scoring

The script needs only numpy and pandas (`/home/eric/miniconda3/envs/hvgym/bin/python`
has both on the desktop; on the Jetson use the system python3 that runs the vicon
scripts). Copy `logs/hw_2026-09/` to the desktop and run, from the rl_sar root:

```bash
# robot only: rollouts (IMU heading, roll, push, kinematic feet proxy) and diag files (push)
python3 scripts/score_dab_session.py logs/hw_2026-09/*/rollout_*.csv logs/hw_2026-09/*/rep*.csv

# with the simulator's ten-handoff batches beside the robot rows
python3 scripts/score_dab_session.py logs/hw_2026-09/*/rollout_*.csv \
    --sim /home/eric/Project/humanoid/firstmate/data/asap-stagec-repair-g1/logs/s29_t* \
          /home/eric/Project/humanoid/firstmate/data/asap-stagec-repair-g1/logs/s17_t*

# per-repetition rows to a CSV for plotting, per-policy table only on screen
python3 scripts/score_dab_session.py logs/hw_2026-09/*/rollout_*.csv --no-reps --csv hw_2026-09_reps.csv
```
If Vicon was captured, merge and clean as in `PROJECT_STATUS.md` section 9
(`merge_rollout_vicon.py`, then `clean_dataset.py` with this session's calibration)
and score the cleaned file too; it adds the height and lateral-shift columns, which
are n/a from the rollout alone. Heights from Vicon are marker-origin heights (v6 read
0.68 m in both captures), not the simulator's 0.75 m; compare policies within the
session.

The per-policy table gives, per policy and source: n, feet planted (no lift event of
1 cm for 80 ms after 0.5 s), max foot rise (median cm), turns over 15 degrees, falls,
aborted, absolute endpoint heading (median, signed median), peak roll in the first
2.2 s, lateral shift, minimum height, ankle push over the first raise (degrees and
N m), and the ankle-quiet count from tau_est (context only; v6 shows it in about
half its reps because the ankle torque passes through zero under load). Read the
script's docstring for the definitions and what each robot column can and cannot see.

For scale, v6 on the robot in the two August captures (69 and 404 reps): planted 69/69
and 402/404, max foot rise 0.2 to 0.3 cm, absolute heading 2.1 and 2.5 degrees, peak
roll 1.7 and 1.6 degrees, lateral 1.5 and 2.3 cm, push +17.6 / +19.1 and
+13.5 / +13.3 degrees. The push moved by 4 degrees between sessions and the lateral by
0.8 cm, which sets the margins below.

## Decision rule

Compare each control with v6 **from the same session** (v6's block medians, not the
simulator's numbers and not the August captures). Margins, from v6's own session-to-
session spread: heading 1.5 degrees, peak roll 1.5 degrees, lateral 1.5 cm, height
1 cm, push 5 degrees, feet planted within 1 rep of v6's count over the same number of
reps.

- **The repaired recipe helps** if a control has no fall or abort, its feet-planted
  count and max foot rise are no worse than v6's, and at least one of heading, peak
  roll, lateral shift or push is better than v6 by more than its margin with nothing
  worse by more than its margin. It is a recipe effect, not a seed effect, only if both
  controls move the same way; one seed alone is a seed result.
- **No difference** if every column is inside its margin of v6 and there is no fall,
  abort or turn over 15 degrees. Ten more reps per policy will not change that
  picture unless the differences are consistent in sign across both controls; then
  they are worth the extra block.
- **Worse than v6** if a control falls or aborts even once, turns over 15 degrees, has
  two or more lift events over ten reps beyond v6's count, or has heading, peak roll,
  lateral shift or push beyond v6's by more than its margin. A larger push than v6's
  by more than 5 degrees on its own is the simulator's warning sign (the policy leaning
  on its ankles) and counts as worse even with every other column inside its margin.

Whatever the outcome, the simulator answered "planted feet, heading within 5 degrees,
height like v6" for both controls; the robot answers whether the recipe's residuals
(heading 2 to 3 degrees further, larger push for seed 29, two brief unloadings for seed
17) are real on hardware. Write the per-policy table into `notes.md` with the verdict.
