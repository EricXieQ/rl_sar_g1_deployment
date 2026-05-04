# ASAP Wall-Dab Policy in rl_sar

## What this is

An ASAP-trained motion tracking policy ported into the rl_sar deployment
framework. The policy was trained in IsaacGym via the ASAP codebase
(`logs/DabTracking/.../model_93600.pt`) on the wall_background_dab motion.
This directory contains everything needed to run that same policy through
rl_sar's standard sim2sim (MuJoCo) and sim2real (real G1) pipelines.

## How the integration works

ASAP and rl_sar use **incompatible observation layouts**, even though the
underlying physical quantities are the same.

ASAP's actor expects a 380-dim input in this exact order (alphabetical sort
of obs keys, with `history_actor` placed between `dof_vel` and
`projected_gravity`):
```
[actions(23), base_ang_vel(3), dof_pos(23), dof_vel(23),
 history_actor(304),     # 4 frames of all 6 components, per-key contiguous
 projected_gravity(3), ref_motion_phase(1)]
```

rl_sar (with `observations_history: [0,1,2,3]` and `term` priority) feeds
the policy a flat 376-dim history blob:
```
[actions_t..t-3 (4*29=116), ang_vel_t..t-3 (4*3=12),
 dof_pos_t..t-3 (4*29=116), dof_vel_t..t-3 (4*29=116),
 gravity_t..t-3 (4*3=12), phase_t..t-3 (4*1=4)]
```

Two key differences:
1. rl_sar uses **all 29 DOFs** of the G1; ASAP only controls 23 (no wrists).
2. rl_sar feeds the model **only** the history blob; ASAP expects the
   current frame as a separate prefix in front of the history.

The fix is `policy.pt` — a TorchScript wrapper around the ASAP actor that
internally:
1. Slices each component out of rl_sar's 376-dim term-priority input
2. Drops the 6 wrist DOFs from `actions`, `dof_pos`, `dof_vel`
3. Builds ASAP's 380-dim layout
4. Calls the original 380→23 actor
5. Pads the 23-dim action back to 29 DOFs (zero on wrist joints)

This was verified bit-exact against the original ONNX export
(max diff < 1e-6 across random inputs).

## Files in this directory

| File              | Purpose |
|-------------------|---------|
| `export_wrapper.py` | Re-creates `policy.pt` from the ASAP `.pt` checkpoint. Run with the `hvgym` conda env. |
| `policy.pt`       | TorchScript wrapper used by rl_sar |
| `config.yaml`     | rl_sar config: obs schema, joint mapping, PD gains, action scale |
| `README.md`       | This file |

## Phase signal

The ASAP policy uses `ref_motion_phase` (a scalar 0..1 advancing over the
motion duration). rl_sar's `RoboMimic_Deploy/phase` computes
`episode_length_buf * dt * decimation / motion_length`. The hardcoded
`motion_length = 5.93f` is set in `RLFSMStateRLASAPDab::Enter()` in
`src/rl_sar/fsm_robot/fsm_g1.hpp`. 5.93s is the duration of the wall_dab
motion .pkl file.

There is no motion file loaded at runtime — the dab is implicit in the
trained weights. The policy only needs the scalar phase to know "where" it
is in the motion.

## How to run (after building rl_sar)

```bash
cd ~/Project/humanoid/rl_sar
./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
```

In the MuJoCo viewer / terminal:
1. Press `0` → robot interpolates to default standing pose (GetUp)
2. Press `1` → enter the standing locomotion policy
3. Press `5` → switch to ASAP wall-dab tracking policy
4. The robot performs the dab over 5.93s, then auto-returns to locomotion

For real robot deployment:
```bash
./cmake_build/bin/rl_real_g1 <eth_interface_name>
```
Same key sequence (0 → 1 → 5).

## Reproducing the wrapper

If you ever retrain the ASAP policy, regenerate `policy.pt` like this:

```bash
cd ~/Project/humanoid/ASAP
LD_LIBRARY_PATH=/home/eric/miniconda3/envs/hvgym/lib:$LD_LIBRARY_PATH \
  conda run -n hvgym python \
  /home/eric/Project/humanoid/rl_sar/policy/g1/asap_dab/export_wrapper.py
```

Edit `ckpt_path` inside `export_wrapper.py` if your checkpoint moved.
