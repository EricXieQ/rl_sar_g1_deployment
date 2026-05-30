# Explaining the ASAP → rl_sar port

## In one sentence

> "I trained the motion-tracking policy using the **ASAP** training framework
> in IsaacGym, then ported the resulting checkpoint into **rl_sar**'s
> standard deployment framework so it can run in MuJoCo and on the real
> Unitree G1 through the same code path the rest of the lab uses."

## How rl_sar typically works

rl_sar is a generic C++ deployment framework for legged-robot RL policies.
The intended workflow is:

1. Train your policy in any framework (IsaacGym, IsaacLab, etc.)
2. Export the policy as **TorchScript** (`.pt` JIT) or ONNX
3. Drop it into `policy/<robot>/<config>/`
4. Write a YAML that describes:
   - Which observation components the policy expects (`ang_vel`, `dof_pos`, ...)
   - In what order
   - How many history frames
   - Per-joint PD gains, action scaling, joint mapping
5. Add an FSM state to switch into the new policy
6. Build with `./build.sh -mj` (CMake + MuJoCo) or `./build.sh -m` (real robot)

The same C++ binary then runs against MuJoCo (sim2sim) or the real Unitree
SDK (sim2real). All the messy pieces — DDS comms, joint state parsing,
torque commands, IMU reading, the FSM — are shared.

The deployment-time pipeline:
```
joint_state (DDS / MuJoCo)
   ↓
ComputeObservation()  ← reads YAML, builds the obs vector
   ↓
ObservationBuffer.insert / get_obs_vec  ← optional history stacking
   ↓
model.forward(obs)    ← LibTorch inference
   ↓
ComputeOutput()       ← action × action_scale + default_dof_pos → PD torque
   ↓
joint_command (DDS)
```

## Why ASAP doesn't drop straight into rl_sar

ASAP's training code has a quirk: when it concatenates the actor
observation, it sorts the obs keys **alphabetically**. This places
`history_actor` between `dof_vel` and `projected_gravity`, so the final
380-dim obs layout is:

```
[actions(23), base_ang_vel(3), dof_pos(23), dof_vel(23),
 history_actor(304), projected_gravity(3), ref_motion_phase(1)]
```

rl_sar, on the other hand, builds obs in **YAML config order**, and when
`observations_history` is set it feeds the model **only** the history
blob (4 frames stacked), not the current obs as a separate prefix.

Concretely, if I configure rl_sar with the obs list `[actions, ang_vel,
dof_pos, dof_vel, gravity_vec, RoboMimic_Deploy/phase]` and term-priority
history `[0,1,2,3]`, it builds a 376-dim vector laid out as:

```
[actions_t..t-3 (116), ang_vel_t..t-3 (12),
 dof_pos_t..t-3 (116), dof_vel_t..t-3 (116),
 gravity_t..t-3 (12), phase_t..t-3 (4)]
```

That term-priority history blob is **almost** what ASAP wants — its
internal `history_actor` is computed in the exact same per-key contiguous
way (we verified this in `observation_buffer.cpp` lines 153-177).

But the ASAP actor wants:
- The current frame as a 76-dim prefix in front (not just the 0th step
  inside the history blob), AND
- Only 23 DOFs, not 29 (ASAP's training excludes the wrist joints), AND
- Two of the obs components (`projected_gravity`, `ref_motion_phase`)
  to also appear once at the **end** of the input, after the history.

## What we did

Rather than modify rl_sar's C++ source to handle ASAP's weird layout, we
wrapped the ASAP actor in a TorchScript module that does the layout
conversion **inside the model**:

```
                  rl_sar provides 376-dim term-priority blob
                                  │
                                  ▼
            ┌────────────────────────────────────────┐
            │  TorchScript wrapper (policy.pt)        │
            │  1. slice each component out of 376    │
            │  2. drop wrist DOFs (29 → 23)          │
            │  3. assemble ASAP's 380-dim layout     │
            │  4. call original ASAP actor           │
            │  5. pad 23-dim action → 29 DOFs        │
            └────────────────────────────────────────┘
                                  │
                                  ▼
                  rl_sar receives 29-dim action vector
```

The wrapper is bit-exact against the original ONNX export across random
inputs (max diff < 1e-6), so we know we haven't introduced any subtle
layout bugs.

The C++ side of rl_sar got **one** addition: a new FSM state
`RLFSMStateRLASAPDab` in `src/rl_sar/fsm_robot/fsm_g1.hpp` that loads the
`g1/asap_dab` config and sets `motion_length = 5.93f` (the dab duration).
That's it. No changes to `rl_sdk.cpp`, no changes to `observation_buffer.cpp`,
no changes to the deployment binary.

## Why this matters

- **The PhD lab's tooling now works for ASAP policies.** Anyone in the lab
  who wants to deploy an ASAP-trained policy on the real G1 can use the
  same `rl_sar` they use for everything else, instead of the bespoke
  Python runner I wrote in `ASAP/sim2real/rl_policy/motion_tracking_single.py`.
- **It's a portable pattern.** The wrapper-as-adapter trick can be reused
  for any other ASAP policy by just re-running `export_wrapper.py` against
  a new checkpoint.
- **No tradeoffs in fidelity.** Because the wrapper is bit-exact, the
  policy behaves identically to the original ONNX inside ASAP's own
  `motion_tracking_single.py` MuJoCo sim2sim setup.

## What's the same vs different

|  | ASAP `motion_tracking_single.py` | rl_sar `g1/asap_dab` |
|--|----|----|
| Inference engine | onnxruntime (Python) | LibTorch (C++) |
| Sim backend | MuJoCo via Unitree DDS bridge (`base_sim.py`) | MuJoCo direct (`rl_sim_mujoco`) |
| Real backend | Unitree DDS via `unitree_sdk2py` | Unitree DDS via `unitree_sdk2` C++ |
| Obs construction | Hand-rolled Python, alphabetical sort | YAML-driven, term-priority history |
| FSM / state machine | Single script, keyboard `i / ] / o` | Compiled FSM, key bindings `0 / 1 / 5` |
| Generality | ASAP-only | Multi-robot, multi-policy |
| Code touched per new policy | 1 Python file | 1 YAML + 1 FSM state class |
