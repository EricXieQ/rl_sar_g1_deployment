# Jetson handoff — context for Claude Code on the device

You (Claude Code, running on the Jetson) are picking up a deployment that was
set up on a desktop PC. This doc is the brain-dump so you don't re-derive it.
Read it first, then help make the Jetson-specific edits.

---

## 1. What this repo is
`rl_sar` (fork: `EricXieQ/rl_sar_g1_deployment`) — deploys RL policies to a real
**Unitree G1** via an onboard **Jetson**. Two policies matter:
- **locomotion** (`policy/g1/robomimic/locomotion/`) — balance + idle stand
- **ASAP dab** (`policy/g1/asap_dab/`) — a motion-tracking dance, entered with key `5`

FSM (`src/rl_sar/fsm_robot/fsm_g1.hpp`): Passive `0`→ GetUp `1`→ Locomotion `5`→ ASAP Dab → back to Locomotion.

## 2. Branches (know which you're on)
- **`jetson-deploy`** ← the Jetson version (repo default). **Commit Jetson changes here.**
- **`pc-deploy`** — the desktop version (do not touch from the Jetson).
- `main` — upstream base.

## 3. What already works (don't re-solve these)
- **Model preload**: all 5 g1 policies are loaded from disk into a RAM cache
  (`preloaded_models_`) once at startup, so the FSM switch does NO disk read
  mid-control. Verified on PC: **0.00 ms cached vs 6.20 ms disk** at the switch.
- **Handoff-jolt fixes** (both active):
  - **Joint re-seed** (`fsm_g1.hpp:608`, "JOLT FIX") — held command re-seeded into
    SDK/physical order at the switch, so no joint gets the wrong target.
  - **Entry interpolation** — `interpolate_commands: true`, `interp_entry_steps: 32`
    in the dab config; ramps the PD target from held pose to policy target over
    32 ticks. (The hardcoded `kEntryBlendSec` blend in `fsm_g1.hpp` is OFF on purpose.)
- **Runtime kp/scale tuning**: keys `+`/`-` (action_scale), `[`/`]` (rl_kp). Default
  starts low; raise toward 100% or aggressive dances fall.

## 4. Your job on the Jetson: build + run (see JETSON_SETUP.md for full steps)
The preload/control code is portable — the only Jetson work is the aarch64 build.
1. System deps (`apt`), then **swap in an aarch64 libtorch** at
   `library/inference_runtime/libtorch/` (the PC's is x86; use NVIDIA's Jetson
   PyTorch wheel — symlink its `torch/` dir there).
2. Build with **`./build.sh -m`** (CMake/hardware) or **`-mj`** (also MuJoCo sim).
3. Copy the `policy/g1/` tree onto the device (preload reads `POLICY_DIR=<repo>/policy`).
4. Run: `./cmake_build/bin/rl_real_g1 <iface>` (real) or
   `./cmake_build/bin/rl_sim_mujoco g1 scene_29dof` (sim).

## 5. CRITICAL GOTCHAS (these cost hours on the PC)
- **Do NOT run plain `./build.sh`** — it runs colcon (ROS2) and fails on missing
  `control_toolbox`, and does NOT build the standalone binary. Use `./build.sh -m`/`-mj`.
- **Incremental rebuild** after editing a `.cpp`:
  `cmake --build cmake_build --target rl_real_g1 -j$(nproc)` (or `rl_sim_mujoco`).
- **A stale binary will silently lack new code.** After building, verify:
  `strings cmake_build/bin/rl_real_g1 | grep -E "INITRL|PRELOAD"` (expect matches).
- **libtorch mmaps the `.pt`**, so `/proc/<pid>/io` `read_bytes` can hide a re-read.
  Don't trust `rchar`/`read_bytes` for the preload proof — use the `[INITRL]` log line.

## 6. Verify the preload on real hardware
- At startup: `[PRELOAD] cached g1/asap_dab ...`
- At the `5` switch: `[INITRL] model for g1/asap_dab from CACHE (preloaded, no disk I/O) | acquire = ~0 ms`
- A/B: `RL_DISABLE_PRELOAD=1 ./cmake_build/bin/rl_real_g1 <iface>` → `from DISK | acquire = ~ms`.
- Helper: `scripts/verify_in_ram.sh --log /tmp/sim.log` (parses the `[INITRL]` line).
- Jetson storage is slower than the PC's, so the preload win should be **larger** here.

## 7. Key files
| File | What |
|---|---|
| `src/rl_sar/library/core/rl_sdk/rl_sdk.cpp` | `PreloadModels()` ~L293, `InitRL()` cache check ~L272, `ComputeOutput`/PD ~L343, entry-interp `RLControl` ~L763 |
| `src/rl_sar/fsm_robot/fsm_g1.hpp` | g1 FSM states; dab `Enter()` re-seed "JOLT FIX" ~L608 |
| `src/rl_sar/src/rl_real_g1.cpp` | real-robot entry; calls `PreloadModels()` ~L53 |
| `src/rl_sar/CMakeLists.txt` | auto-detects Jetson (`/etc/nv_tegra_release`)+aarch64; libtorch at `library/inference_runtime/libtorch` |
| `policy/g1/*/config.yaml` | per-policy params (kp/kd, action_scale, joint_mapping, interp) |
| `JETSON_SETUP.md` | full aarch64 build recipe |

## 8. Don't commit
The aarch64 libtorch binaries / the `library/inference_runtime/libtorch` symlink
are machine-local — set up per device, never commit. Commit only source/config
changes (CMake tweaks, launch scripts, Jetson-tuned params) to `jetson-deploy`.
