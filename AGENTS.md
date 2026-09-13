# Project agent memory

This file is the project's committed home for project-intrinsic agent knowledge: build, test, release, architecture, and sharp-edge notes that should travel with the code.

- This fork (EricXieQ/rl_sar_g1_deployment, branch jetson-deploy) deploys ASAP dab
  policies on a real Unitree G1. The validated policy is `policy/g1/asap_dab` (v6);
  the FSM picks the dab policy at runtime from `RL_DAB_POLICY=<folder under policy/g1>`
  (`src/rl_sar/fsm_robot/fsm_g1.hpp`, `RLFSMStateRLASAPDab`). Each folder's README
  says what it is, where its checkpoint came from and whether it has run on hardware.
- New dab policies are exported with `policy/g1/asap_dab/export_wrapper.py --ckpt
  --out` (v6's 376-dim rl_sar layout into ASAP's 380-dim actor); `config.yaml` is
  `asap_dab`'s with the top-level key renamed. Verify against the checkpoint on random
  observations before shipping.
- Robot logs: the dab diagnostic `/tmp/rl_sar_dab_log.csv` (per rep, overwritten, no
  IMU) and the rollout recorder `logs/rollout_*.csv` (`RL_RECORD=1`, every RL tick,
  IMU and tau_est, policy in the `state` column). `scripts/score_dab_session.py` scores
  both, and the MuJoCo harness trial folders, on the simulator's bar; its docstring
  records what the robot can and cannot measure (no foot force sensors).
- Session procedure and the decision rule for policy comparisons:
  `reports/HW_SESSION_2026-09.md`; capture, Vicon and Jetson gotchas:
  `reports/PROJECT_STATUS.md`. The loop timer runs 2.5 percent slow (48.8 Hz); the
  recorded dataset was taken on that clock, so do not change `loop.hpp` casually.
- The captain does not merge to main here; branches are named at delivery. Never
  touch `policy/g1/asap_dab` when adding alternatives.

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
