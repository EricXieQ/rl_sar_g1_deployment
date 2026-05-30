"""
Export ASAP wall-dab policy as a TorchScript module wrapped for rl_sar's
observation layout.

ASAP's actor expects a 380-dim input in this order:
    [actions(23), base_ang_vel(3), dof_pos(23), dof_vel(23),
     history_actor(304), projected_gravity(3), ref_motion_phase(1)]

rl_sar (with `observations_history: [0,1,2,3]`, priority `term`) feeds the
model a flat 376-dim vector laid out as:
    [actions_t..t-3 (4*29=116), ang_vel_t..t-3 (4*3=12),
     dof_pos_t..t-3 (4*29=116), dof_vel_t..t-3 (4*29=116),
     gravity_t..t-3 (4*3=12), phase_t..t-3 (4*1=4)]

This wrapper converts rl_sar's layout into ASAP's, drops wrist joints
(rl_sar uses 29 DOFs, ASAP only 23), runs the actor, and pads the 23-dim
action back to 29 DOFs (zeros on the 6 wrist joints).

Usage:
    cd ~/Project/humanoid/ASAP   # so that LD_LIBRARY_PATH lib is found
    LD_LIBRARY_PATH=/home/eric/miniconda3/envs/hvgym/lib:$LD_LIBRARY_PATH \
      conda run -n hvgym python \
      ~/Project/humanoid/rl_sar/policy/g1/asap_dab/export_wrapper.py
"""

import torch
import torch.nn as nn
from typing import Final

# Indices of the 23 ASAP-controlled joints inside the 29-DOF G1 robot.
# Same mapping used in motion_tracking_single.py:
#   policy_to_29 = list(range(0, 19)) + list(range(22, 26))
ASAP_DOF_INDICES = [0, 1, 2, 3, 4, 5,
                    6, 7, 8, 9, 10, 11,
                    12, 13, 14,
                    15, 16, 17, 18,
                    22, 23, 24, 25]
assert len(ASAP_DOF_INDICES) == 23


class ASAPActorModule(nn.Module):
    """Inner module — checkpoint keys are actor_module.module.{0,2,4,6}.{weight,bias}."""
    def __init__(self):
        super().__init__()
        self.module = nn.Sequential(
            nn.Linear(380, 512), nn.ELU(),
            nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, 23),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.module(x)


class ASAPActor(nn.Module):
    """Reconstruction of the ASAP MLP actor matching the checkpoint key structure."""
    def __init__(self):
        super().__init__()
        self.actor_module = ASAPActorModule()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.actor_module(x)


class RlSarASAPWrapper(nn.Module):
    """
    Bridge from rl_sar's term-priority history layout to ASAP's actor input.

    Per-frame, rl_sar provides 94 floats (29 actions + 3 ang_vel + 29 dof_pos
    + 29 dof_vel + 3 gravity + 1 phase). With history_length=4 and `term`
    priority, the input is 376 floats.
    """

    HIST_LEN: Final[int] = 4
    NDOF: Final[int] = 29
    ASAP_NDOF: Final[int] = 23

    def __init__(self, asap_actor: ASAPActor):
        super().__init__()
        self.asap_actor = asap_actor
        # Buffer of int indices that TorchScript can use to index dim=2 of a
        # (B, hist_len, 29) tensor. Registering as a buffer keeps it in the
        # state dict and on the right device.
        self.register_buffer(
            "asap_dof_idx",
            torch.tensor(ASAP_DOF_INDICES, dtype=torch.long),
            persistent=False,
        )

    def forward(self, rl_sar_input: torch.Tensor) -> torch.Tensor:
        # rl_sar_input: (B, 376)
        B = rl_sar_input.shape[0]
        H = self.HIST_LEN
        N = self.NDOF

        # Per-component slice offsets in the term-priority layout.
        # actions: 0..116, ang_vel: 116..128, dof_pos: 128..244,
        # dof_vel: 244..360, gravity: 360..372, phase: 372..376.
        actions_4 = rl_sar_input[:, 0:116].reshape(B, H, N)
        ang_vel_4 = rl_sar_input[:, 116:128].reshape(B, H, 3)
        dof_pos_4 = rl_sar_input[:, 128:244].reshape(B, H, N)
        dof_vel_4 = rl_sar_input[:, 244:360].reshape(B, H, N)
        gravity_4 = rl_sar_input[:, 360:372].reshape(B, H, 3)
        phase_4 = rl_sar_input[:, 372:376].reshape(B, H, 1)

        # Drop wrist DOFs from any 29-dim component
        actions_4_a = actions_4.index_select(2, self.asap_dof_idx)  # (B, 4, 23)
        dof_pos_4_a = dof_pos_4.index_select(2, self.asap_dof_idx)
        dof_vel_4_a = dof_vel_4.index_select(2, self.asap_dof_idx)

        # Current frame (rl_sar's term layout: index 0 == newest)
        actions_t = actions_4_a[:, 0, :]   # (B, 23)
        ang_vel_t = ang_vel_4[:, 0, :]     # (B, 3)
        dof_pos_t = dof_pos_4_a[:, 0, :]   # (B, 23)
        dof_vel_t = dof_vel_4_a[:, 0, :]   # (B, 23)
        gravity_t = gravity_4[:, 0, :]     # (B, 3)
        phase_t = phase_4[:, 0, :]         # (B, 1)

        # ASAP's history_actor: per-key contiguous, 4 frames each, in
        # alphabetical sorted-key order (actions, base_ang_vel, dof_pos,
        # dof_vel, projected_gravity, ref_motion_phase).
        history_actor = torch.cat([
            actions_4_a.reshape(B, H * self.ASAP_NDOF),  # 92
            ang_vel_4.reshape(B, H * 3),                 # 12
            dof_pos_4_a.reshape(B, H * self.ASAP_NDOF),  # 92
            dof_vel_4_a.reshape(B, H * self.ASAP_NDOF),  # 92
            gravity_4.reshape(B, H * 3),                 # 12
            phase_4.reshape(B, H * 1),                   # 4
        ], dim=-1)  # (B, 304)

        # Build ASAP's 380-dim input.
        asap_input = torch.cat([
            actions_t,      # 23
            ang_vel_t,      # 3
            dof_pos_t,      # 23
            dof_vel_t,      # 23
            history_actor,  # 304
            gravity_t,      # 3
            phase_t,        # 1
        ], dim=-1)  # (B, 380)

        asap_action = self.asap_actor(asap_input)  # (B, 23)

        # Pad 23-dim action back to 29 DOFs (wrist joints stay at 0).
        out = torch.zeros(
            B, self.NDOF, dtype=asap_action.dtype, device=asap_action.device
        )
        out.index_copy_(1, self.asap_dof_idx, asap_action)
        return out


def main():
    # Off-trajectory entry fine-tuned checkpoint (resumed from the original
    # model_93600 with off_trajectory_entry_fraction=0.2). This is the version
    # that handles the loco-stance entry without the first-tick jolt/fall.
    # Original pre-fine-tune was: 20260328_172005-...wall_domainrand.../model_93600.pt
    ckpt_path = (
        "/home/eric/Project/humanoid/ASAP/logs/DabTracking/"
        "20260504_015435-DabTracking_wall_offtraj_finetune_v3-motion_tracking-"
        "g1_29dof_anneal_23dof/model_25600.pt"
    )
    out_path = (
        "/home/eric/Project/humanoid/rl_sar/policy/g1/asap_dab/policy.pt"
    )

    print(f"Loading ASAP actor from {ckpt_path}")
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    actor_state = {
        k: v for k, v in ckpt["actor_model_state_dict"].items()
        if k.startswith("actor_module.")
    }

    actor = ASAPActor()
    missing, unexpected = actor.load_state_dict(actor_state, strict=False)
    print(f"  missing keys: {len(missing)}, unexpected keys: {len(unexpected)}")
    actor.eval()

    # Sanity check the raw actor on a zero input
    with torch.no_grad():
        sample = torch.zeros(1, 380)
        out = actor(sample)
        print(f"Raw actor output (zero input): shape={tuple(out.shape)}, "
              f"first 3={out[0, :3].tolist()}")

    wrapper = RlSarASAPWrapper(actor)
    wrapper.eval()

    # Sanity check the wrapper end-to-end
    with torch.no_grad():
        sample_376 = torch.zeros(1, 376)
        out_29 = wrapper(sample_376)
        print(f"Wrapper output (zero input): shape={tuple(out_29.shape)}")
        print(f"  wrist indices (should be 0):"
              f" 19={out_29[0, 19].item():.4f},"
              f" 20={out_29[0, 20].item():.4f},"
              f" 26={out_29[0, 26].item():.4f}")
        print(f"  active idx 0={out_29[0, 0].item():.4f},"
              f" 22={out_29[0, 22].item():.4f}")

    # Compile to TorchScript
    scripted = torch.jit.script(wrapper)
    scripted.save(out_path)
    print(f"\nSaved TorchScript wrapper to {out_path}")

    # Verify the saved file by reloading
    reloaded = torch.jit.load(out_path)
    with torch.no_grad():
        out_check = reloaded(torch.zeros(1, 376))
        print(f"Reloaded check: out shape={tuple(out_check.shape)},"
              f" close to wrapper output: {torch.allclose(out_check, out_29)}")


if __name__ == "__main__":
    main()
