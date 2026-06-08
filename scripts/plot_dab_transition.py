"""Plot the locomotion → ASAP-dab transition from the rl_sar diagnostic CSV.

Run rl_sim_mujoco and trigger the dab (key '5'). Then exit the dab state
(e.g. press '1' to return to locomotion, or wait for the dab to end).
The CSV at /tmp/rl_sar_dab_log.csv will be flushed on FSM Exit().

This script overlays the PD target (what the controller is asked to track)
and the actual motor position over time, with a dashed vertical line at
the FSM switch (t=0) so the step in the target is visible.
"""

import csv
import os
import sys
import matplotlib.pyplot as plt
import numpy as np

CSV_PATH = "/tmp/rl_sar_dab_log.csv"
PNG_PATH_FULL = "/tmp/rl_sar_dab_transition_full.png"
PNG_PATH_ZOOM = "/tmp/rl_sar_dab_transition_zoom.png"

# 29-DOF G1 layout (SDK order).
JOINT_NAMES = [
    "L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r",
    "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r",
    "waist_y", "waist_r", "waist_p",
    "L_sh_p", "L_sh_r", "L_sh_y", "L_elbow", "L_wr_r", "L_wr_p", "L_wr_y",
    "R_sh_p", "R_sh_r", "R_sh_y", "R_elbow", "R_wr_r", "R_wr_p", "R_wr_y",
]

# Locomotion's joint_mapping (policy slot → SDK joint index). Copied from
# rl_sar/policy/g1/robomimic/locomotion/config.yaml. rl_sar stores
# motor_state.q / motor_command.q in *policy* order while a given policy is
# active, so the loco_last row uses these slot positions. ASAP uses identity,
# which is why every other row in the CSV is already in SDK order.
LOCOMOTION_JOINT_MAPPING = [
    0,  6, 12,
    1,  7, 13,
    2,  8, 14,
    3,  9,     15, 22,
    4, 10,     16, 23,
    5, 11,     17, 24,
                18, 25,
                19, 26,
                20, 27,
                21, 28,
]
assert len(LOCOMOTION_JOINT_MAPPING) == 29


def remap_loco_last_to_sdk_order(rows, header, mapping):
    """Convert any locomotion-sourced rows (loco_last, pre_Locomotion) from
    locomotion's policy order into SDK order, in place. After this, every
    row in the CSV is in SDK order and the plot panels show the same physical
    joint pre and post FSM switch — no apparent "jump" from the policy-mapping
    boundary alone.
    """
    # inv[sdk_idx] = the policy-order slot where this SDK joint's value lives
    inv = [0] * len(mapping)
    for policy_slot, sdk_idx in enumerate(mapping):
        inv[sdk_idx] = policy_slot

    tag_col = header.index("tag")
    LOCO_TAGS = {"loco_last", "pre_Locomotion"}
    for row in rows:
        if row[tag_col] not in LOCO_TAGS:
            continue
        for prefix in ("q", "tgt", "kp"):
            cols = [header.index(f"{prefix}{i}") for i in range(29)]
            policy_vals = [row[c] for c in cols]
            for sdk_idx in range(29):
                row[cols[sdk_idx]] = policy_vals[inv[sdk_idx]]

# All 23 active joints (the 6 wrist joints are frozen by anneal_23dof).
# Layout: 4-col x 6-row grid, grouped by body region.
# Cols 1-2: left side, cols 3-4: right side. Last cell of waist row is blank.
JOINTS_TO_PLOT = [
    # legs - hip pitch / roll
    "L_hip_p", "L_hip_r", "R_hip_p", "R_hip_r",
    # legs - hip yaw / knee
    "L_hip_y", "L_knee",  "R_hip_y", "R_knee",
    # legs - ankles
    "L_ank_p", "L_ank_r", "R_ank_p", "R_ank_r",
    # waist (3 joints, last cell blank)
    "waist_y", "waist_r", "waist_p", None,
    # arms - shoulder pitch / roll
    "L_sh_p",  "L_sh_r",  "R_sh_p",  "R_sh_r",
    # arms - shoulder yaw / elbow
    "L_sh_y",  "L_elbow", "R_sh_y",  "R_elbow",
]


def load_csv(path):
    """Load CSV without pandas. Returns (header, list_of_rows)."""
    with open(path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader]
    return header, rows


def col_array(rows, header, name, dtype=float):
    idx = header.index(name)
    if dtype is str:
        return np.array([r[idx] for r in rows])
    out = np.empty(len(rows), dtype=float)
    for k, r in enumerate(rows):
        try:
            out[k] = float(r[idx])
        except ValueError:
            out[k] = np.nan
    return out


def main():
    if not os.path.exists(CSV_PATH):
        print(f"ERROR: {CSV_PATH} not found. Run rl_sim_mujoco and trigger ASAP dab first.")
        sys.exit(1)

    header, rows = load_csv(CSV_PATH)
    # rl_sar stores motor_state.q / motor_command.q in the active policy's
    # joint order. The loco_last row was captured under locomotion's mapping;
    # convert it to SDK order so it lines up with the asap rows below.
    remap_loco_last_to_sdk_order(rows, header, LOCOMOTION_JOINT_MAPPING)

    # Insert a NaN-valued "gap" row between loco_last and the first asap row
    # so matplotlib doesn't draw a misleading interpolation line across the
    # InitRL load window (where there is no real data).
    tag_col = header.index("tag")
    loco_idx = next((i for i, r in enumerate(rows) if r[tag_col] == "loco_last"), None)
    asap_idx = next((i for i, r in enumerate(rows) if r[tag_col] == "asap"), None)
    if loco_idx is not None and asap_idx is not None and asap_idx == loco_idx + 1:
        loco_t = float(rows[loco_idx][header.index("time_ms")])
        asap_t = float(rows[asap_idx][header.index("time_ms")])
        loco_wc = float(rows[loco_idx][header.index("wallclock_ms")]) if "wallclock_ms" in header else loco_t
        asap_wc = float(rows[asap_idx][header.index("wallclock_ms")]) if "wallclock_ms" in header else asap_t
        gap_row = ["nan"] * len(header)
        gap_row[header.index("time_ms")] = str(0.5 * (loco_t + asap_t))
        if "wallclock_ms" in header:
            gap_row[header.index("wallclock_ms")] = str(0.5 * (loco_wc + asap_wc))
        gap_row[tag_col] = "gap"
        rows.insert(asap_idx, gap_row)

    sim_time_ms = col_array(rows, header, "time_ms")
    has_wallclock = "wallclock_ms" in header
    if has_wallclock:
        wc_time_ms = col_array(rows, header, "wallclock_ms")
        # Use wall-clock time: it directly measures the InitRL gap.
        time_ms = wc_time_ms
    else:
        time_ms = sim_time_ms
    tag = col_array(rows, header, "tag", dtype=str)
    print(f"Loaded {len(rows)} rows from {CSV_PATH}")
    print(f"Columns: {header[:6]}... ({len(header)} total)")
    print(f"Wall-clock column present: {has_wallclock}")
    unique, counts = np.unique(tag, return_counts=True)
    print(f"Tags: {dict(zip(unique.tolist(), counts.tolist()))}")
    print(f"Time range: {time_ms.min():.1f} → {time_ms.max():.1f} ms")

    # Detect the InitRL model-load gap: rows where wall-clock dt is much
    # larger than the FSM tick rate (~5ms at 200Hz). The gap is the time
    # spent loading the TorchScript model in Enter().
    init_rl_gap_end_ms = 0.0
    if has_wallclock:
        asap_mask = tag == "asap"
        if asap_mask.any():
            first_asap_idx = np.where(asap_mask)[0][0]
            # Wall-clock time of first asap row = duration of InitRL.
            init_rl_gap_end_ms = float(time_ms[first_asap_idx])
            print(f"InitRL load gap: {init_rl_gap_end_ms:.0f} ms")

    # Earliest pre-switch sample (for zoom xlim).
    pre_min_ms = 0.0
    pre_mask = np.array([t.startswith("pre_") for t in tag])
    if pre_mask.any():
        pre_min_ms = float(time_ms[pre_mask].min())
        print(f"Pre-switch samples span: {pre_min_ms:.0f} → 0 ms (n={int(pre_mask.sum())})")

    # Find when the ASAP policy first *computed* a new action. The first
    # asap row contains leftover-locomotion bytes (re-interpreted under the
    # new joint mapping) — it's NOT a fresh policy output. The policy runs
    # on a decimation clock (every `decimation` FSM ticks). The first row
    # whose target differs from the first asap row's target is where the
    # policy actually computed and overwrote the buffer.
    asap_indices_global = np.where(tag == "asap")[0]
    asap_start_ms = None
    if len(asap_indices_global) > 0:
        ref_col = header.index("tgt18")
        first_asap_tgt = float(rows[asap_indices_global[0]][ref_col])
        for ai in asap_indices_global[1:]:
            if abs(float(rows[ai][ref_col]) - first_asap_tgt) > 1e-4:
                asap_start_ms = time_ms[ai]
                break

    # ====================================================================
    # JOLT-TIMING SUMMARY (defensible numbers, straight from raw CSV).
    # These are the values to quote on a slide — directly measurable, no
    # plotter magic involved. Print them prominently every run.
    # ====================================================================
    print()
    print("=" * 60)
    print("JOLT-TIMING SUMMARY (cite these on the slide)")
    print("=" * 60)
    init_rl_ms_int = int(round(init_rl_gap_end_ms))
    first_policy_ms_int = int(round(asap_start_ms)) if asap_start_ms is not None else None
    print(f"  Source CSV: {CSV_PATH}")
    print(f"  Method: timestamps read directly from `wallclock_ms` column.")
    print()
    print(f"  Buffer reshuffle (~ InitRL load gap):")
    print(f"    width of the no-data window between loco_last and first asap row")
    print(f"    = {init_rl_ms_int} ms")
    print()
    if first_policy_ms_int is not None:
        print(f"  Policy first-action step:")
        print(f"    first asap row whose tgt[L_elbow] differs from the immediate")
        print(f"    post-load value (>1e-4 rad)")
        print(f"    = {first_policy_ms_int} ms")
    print()
    print(f"  Slide-ready: ~{init_rl_ms_int} ms (reshuffle) | ~{first_policy_ms_int} ms (policy step)")
    print("=" * 60)
    print()

    def render(xlim, output_path, suptitle_extra):
        n = len(JOINTS_TO_PLOT)
        n_cols = 4
        n_rows = (n + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(20, 2.2 * n_rows), sharex=True)
        axes = axes.flatten()

        # NaN-out blue (motor_q) only on asap rows inside the InitRL gap
        # (where motor_state.q can drift uncontrolled during model load).
        # Pre-switch frames are normal locomotion data — don't mask them.
        if has_wallclock and init_rl_gap_end_ms > 0:
            asap_mask_local = (tag == "asap")
            in_gap = asap_mask_local & (time_ms >= 0) & (time_ms < init_rl_gap_end_ms)

        # NaN-out the red PD target on asap rows BEFORE the policy is active
        # (0 -> asap_start_ms). In that window the policy hasn't run yet, so
        # motor_command.q still holds locomotion's leftover command in loco's
        # (scrambled) joint order; asap rows aren't remapped to SDK order, so
        # the logged target is wrong there. Mask it so the red line connects
        # the last valid loco command to the first real policy command.
        tgt_premask = None
        if asap_start_ms is not None:
            tgt_premask = (tag == "asap") & (time_ms >= 0) & (time_ms < asap_start_ms)

        for ax, name in zip(axes, JOINTS_TO_PLOT):
            if name is None:
                ax.axis("off")
                continue
            idx = JOINT_NAMES.index(name)
            q_arr = col_array(rows, header, f"q{idx}")
            tgt_arr = col_array(rows, header, f"tgt{idx}")
            kp_arr = col_array(rows, header, f"kp{idx}")

            q_deg = np.rad2deg(q_arr).astype(float)
            tgt_deg = np.rad2deg(tgt_arr).astype(float)

            # Mask the artifact-corrupted blue samples across the whole handoff
            # window (0 -> ASAP active). motor_state.q is read while InitRL is
            # swapping the model/joint-mapping, so those readings are invalid
            # (they show physically-impossible jumps). Hide them; blue resumes
            # once the policy is active and readings are trustworthy again.
            q_deg_clean = q_deg
            if tgt_premask is not None and tgt_premask.any():
                q_deg_clean = q_deg.copy()
                q_deg_clean[tgt_premask] = np.nan
            elif has_wallclock and init_rl_gap_end_ms > 0:
                q_deg_clean = q_deg.copy()
                q_deg_clean[in_gap] = np.nan

            # Render the pre-policy handoff target as a continuous HOLD: carry
            # the last valid pre-switch target across the window instead of the
            # (unreliable, loco-order) logged values there. Matches the intended
            # behavior — PD holds the entry pose until the policy first runs.
            tgt_deg_clean = tgt_deg
            if tgt_premask is not None and tgt_premask.any():
                tgt_deg_clean = tgt_deg.copy()
                pre_idx = np.where((time_ms < 0) & ~np.isnan(tgt_deg))[0]
                if len(pre_idx):
                    tgt_deg_clean[tgt_premask] = tgt_deg[pre_idx[-1]]

            ax.plot(time_ms, tgt_deg_clean, label="PD target", linewidth=2,
                    color="tab:red", linestyle="-",
                    marker="." if xlim is not None else None, markersize=3)
            ax.plot(time_ms, q_deg_clean, label="motor q (actual)", linewidth=1.5,
                    color="tab:blue",
                    marker="." if xlim is not None else None, markersize=3)

            ax.axvline(x=0, color="gray", linestyle=":", alpha=0.7,
                       label="FSM switch (key 5)")
            if has_wallclock and init_rl_gap_end_ms > 0:
                ax.axvspan(0, init_rl_gap_end_ms, color="lightgray", alpha=0.4,
                           label=f"InitRL load gap ({init_rl_gap_end_ms:.0f}ms)")
            if asap_start_ms is not None:
                ax.axvline(x=asap_start_ms, color="tab:green", linestyle="--",
                           alpha=0.7, label=f"ASAP active (t={asap_start_ms:.0f}ms)")

            if xlim is not None:
                ax.set_xlim(xlim)
                # Auto-fit y to visible non-NaN data.
                mask = (time_ms >= xlim[0]) & (time_ms <= xlim[1])
                if mask.any():
                    visible = np.concatenate([tgt_deg[mask], q_deg_clean[mask]])
                    visible = visible[np.isfinite(visible)]
                    if visible.size > 0:
                        ymin, ymax = visible.min(), visible.max()
                        pad = max(2.0, 0.1 * (ymax - ymin))
                        ax.set_ylim(ymin - pad, ymax + pad)

            kp_value = kp_arr[-1] if len(kp_arr) > 0 else 0
            ax.set_title(f"{name} (idx={idx}, kp≈{kp_value:.0f})", fontsize=9)
            ax.set_ylabel("deg")
            ax.legend(loc="best", fontsize=7)
            ax.grid(True, alpha=0.3)

        for ax in axes[n:]:
            ax.axis("off")

        time_label = "wall-clock time (ms) since FSM switch" if has_wallclock else "sim time (ms) since FSM switch"
        # Label all axes in the bottom row.
        for col in range(n_cols):
            ax_idx = (n_rows - 1) * n_cols + col
            if ax_idx < len(axes):
                axes[ax_idx].set_xlabel(time_label)
        fig.suptitle(f"Locomotion → ASAP dab transition: PD target vs motor q  ({suptitle_extra})",
                     fontsize=11, fontweight="bold")
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        fig.savefig(output_path, dpi=110)
        print(f"Saved {output_path}")

    # Full-motion view (whole dab, ~6 s).
    render(xlim=None, output_path=PNG_PATH_FULL, suptitle_extra="full dab")

    # Zoomed view around the FSM switch — this is where the jolt lives.
    # Include some pre-switch context so the audience can see "normal" before
    # the chaos starts.
    zoom_left = min(-50.0, pre_min_ms - 20) if pre_mask.any() else -20
    render(xlim=(zoom_left, 300), output_path=PNG_PATH_ZOOM,
           suptitle_extra="pre-switch context + 300 ms post — the transition jolt")

    # Report the step magnitude. Caveat: rl_sar's FSM Run() ticks at the
    # physics rate (200 Hz), but the policy only runs every `decimation` ticks
    # (50 Hz with decimation=4). So the first few "asap" rows still show
    # locomotion's stale target. We find the first actual change instead.
    print("\n=== PD target step at FSM switch (loco_last → first decimation-aligned ASAP target) ===")
    loco_indices = np.where(tag == "loco_last")[0]
    asap_indices = np.where(tag == "asap")[0]
    if len(loco_indices) > 0 and len(asap_indices) > 0:
        loco_row = rows[loco_indices[0]]
        # Use a representative joint (L_elbow, idx 18) to find when the policy
        # first updates the target. Pick the first asap row whose elbow target
        # differs from loco's by > 1e-4 (anything tiny is rounding).
        ref_col = header.index("tgt18")
        loco_ref = float(loco_row[ref_col])
        first_changed = None
        for ai in asap_indices:
            if abs(float(rows[ai][ref_col]) - loco_ref) > 1e-4:
                first_changed = ai
                break
        if first_changed is None:
            first_changed = asap_indices[0]
        asap_row = rows[first_changed]
        print(f"  (first changed asap row at t={time_ms[first_changed]:.0f} ms, "
              f"row {first_changed - asap_indices[0] + 1} of asap stream)\n")
        print(f"{'joint':<10} {'loco_tgt':>9} {'asap_tgt':>9} {'step_rad':>10} {'step_deg':>10}")
        for name in JOINTS_TO_PLOT:
            if name is None:
                continue
            idx = JOINT_NAMES.index(name)
            tgt_col_idx = header.index(f"tgt{idx}")
            loco_tgt = float(loco_row[tgt_col_idx])
            asap_tgt = float(asap_row[tgt_col_idx])
            step_rad = asap_tgt - loco_tgt
            print(f"{name:<10} {loco_tgt:>+9.3f} {asap_tgt:>+9.3f} {step_rad:>+10.3f} {np.rad2deg(step_rad):>+10.1f}")

    plt.show()


if __name__ == "__main__":
    main()
