"""Slide diagram: the rl_sar FSM tick mechanism + a 2-tick state transition.

Shows how a 200 Hz control loop runs current_state.Run() and CheckChange()
every 5 ms, while a 50 Hz RL thread independently fills the action queue.
A state transition spans TWO ticks: tick N detects (CheckChange returns a new
name), tick N+1 actually Exits / swaps / Enters (where InitRL loads the new
policy). The InitRL gap is the unmonitored window where the visible jolt is
born.

Output: /tmp/fsm_tick_flow.png
"""

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

fig, ax = plt.subplots(figsize=(15, 9))
ax.set_xlim(0, 15)
ax.set_ylim(0, 10)
ax.axis("off")

# ---------- helpers ----------
def box(x, y, w, h, text, color="#e8eef7", edge="#1f4e79", fontsize=9, weight="normal"):
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.05",
                       linewidth=1.4, facecolor=color, edgecolor=edge)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, fontweight=weight, color="#1f1f1f")

def arrow(x1, y1, x2, y2, color="#1f4e79", style="->", lw=1.4, ls="-"):
    a = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style,
                        mutation_scale=14, color=color, linewidth=lw, linestyle=ls)
    ax.add_patch(a)

# ---------- title ----------
ax.text(7.5, 9.55, "rl_sar FSM tick mechanism — 200 Hz control loop, 50 Hz RL thread",
        ha="center", fontsize=14, fontweight="bold", color="#1f1f1f")
ax.text(7.5, 9.15, "A state transition spans two ticks: detect on tick N, swap + Enter on tick N+1",
        ha="center", fontsize=10, color="#555555", style="italic")

# ---------- TICK N column (steady state) ----------
TX, TY, TW = 0.6, 1.4, 5.4
ax.text(TX + TW / 2, 8.55, "TICK N  (steady-state running locomotion)",
        ha="center", fontsize=11, fontweight="bold", color="#1f4e79")
ax.text(TX + TW / 2, 8.20, "t = 0 ms      (next tick at t = 5 ms)",
        ha="center", fontsize=9, color="#555555")

box(TX,         7.30, TW,       0.55, "RobotControl()                       rl_real_g1.cpp:196")
box(TX + 0.30,  6.55, TW - 0.6, 0.55, "StateController()                    rl_real_g1.cpp:200")
box(TX + 0.60,  5.80, TW - 1.2, 0.55, "fsm.Run()                            rl_sdk.cpp:23",
    color="#fff4d6", edge="#b08000", weight="bold")

# Two children of fsm.Run on tick N: Run() and CheckChange()
box(TX + 0.30,  4.75, 2.20, 0.85,
    "current_state_->Run()\n→ RLControl()  → try_pop\nrl_sdk.cpp:613",
    color="#e8f5e8", edge="#2d8c2d", fontsize=8)
box(TX + 2.85,  4.75, 2.30, 0.85,
    "CheckChange()\nfsm.hpp:70\nkey '5' → return \"asap_dab\"",
    color="#fce8e8", edge="#b03030", fontsize=8)

# Outcome of CheckChange on tick N
box(TX + 0.30,  3.70, TW - 0.6, 0.55,
    "mode = CHANGE   (next_state_ = asap_dab)",
    color="#fce8e8", edge="#b03030", fontsize=9, weight="bold")

# Connecting arrows down the column
arrow(TX + TW / 2, 7.30, TX + TW / 2, 7.10)
arrow(TX + TW / 2, 6.55, TX + TW / 2, 6.35)
arrow(TX + TW / 2, 5.80, TX + TW / 2, 5.60)
# fsm.Run branches to its two children
arrow(TX + TW / 2, 5.80, TX + 1.40, 5.60)
arrow(TX + TW / 2, 5.80, TX + 4.00, 5.60)
# CheckChange → mode=CHANGE
arrow(TX + 4.00, 4.75, TX + TW / 2, 4.25)

# ---------- TICK N+1 column (transition tick) ----------
NX = 8.4
ax.text(NX + TW / 2, 8.55, "TICK N+1  (the swap happens here)",
        ha="center", fontsize=11, fontweight="bold", color="#b03030")
ax.text(NX + TW / 2, 8.20, "t = 5 ms",
        ha="center", fontsize=9, color="#555555")

box(NX,         7.30, TW,       0.55, "fsm.Run()  sees mode == CHANGE",
    color="#fff4d6", edge="#b08000", weight="bold")
box(NX + 0.30,  6.45, TW - 0.6, 0.55, "current_state_->Exit()         (locomotion)",
    color="#e8f5e8", edge="#2d8c2d")
box(NX + 0.30,  5.60, TW - 0.6, 0.55, "current_state_ = next_state_   (pointer swap)",
    color="#e8eef7", edge="#1f4e79")
box(NX + 0.30,  4.40, TW - 0.6, 0.85,
    "current_state_->Enter()\n  └─ InitRL()  ← loads ASAP TorchScript, resets buffers\n     ★ ~8 ms gap, no diagnostic frames captured",
    color="#fce8e8", edge="#b03030", weight="bold", fontsize=8)
box(NX + 0.30,  3.55, TW - 0.6, 0.55,
    "current_state_->Run()      (first ASAP tick — first jolt visible here)",
    color="#fce8e8", edge="#b03030", fontsize=9)

arrow(NX + TW / 2, 7.30, NX + TW / 2, 7.00)
arrow(NX + TW / 2, 6.45, NX + TW / 2, 6.15)
arrow(NX + TW / 2, 5.60, NX + TW / 2, 5.25)
arrow(NX + TW / 2, 4.40, NX + TW / 2, 4.10)

# ---------- cross-tick CHANGE flag arrow ----------
arrow(TX + TW, 3.97, NX, 7.55, color="#b03030", lw=2.0, ls="--")
ax.text((TX + TW + NX) / 2, 5.85, "mode == CHANGE\ncarries to next tick",
        ha="center", fontsize=8, color="#b03030", style="italic")

# ---------- RL thread (independent, 50 Hz) ----------
RX, RY, RW, RH = 0.6, 0.10, 5.4, 1.10
box(RX, RY, RW, RH,
    "loop_rl   (50 Hz, every 20 ms — independent thread)\n"
    "  policy.forward(obs) → push action to output queue",
    color="#eef2ff", edge="#5050a0", fontsize=9)
# arrow up into try_pop in tick N
arrow(RX + RW / 2, RY + RH, TX + 1.40, 4.75, color="#5050a0", ls=":", lw=1.4)
ax.text(RX + RW + 0.15, 0.65, "fills queue", fontsize=8, color="#5050a0",
        style="italic", ha="left", va="center")

# ---------- legend / annotation ----------
ax.text(8.4, 1.00,
        "Why this matters for the jolt:\n"
        "• tick N's Run() pops the LAST locomotion action — fine.\n"
        "• InitRL on tick N+1 loads the ASAP graph; this gap is unmonitored.\n"
        "• Tick N+1's Run() is the first ASAP forward — first observable jolt.",
        fontsize=9, color="#1f1f1f",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#fff4d6",
                  edgecolor="#b08000", linewidth=1))

plt.tight_layout()
plt.savefig("/tmp/fsm_tick_flow.png", dpi=160, bbox_inches="tight")
print("wrote /tmp/fsm_tick_flow.png")
