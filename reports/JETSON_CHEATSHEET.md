# Jetson cheat sheet: the September dab session in short commands

Paste the block below once in each Jetson terminal you use (it only defines shortcuts,
nothing runs yet). It follows `reports/HW_SESSION_2026-09.md` exactly; that file stays
the reference for the order, the reps, and the decision rule.

```bash
cd ~/qxie2/rl_sar
HW=~/qxie2/rl_sar/logs/hw_2026-09
pol() { case "$1" in v6) echo asap_dab;; s29) echo asap_dab_v6recipe_s29;; s17) echo asap_dab_v6recipe_s17;; *) echo "use v6, s29 or s17" >&2; return 1;; esac; }
dab()  { p=$(pol "$1") || return 1; mkdir -p "$HW/$p"; if [ "$1" = v6 ]; then RL_RECORD=1 ./run_g1.sh eth0 "/tmp/g1_$1.log"; else RL_RECORD=1 RL_DAB_POLICY="$p" ./run_g1.sh eth0 "/tmp/g1_$1.log"; fi; }
save() { p=$(pol "$1") || return 1; mkdir -p "$HW/$p"; f="$HW/$p/rep$(printf %02d "$2").csv"; cp /tmp/rl_sar_dab_log.csv "$f" && echo "saved $f"; }
endblock() { p=$(pol "$1") || return 1; mv ~/qxie2/rl_sar/logs/rollout_*.csv "$HW/$p/" 2>/dev/null && echo "rollouts moved to $HW/$p/" || echo "no rollout file found"; }
```

## The session, in order

Terminal 1 runs the robot. Terminal 2 saves logs. Both need the block above.

| Block | Terminal 1 (run) | Robot keys | Terminal 2, after each rep | Terminal 2, at block end |
|---|---|---|---|---|
| 1 | `dab v6` | `0` stand, `1` walk, wait 4 to 8 s, `5` dab | `save v6 1` ... `save v6 5` | `endblock v6` |
| 2 | `dab s29` | same | `save s29 1` ... `save s29 5` | `endblock s29` |
| 3 | `dab s17` | same | `save s17 1` ... `save s17 5` | `endblock s17` |
| 4 | `dab v6` | same | `save v6 6` ... `save v6 10` | `endblock v6` |
| 5 | `dab s29` | same | `save s29 6` ... `save s29 10` | `endblock s29` |
| 6 | `dab s17` | same | `save s17 6` ... `save s17 10` | `endblock s17` |

Ending a block: walk settled, then `9` (get down) or `p` (passive) on the hoist, then
Ctrl+C in terminal 1. Never Ctrl+C mid-control.

## What to check on screen

* First policy tick: `[RECORD] rollout -> logs/rollout_...csv`
* First dab of an s29 or s17 block: `[WARNING] [DAB] policy overridden by RL_DAB_POLICY -> asap_dab_v6recipe_s29` (or `_s17`). If that line is missing, the block ran v6; stop and check.
* v6 blocks show no override line, which is correct.

## Afterwards

Copy `~/qxie2/rl_sar/logs/hw_2026-09/` back to the desktop and hand it over; the scoring
is `python3 scripts/score_dab_session.py logs/hw_2026-09` on the desktop, or send the
folder to firstmate and the verdict is written against the decision rule in the runbook.
