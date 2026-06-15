#!/usr/bin/env bash
# verify_in_ram.sh
# --------------------------------------------------------------------------
# Prove the ASAP dab policy is served from RAM (preloaded), NOT re-read from
# disk, at the locomotion -> dab FSM switch.  External measurement: it reads
# the OS's own counters, so it does not rely on trusting the program's logs.
#
# USAGE
#   ./verify_in_ram.sh [process-name-or-pid]      # io-counter check (default)
#   ./verify_in_ram.sh --strace [process-name]    # definitive syscall check
#
#   Default process name: rl_sim_mujoco   (use rl_real_g1 on hardware)
#
# HOW THE io-counter CHECK WORKS
#   /proc/<pid>/io 'rchar' counts bytes the process pulled in via read()
#   syscalls (whether from disk OR the OS page cache).  Loading a model
#   (torch::jit::load) reads the whole .pt file, so rchar jumps by ~the model
#   size.  If the model is already in RAM (preloaded), the switch issues NO
#   read() for it  ->  rchar stays flat.
#
#   We use rchar (not read_bytes) on purpose: the page cache can hide a
#   *physical* disk read, but the read() syscall — and thus rchar — happens
#   either way, so rchar reliably distinguishes "re-read the file" from
#   "already in RAM".  (read_bytes is printed too, for reference.)
#
# A/B TO SEE BOTH OUTCOMES
#   normal run                -> expect FLAT rchar  => served from RAM  ✅
#   RL_DISABLE_PRELOAD=1 run  -> expect a JUMP      => read from disk   ⚠
# --------------------------------------------------------------------------
set -u

THRESH_BYTES=$((500 * 1024))   # 0.5 MB; any policy .pt read far exceeds this
                               # (smallest g1 model is ~1.5 MB)
SELF="verify_in_ram"           # used to filter THIS script out of pgrep hits

# Resolve a process name (or pid) to a single pid, excluding this script's own
# processes (pgrep -f would otherwise match our command line, which contains
# the target string).
find_pid() {
  local target="$1" p cmdline
  if [[ "$target" =~ ^[0-9]+$ ]]; then echo "$target"; return; fi
  local hits=()
  while read -r p; do
    [[ -r "/proc/$p/cmdline" ]] || continue
    cmdline="$(tr '\0' ' ' < "/proc/$p/cmdline")"
    [[ "$cmdline" == *"$SELF"* ]] && continue   # skip our own shell/subshells
    hits+=("$p")
  done < <(pgrep -f "$target")
  (( ${#hits[@]} == 0 )) && return 1
  (( ${#hits[@]} > 1 )) && echo "NOTE: multiple matches (${hits[*]}); using first: ${hits[0]}" >&2
  echo "${hits[0]}"
}

# ---- strace mode (definitive: shows the actual file open at the switch) ----
if [[ "${1:-}" == "--strace" ]]; then
  TARGET="${2:-rl_sim_mujoco}"
  if ! command -v strace >/dev/null 2>&1; then
    echo "ERROR: strace not installed (sudo apt install strace)."; exit 1
  fi
  PID="$(find_pid "$TARGET")" || { echo "ERROR: no running process matches '$TARGET'. Start the sim first."; exit 1; }
  echo "Attaching strace to PID $PID ($TARGET). Watching for *.pt file opens."
  echo ">>> Now press 'Num 5' in the sim to switch to the dab."
  echo ">>> A line below = the policy was opened FROM DISK at that moment."
  echo ">>> Silence at the switch = it was already in RAM (no open).  Ctrl-C to stop."
  echo "------------------------------------------------------------------"
  sudo strace -f -p "$PID" -e trace=openat 2>&1 | grep --line-buffered '\.pt"'
  exit 0
fi

# ---- log mode (authoritative: parse the program's own [INITRL] verdict) -----
# Works regardless of mmap / page cache, because it reads the timed CACHE-vs-DISK
# line the controller prints at the switch.  Capture the sim like:
#     ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof 2>&1 | tee /tmp/sim.log
# press 1 then 5, then:  ./scripts/verify_in_ram.sh --log /tmp/sim.log
if [[ "${1:-}" == "--log" ]]; then
  LOG="${2:-/tmp/sim.log}"
  [[ -r "$LOG" ]] || { echo "ERROR: cannot read log '$LOG'. Capture the sim with: ... 2>&1 | tee $LOG"; exit 1; }
  echo "Reading: $LOG"
  echo "------------------------------------------------------------------"
  grep -E "\[PRELOAD\]" "$LOG" | tail -n 6
  echo "..."
  line="$(grep -E "\[INITRL\].*asap_dab" "$LOG" | tail -n1)"
  if [[ -z "$line" ]]; then
    echo "No [INITRL] line for asap_dab found. Did you press Num 5 while capturing?"
    exit 1
  fi
  echo "$line"
  echo "------------------------------------------------------------------"
  if grep -qi "from CACHE" <<<"$line"; then
    echo "RESULT:  ✅ SERVED FROM RAM (preloaded cache) — no disk load at the switch."
  elif grep -qi "from DISK" <<<"$line"; then
    echo "RESULT:  ⚠  READ FROM DISK — model loaded from disk at the switch (preload off/miss)."
  else
    echo "RESULT:  (could not classify; raw line above)"
  fi
  acquire="$(grep -oE "acquire = [0-9.]+ ms" <<<"$line")"
  [[ -n "$acquire" ]] && echo "         model $acquire   <- the number to cite on the slide"
  exit 0
fi

# ---- watch mode (foolproof timing: baseline, then poll until you press 5) ----
if [[ "${1:-}" == "--watch" ]]; then
  TARGET="${2:-rl_sim_mujoco}"
  WATCH_SECS="${3:-15}"
  PID="$(find_pid "$TARGET")" || { echo "ERROR: no running process matches '$TARGET'. Start the sim first."; exit 1; }
  PID="$(tail -n1 <<<"$PID")"
  [[ -r "/proc/$PID/io" ]] || { echo "ERROR: cannot read /proc/$PID/io."; exit 1; }

  r0=$(awk '/^rchar:/{print $2}' "/proc/$PID/io")
  b0=$(awk '/^read_bytes:/{print $2}' "/proc/$PID/io")
  echo "Watching PID $PID ($TARGET).  Baseline captured."
  echo ">>> You have ${WATCH_SECS}s: press 'Num 5' in the sim now (timing no longer matters)."
  echo "------------------------------------------------------------------"
  detected=0; dr=0; db=0
  steps=$(( WATCH_SECS * 5 ))
  for (( i=1; i<=steps; i++ )); do
    sleep 0.2
    r=$(awk '/^rchar:/{print $2}' "/proc/$PID/io" 2>/dev/null)
    [[ -z "${r:-}" ]] && { echo "process ended."; break; }
    b=$(awk '/^read_bytes:/{print $2}' "/proc/$PID/io")
    dr=$(( r - r0 )); db=$(( b - b0 ))
    if (( dr >= THRESH_BYTES )); then
      printf "\n>>> model-sized read() DETECTED at ~%d.%01ds : %d bytes via read()\n" $(( i/5 )) $(( (i%5)*2 )) "$dr"
      detected=1; break
    fi
  done
  echo "------------------------------------------------------------------"
  printf "final read() bytes (rchar) delta: %d\n" "$dr"
  if (( detected )); then
    echo "RESULT:  ⚠  READ FROM DISK  (policy was NOT preloaded — e.g. RL_DISABLE_PRELOAD=1)"
  else
    echo "RESULT:  ✅ SERVED FROM RAM  (no model-sized read() during the window — preloaded cache)"
    echo "         NOTE: only valid if you actually pressed Num 5 during the window above."
  fi
  exit 0
fi

# ---- default: io-counter check ----
TARGET="${1:-rl_sim_mujoco}"
PID="$(find_pid "$TARGET")" || { echo "ERROR: no running process matches '$TARGET'. Start the sim first."; exit 1; }
# find_pid may emit a NOTE on stderr; PID itself is the last line on stdout
PID="$(tail -n1 <<<"$PID")"

if [[ ! -r "/proc/$PID/io" ]]; then
  echo "ERROR: cannot read /proc/$PID/io (process gone, or permission)."; exit 1
fi

rchar()      { awk '/^rchar:/{print $2}'      "/proc/$1/io"; }
read_bytes() { awk '/^read_bytes:/{print $2}' "/proc/$1/io"; }
rss_kb()     { awk '/^VmRSS:/{print $2}'      "/proc/$1/status"; }

echo "Watching PID $PID  ($TARGET)"
echo "RAM resident now (VmRSS): $(rss_kb "$PID") kB"
echo "------------------------------------------------------------------"
r0=$(rchar "$PID"); b0=$(read_bytes "$PID")
echo "Baseline captured."
echo ">>> Now press 'Num 5' in the sim to switch  locomotion -> ASAP dab."
read -rp ">>> Then press Enter here to measure... " _

r1=$(rchar "$PID"); b1=$(read_bytes "$PID")
if [[ -z "${r1:-}" ]]; then echo "ERROR: process ended before the measurement."; exit 1; fi

dr=$(( r1 - r0 ))
db=$(( b1 - b0 ))
echo "------------------------------------------------------------------"
printf "read() bytes (rchar)        across switch: %d bytes\n" "$dr"
printf "physical disk (read_bytes)  across switch: %d bytes\n" "$db"
echo "------------------------------------------------------------------"
if (( dr < THRESH_BYTES )); then
  echo "RESULT:  ✅ SERVED FROM RAM"
  echo "         No model-sized read() at the switch ($dr B < $THRESH_BYTES B)."
  echo "         The dab policy was already in the preloaded cache."
else
  echo "RESULT:  ⚠  READ FROM DISK"
  echo "         A model-sized read() happened at the switch ($dr B)."
  echo "         The policy was NOT preloaded (e.g. RL_DISABLE_PRELOAD=1)."
fi
echo
echo "Tip: run the sim normally for ✅, then re-run it with"
echo "     RL_DISABLE_PRELOAD=1 ./rl_sim_mujoco  to see the ⚠ contrast."
