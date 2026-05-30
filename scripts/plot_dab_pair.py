"""Render BEFORE and AFTER zoom plots side-by-side / individually.

Calls the existing plot_dab_transition module on two CSVs by overriding the
CSV_PATH and PNG_PATH globals.
"""
import importlib
import sys
sys.path.insert(0, "/home/eric/Project/humanoid/rl_sar/scripts")

import plot_dab_transition as P

# BEFORE
P.CSV_PATH = "/tmp/rl_sar_dab_log_BEFORE.csv"
P.PNG_PATH_FULL = "/tmp/rl_sar_dab_BEFORE_full.png"
P.PNG_PATH_ZOOM = "/tmp/rl_sar_dab_BEFORE_zoom.png"
print("=" * 60)
print("Rendering BEFORE (no fixes, Num 6 direct path)…")
print("=" * 60)
P.main()

# AFTER (have to reset module state — easiest is reload)
import matplotlib.pyplot as plt
plt.close("all")
importlib.reload(P)
P.CSV_PATH = "/tmp/rl_sar_dab_log_AFTER.csv"
P.PNG_PATH_FULL = "/tmp/rl_sar_dab_AFTER_full.png"
P.PNG_PATH_ZOOM = "/tmp/rl_sar_dab_AFTER_zoom.png"
print("=" * 60)
print("Rendering AFTER (leftover-buffer fix active, Num 6 direct path)…")
print("=" * 60)
P.main()

print()
print("Done. Outputs:")
for f in ("/tmp/rl_sar_dab_BEFORE_zoom.png",
          "/tmp/rl_sar_dab_BEFORE_full.png",
          "/tmp/rl_sar_dab_AFTER_zoom.png",
          "/tmp/rl_sar_dab_AFTER_full.png"):
    print(" ", f)
