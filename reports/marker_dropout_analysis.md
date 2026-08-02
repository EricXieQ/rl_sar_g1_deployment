# PELVLF marker dropouts — measured, and why they do not matter

Session of **2 August 2026**, 402,189 mocap frames over 67 minutes.

During the session it looked as though the `PELVLF` marker was dropping out. It was —
the observation was correct — but the effect on the dataset is negligible. This
documents the measurement so the question does not have to be re-litigated.

---

## What actually happened

| marker | frames missing | share |
|---|---|---|
| PELVLB | 0 | 0.0000 % |
| PELVRB | 0 | 0.0000 % |
| PELVRF | 1 | 0.0002 % |
| **PELVLF** | **231** | **0.0574 %** |

`PELVLF` was lost in 231 frames — about **2.3 seconds** spread across a 67-minute
session, and roughly 230× more often than any other marker. That asymmetry is real, not
noise. The most likely cause is the left arm sweeping over it during the dab, briefly
hiding it from enough cameras to reconstruct.

## Why it does not affect the data

**1. The pose solve only needs three markers.**
Pelvis pose comes from a least-squares rigid fit (Umeyama/Kabsch) over the marker
cluster. Three non-collinear points fully determine a rigid body's position and
orientation; the fourth adds redundancy and noise-averaging. So during those 231 frames
the fit still produced a valid pose — just with one fewer constraint.

**2. Those frames are 0.057 % of the session**, and are scattered rather than
clustered, so no repetition loses a meaningful stretch.

**3. Nothing downstream degraded.** Across the whole session:

| | |
|---|---|
| Genuine orientation flips | **0** |
| Genuine mislabelled frames | **0** |
| Vicon coverage of policy steps | **100 %** |
| Repetitions dropped | **0** |

**4. The quality filter already removes anything marginal.** Frames whose rigid-fit
residual exceeded 5 mm were dropped before interpolation (0.93 %), and the pose is
interpolated across the gap rather than contaminated by it. Any frame where losing
`PELVLF` genuinely hurt the fit was caught by that filter regardless of the cause.

## PELVLF was not the geometry problem either

A separate concern was whether a marker had physically shifted during the session. It
had not — and `PELVLF` is not the culprit for the residual noise either:

| marker | mean std of its three inter-marker distances |
|---|---|
| PELVLB | 2.11 mm |
| **PELVLF** | **2.29 mm** — middle of the pack |
| PELVRB | 2.40 mm |
| **PELVRF** | **2.87 mm** — the noisiest |

`PELVLF`'s distances to the other three stay stable across all ten blocks of the
session (~170 / ~167 / ~81 mm). When it was visible, it behaved well. `PELVRF` is the
noisiest marker, but its distances **return to baseline** rather than drifting, so it
did not slip — it is simply reconstructed less consistently.

## Recorded for honesty: two earlier wrong conclusions

- I first reported that a marker had "shifted ~5–7 mm" based on comparing only the
  first and last quartile of the session. The full time course showed the distance
  rising and then **returning to baseline**, which rules out physical slippage. Two
  points are not a trend.
- A 119.7° orientation jump at 99.3 % through the session looked like a marker swap. It
  is not: the Vicon frame counter reset to 0 there (a restarted trial) and the rigid-fit
  residual at that frame is **0.05 mm**, meaning the geometry was perfect. It is a
  recording boundary, and it is harmless because alignment uses `t_wall`, not frame
  numbers.

## For the next session

Worth doing, but not urgent:

- **Re-seat or reposition `PELVLF`** — moving it slightly lower or further back, out of
  the left arm's sweep, should eliminate the dropouts.
- Re-check `PELVRF`'s mounting; it is the noisiest of the four.
- If any marker is moved, **the calibration must be re-solved** — it is tied to where
  the markers sit. Roll and pitch would change; yaw is re-solved per session anyway.

**No re-capture is needed.** The 404 repetitions from this session are usable as they
stand.
