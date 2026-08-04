# How the capture data is cleaned

One account of what happens between the cameras and the training CSV, why each
step exists, and what it costs. Supersedes the scattered notes in
`marker_dropout_analysis.md`, two of whose claims are corrected in §5.

Every number here was measured from the published capture archives, not quoted.

---

## 1. Three units, three rates — the source of most confusion

Almost every "wait, how can that be?" question comes from mixing these up:

| unit | rate | count | what can go wrong |
|---|---|---|---|
| **Vicon frame** | 100 Hz | ~609 per rep | a marker is occluded |
| **Policy step** (a dataset row) | 48.9 Hz | ~298 per rep | pose can't be determined |
| **Repetition** (a "dab") | — | 473 total | segment incomplete or dead |

Markers go missing at the **frame** level. Rows and reps live two levels down.
Nothing propagates automatically, because **Vicon oversamples the policy rate
about 2:1** — you only need every other Vicon frame to place a policy step.

This is why "231 frames missing" and "0 dabs lost" are both true and not in
tension.

---

## 2. The pipeline, in order

### Stage 0 — capture (`osc_listener.py`, on the Jetson)

Nexus streams **labelled** markers over OSC at 100 Hz. Labelled matters: identity
is fixed upstream, so markers cannot be swapped. Per frame:

- **Position** = the centroid of the markers. Not a fit — just the mean. This is
  why position is immune to label swaps: relabelling doesn't change an average.
- **Orientation** = Kabsch/Umeyama rotation fitting the centred current markers
  onto a reference configuration captured from the first good frame.
- Also logged: `n_markers` and `rms_mm`, the per-frame health signals.

The one guard: `if len(common) >= 3`. Three non-collinear points fully determine a
rigid body; two do not (rotation about the line joining them is unconstrained), so
a 2-marker frame produces **no pose row at all** rather than a guess.

Output: raw markers CSV + pose CSV.

### Stage 1 — merge (`merge_rollout_vicon.py`, offline)

Joins Vicon pose to the robot's own state/action stream, both timestamped by the
**same machine**, so alignment is timestamp matching — no clock sync, no offset
estimation. Verified: cross-correlating Vicon and IMU angular velocity peaks at
**0 ms lag**.

Two operations, and the order is the design decision:

1. **`--max-rms 5.0`** drops Vicon frames whose fit residual exceeds 5 mm —
   *before* interpolation, so pose is interpolated **across** a bad frame instead
   of **contaminated by** it.
2. **Resample onto policy step times**: position by linear interpolation,
   orientation by shortest-path slerp, then differentiate for base velocities.

There is **no gap-handling code anywhere**. The merge doesn't know a frame is
missing; it just has timestamped samples and resamples onto the policy clock. A
dropout merely makes one bracketing pair 20 ms wide instead of 10 ms. Since 100 Hz
and 48.9 Hz never align, this resampling was happening regardless.

It never extrapolates: if a step falls outside the Vicon range entirely, pose stays
`NaN` and `vicon_cover = 0`.

### Stage 2 — clean (`clean_dataset.py`)

Keeps only `asap_dab` segments that are:

1. **Complete** — at least `--min-steps`. Every real segment ran 297–299 steps.
2. **Alive** — mean `|tau_est|` above threshold, which drops reps recorded after a
   firmware power-cut.
3. **Covered** — steps without Vicon coverage are dropped.

Then rotates pose and velocities into the URDF pelvis frame using that session's
calibration, and tags `rep_id` / `rep_step`.

`--recenter` is deliberately **off**: zeroing each rep's yaw would destroy
comparability with the IMU. That is a modelling choice for the training side, not
something the export should bake in.

### Stage 3 — validate (`validate.py`)

Vicon and the onboard IMU measure the same motion through physically independent
paths, so agreement is evidence rather than self-consistency: angular velocity
correlates 0.924 (pitch) / 0.936 (yaw), gravity agrees to 0.08 %, 0 ms lag, 1.4 mm
repeatability across 69 overlaid reps.

Not validated: dynamic linear acceleration. Double-differentiating position at
50 Hz is noise-dominated.

---

## 3. What actually happens to a dropout

Following one missing marker all the way down:

```
marker occluded
   → frame has 3 markers → pose still solved (see §5 for the caveat)
   → residual is high → dropped by --max-rms
   → 10 ms hole in the pose stream
   → policy step is still BRACKETED by good samples either side
   → slerp/lerp fills it → row exists, vicon_cover = 1
```

Measured cost of that interpolation, by simulating gaps in clean data and
comparing against ground truth **during dab motion**:

| gap bridged | median error | p95 | max |
|---|---|---|---|
| 1 frame (20 ms bracket) | 0.05 mm | 0.50 mm | 3.29 mm |
| 3 frames (40 ms) | 0.09 mm | 0.59 mm | 4.08 mm |
| 7 frames (80 ms) — worst case | **0.20 mm** | 0.94 mm | 7.83 mm |

Against a 2.51 mm median fit residual and 1.4 mm rep-to-rep repeatability, the
interpolation is **an order of magnitude more precise than the measurement**. The
pelvis is heavy and smooth; over 70 ms it does nothing a slerp can't follow. This
would *not* hold for a foot or hand at impact.

---

## 4. What the dropouts actually were

| | S1 (29 Jul) | S2 (2 Aug) |
|---|---|---|
| Vicon frames | 67,504 | 402,189 |
| Frames missing a marker | 77 | 231 |
| **Inside a repetition** | **57** | **2** |
| Between reps | 20 | 0 |
| After the session ended | 0 | **229** |
| Longest run inside a rep | 7 frames (70 ms) | 1 frame (10 ms) |

**Session 2's marker tracking was essentially perfect during capture: 2 dropped
frames across 404 repetitions (0.0005 %).** Its 229 other dropouts happened in the
18.4 minutes the listener kept running *after* the last dab — packing up, people
walking between the robot and the cameras.

Session 1 is where in-rep dropouts are real, and PELVLF is the marker the left arm
sweeps over.

**Practical fix: stop `osc_listener.py` when you stop capturing.** The post-session
tail generated essentially the entire dropout statistic.

---

## 5. Two defects found, and what they change

### (a) `vicon_cover` is weaker than its name

It is a **range test**, not a density test — it asks only whether a step was
straddled by two pose samples, which can only fail at the very start or end of the
recording. It would report 100 % even if Vicon had died for ten seconds mid-session,
and every step in that hole would receive a badly wrong pose.

Report **bracket width** alongside it. Measured:

| | median | max |
|---|---|---|
| S1 | 10.1 ms | 40.2 ms |
| S2 | 10.0 ms | 25.1 ms |

Nothing anywhere exceeds 80 ms, so the data is genuinely dense — `vicon_cover`
just isn't what proves it. Worth emitting `max_bracket_ms` from the merge; right
now nothing in the pipeline would catch a Vicon outage.

### (b) The centring bug — a real defect

The reference is built once, centred on all **4** markers. On a 3-marker frame the
code takes a 3-subset of those reference vectors but centres the current points on
their own **3**-marker centroid. Kabsch assumes both clouds are centred; they
aren't, and the mismatch equals the centroid shift — **29.08 mm**, since markers
sit 86–89 mm from centre and removing one moves the mean by |m|/3.

Consequence: a 3-marker frame reports a position ~29 mm displaced, then snaps back.
Measured residual on those frames: **29.19 mm** vs 3.00 mm for 4-marker frames.

**This corrects two claims in `marker_dropout_analysis.md`:**

1. It states the pose *"still produced a valid pose — just with one fewer
   constraint."* It does not. It produces a pose with ~29 mm of systematic error.
2. It attributes the dropouts to the arm sweeping over the marker during the dab.
   True for S1; false for S2, where 229 of 231 happened after the session ended.

Its *conclusion* — the data is unaffected — still holds, but for a different
reason: in S2 the residual filter rejected all 230 bad poses, and interpolation
replaced them with something **145× more accurate** (0.20 mm vs 29 mm). Discarding
those frames is strictly better than using them.

**The fix**, one line:

```python
p0  = P.mean(0)          # subset mean of the reference vectors
Pc  = P - p0             # centre both clouds consistently
R   = kabsch(Pc, Q)
pos = cc - R @ p0        # map the 3-marker centroid back to the body origin
```

Verified: residual on those frames drops **29.19 mm → 1.66 mm**, better than
4-marker frames (fewer points, less to disagree with). 4-marker frames are
bit-identical before and after, so the change is surgical.

### Where it actually landed

Session 2 applied `--max-rms 5.0`, so all 230 bad poses were rejected — **S2 is
clean**. Session 1 did not, so the artifact reached the shipped CSV:

| | S1 |
|---|---|
| Steps carrying the artifact | **46** of 20,556 (0.22 %) |
| Repetitions touched | 15 of 69 |
| Max position correction | 29.08 mm |
| Base velocity at those steps | 0.255 m/s median vs **0.043 m/s** normal |
| Max step-to-step, before → after | 30.07 mm → **13.51 mm** |

The velocity column is where it bites: `base_lin_vel` is finite-differenced from
position, so a 29 mm jump in 20 ms becomes a spurious ~6× velocity — the fastest
"motion" in session 1, none of which happened. For delta-model training that is the
worst kind of outlier.

Both candidate fixes — re-merging with `--max-rms 5.0`, or applying the centring
fix — agree to **0.000 mm median, 1.376 mm max**, which is strong evidence both are
correct. The reimplementation reproduces the shipped pipeline to 0.0006 mm.

---

## 6. Where this leaves the dataset

- **473 of 473 repetitions retained.** Verified against the pre-clean capture for
  S1: 69 dab segments in, 69 kept, zero short or aborted.
- **0 rows missing**, 0 NaNs, every row bracketed by two real measurements.
- **S2 is clean as shipped.**
- **S1 has 46 steps (0.22 %) needing correction** — 0.03 % of the combined dataset.
  No re-capture needed; it is a re-merge.

Nothing here changes a headline number. It changes whether "session 1 is the
cleaner session" survives scrutiny, and it removes a small population of physically
impossible velocity samples before anyone trains on them.
