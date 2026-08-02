# What the cleaning stage actually did

`merged_20260730_060409.csv` → `clean_dataset.py` → `dab_clean_20260730.csv`

**Headline: almost nothing was repaired, because nothing was broken.** The only rows
removed were locomotion — which is *selecting the skill*, not fixing data. The three
filters that exist to catch genuine faults all found zero.

---

## Rows

```
31,303  →  20,556        removed 10,747
```

| removed | count | why |
|---|---|---|
| `robomimic/locomotion` rows | **10,747** | transit into and out of each dab — not the modelled skill |
| Incomplete repetitions (< 250 steps) | **0** | filter exists; found nothing |
| Dead repetitions (mean \|tau\| ≤ 8 Nm) | **0** | would be segments logged *after* a firmware power-cut |
| Rows without Vicon coverage | **0** | coverage was 100 % |

## Columns

**Removed: none.** Every original field is preserved, so nothing is lost and the
merged file can always be recovered.

**Added: 15.**

| columns | meaning |
|---|---|
| `rep_id`, `rep_step` | which repetition, and position within it |
| `base_quat_urdf_{w,x,y,z}` | orientation with the calibration applied — in the robot's own URDF pelvis frame |
| `base_lin_vel_urdf_b_{x,y,z}` | linear velocity in the body frame, derived from the corrected orientation |
| `base_ang_vel_urdf_b_{x,y,z}` | angular velocity in the body frame |
| `base_pos_rel_{x,y,z}` | position relative to each repetition's own start |

So the stage is better described as **select + derive** than as *clean*.

---

## Why the zeros are the interesting result

The filters were written because earlier captures needed them. Compare what the
previous sessions would have had to remove:

| | earlier captures | this capture |
|---|---|---|
| Orientation flips | **655** | **0** |
| Mislabelled frames | **17.8 %** | **0.00 %** |
| Repetitions logged after a power-cut | routine | **0** |
| Steps missing a base pose | frequent (arm occlusion) | **0** |

None of it appeared this time. Three upstream fixes are why:

1. **Nexus subject redefined** — markers stream with persistent labels, so identity
   cannot be swapped → no flips.
2. **Ankle fine-tune** — `R_ankle_roll` no longer commanded past its joint limit → no
   firmware power-cuts → no dead repetitions.
3. **Single-subnet network** — one address on the robot interface, so DDS advertises
   one locator → no dropped robot state.

**The claim to make is not "we cleaned the data" but "the data arrived clean."** The
problems were fixed at the source rather than patched afterwards, and the cleaning
stage is the evidence: it had nothing to remove.

---

## An approach that was tried and abandoned

An automatic marker-repair step (`repair_markers.py`) was written first. It relabelled
frames whose inter-marker geometry did not match a rigid reference.

**It made the pose worse** — 39 → 134 orientation jumps. On a near-symmetric marker
cluster a 180°-rotated labelling fits the rigid geometry just as well as the correct
one, so the repair confidently "fixed" frames into the flipped labelling. A
temporal-continuity tiebreak was added but never validated, because redefining the
Nexus subject removed the problem at source instead.

The script is kept in the repository for reference and is **not** part of the
pipeline. Worth mentioning if asked what was tried — it is a concrete example of why
fixing the measurement beats post-processing it.
