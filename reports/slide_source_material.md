# Source material for the progress talk

Not a deck — the raw material to build one from. Everything here is verified against
the data; nothing is estimated. Numbers you can quote are marked in **bold**.

---

## 1. The thread (the one-paragraph version)

ASAP corrects the sim-to-real gap by learning a *delta-action model* from real
rollouts. Training it needs, at every control step, the joint state, the action the
policy actually sent, **and the floating-base pose** — which the robot cannot measure
itself, because an IMU gives orientation but not position. So the work is: capture
robot state/action and external motion capture of the pelvis, on one trustworthy
timeline. Three things broke along the way (a policy fault, a mocap fault, a network
fault); each was diagnosed from the logs and fixed; the result is a clean, calibrated,
independently validated dataset.

**If you only make one point:** the hard part was never measuring either stream. It
was putting them on one timeline you can defend, and proving the result is right.

---

## 2. Concepts you should be able to explain in your own words

### Why the base pose needs mocap
Joint encoders give every joint angle. The IMU gives the pelvis *orientation* (it
senses gravity, so roll and pitch are absolute and drift-free). Nothing onboard gives
**position** — an IMU would have to double-integrate acceleration, which drifts
without bound in seconds. Hence external cameras.

### Why "one clock" matters
Two machines each timestamping their own data means two clocks that disagree and
drift, so you must estimate an offset (NTP/PTP, or cross-correlating a shared event).
Our design sidesteps that: the mocap stream is sent to the **robot's Jetson**, which
timestamps it on arrival with the *same* clock rl_sar uses. One clock exists, so
alignment is just matching timestamps.
*Analogy:* two people timing a race with their own watches must reconcile the watches
first; one person with one stopwatch doesn't.

### What the Kabsch fit does
Vicon reports four marker positions per frame. Kabsch (a.k.a. Umeyama) finds the
single rotation + translation that best maps the *reference* marker arrangement onto
the *current* one, in a least-squares sense. Output: one pose (position + orientation)
per frame, plus a residual telling you how well the four points still formed a rigid
body — which doubles as a health check.

### Why labelled markers matter (this is the key mocap insight)
A rigid body is defined as "marker A here, B there, C there." Every frame the solver
must decide which detected dot is which. With a near-symmetric cluster and phantom
reflections around, it can assign them wrongly — and a *mislabelled* frame produces a
perfectly plausible-looking pose that is rotated ~180°. Vicon Nexus streams markers
with **persistent labels**, so identity is fixed and cannot be swapped. That single
change took orientation flips from 655 to 0.

### What the calibration is
The Vicon marker frame (defined by where the markers sit) and the robot's URDF pelvis
frame are the same body described in two frames, differing by a fixed rotation. We
solve it by least squares against the IMU — which is in the URDF frame by definition.
It is one constant applied in software; nothing is adjusted in Vicon.

### The DDS locator bug (if asked about the network)
DDS works in two phases: *discovery* (multicast — "here is where to reach me") and
*data* (unicast to the address learned in discovery). We had briefly put **two** IP
addresses on the robot-facing interface, so the Jetson advertised two locators; the
robot intermittently sent state to the unreachable one. Discovery still succeeded, so
everything *looked* healthy while data silently vanished — which is why it presented
as random comms loss.

---

## 3. Verified numbers

### The dataset
| | |
|---|---|
| Complete dab repetitions | **69** |
| Transitions (rows) | **20,556** |
| Dab motion captured | **411 s** |
| Rep length | **~298 steps ≈ 6.0 s** |
| Full capture before filtering | 31,303 policy steps over 644 s |
| Mocap coverage of policy steps | **100 %** |
| Reps dropped (short / dead / no coverage) | **0 / 0 / 0** |

### Mocap quality (before → after the fixes)
| | before | after |
|---|---|---|
| Orientation flips | **655** | **0** |
| Mislabelled frames | **17.8 %** | **0.00 %** |
| Markers tracked per frame | dropouts | **4 / 4 always** |
| Rigid-body fit residual | — | **0.40 mm median** (max 1.77 mm) |

### The ankle fix (fine-tune result)
| | before | after |
|---|---|---|
| R_ankle_roll commanded | **~100°** | **1.7–2.9°** |
| R_ankle_roll torque | **~30 Nm** | **0.6–3.5 Nm** |
| Joint's physical limit | **~15°** | — |
| Power cuts | every run | **none** |

### Calibration (Vicon frame → URDF pelvis frame)
| | |
|---|---|
| Rotation | **roll +4.51°, pitch +5.99°, yaw −3.71°** |
| IMU agreement before | **8.23°** median |
| IMU agreement after | **2.63°** median (p95 4.45°) |
| Independent hand measurement, days earlier | +4.5 / −5.2 / −3.7° — matches |

### Validation (Vicon vs onboard IMU — independent sensors)
| | |
|---|---|
| Angular velocity, pitch axis | **corr 0.924** |
| Angular velocity, yaw axis | **corr 0.936** |
| Gravity, two independent measurements | **+9.727 vs +9.719 m/s²** (0.08 % apart) |
| Time alignment | **0 ms lag** |
| Repeatability across 69 reps | **1.4 mm** median spread (max 4.9 mm) |
| IMU gravity magnitude | 9.820 m/s² (true 9.81) |

### Rates (get these right — the earlier diagram had one wrong)
| stream | rate |
|---|---|
| Vicon / OSC | **100 Hz** |
| Policy step (and therefore rollout rows) | **~50 Hz** |
| `rt/lowstate` from the robot | **~1000 Hz** (sampled at the policy step) |

---

## 4. A suggested slide order (skeleton only — write your own words)

1. **Title** — what was built, and when
2. **Why** — ASAP needs real rollouts; base pose is the missing piece
3. **The setup** — G1 + Vicon, what each measures
4. **The pipeline** — one diagram (`pipeline_diagram.md`)
5. **The one design decision** — one clock, and what it buys you
6. **Problem 1: the ankle** — symptom → root cause → fix → before/after table
7. **Problem 2: the markers** — symptom → root cause → fix → before/after table
8. **Problem 3: the network** — brief; it is an operational lesson, not a research finding
9. **The dataset** — the numbers
10. **Calibration** — what it is, the value, why small is good
11. **Validation** — the independent-sensor argument + the plot
12. **What is *not* proven** — the honest limitations slide
13. **Next steps**

Slides 6–8 all follow the same shape: *symptom → root cause → fix → evidence*. Keeping
that rhythm makes the talk easy to follow.

---

## 5. Questions to be ready for

**"Why not do it the way the paper does?"**
Their design is sound and I kept its core idea (one clock). I diverged on one point:
they consume an already-solved pose, I record raw markers and solve the pose myself.
That mattered here — our first captures had 655 orientation flips, and the only way to
detect that was checking inter-marker distances in the raw data. A mislabelled frame
produces a plausible-looking pose. Concede: their `nav_msgs/Odometry` interface is
mocap-agnostic, which is genuinely more portable than mine.

**"How do you know the data is right?"**
Vicon and the IMU measure the same motion through completely separate physical paths,
so agreement is independent evidence: 0.92–0.94 correlation on angular velocity,
gravity agreeing to 0.08 %, and zero-lag timing.

**"Did you validate the position, not just the orientation?"**
Only indirectly — via gravity and via 1.4 mm repeatability. Direct validation would
mean differentiating position twice, which at 50 Hz is noise-dominated (0.4 mm of
position noise becomes ~1 m/s² of spurious acceleration). State this before someone
finds it.

**"Is 69 repetitions enough?"**
Enough to fit a first delta-action model (typical is 20–50 reps for one skill). But
they are all the same motion from broadly the same place — the next session should add
*coverage*, not count: different positions and headings, some reps at reduced action
scale, small perturbations.

**"Why is the ankle problem interesting?"**
Because it is a training artifact that simulation structurally cannot reveal: joint
limits, contact and the force cap absorb the bad command in sim, so it only appears on
hardware. That is exactly the class of gap ASAP exists to close.

---

## 6. Where things live

| what | where |
|---|---|
| Cleaned dataset | `logs/dab_clean_20260730.csv` |
| Merged (all steps, incl. locomotion) | `logs/merged_20260730_060409.csv` |
| Raw markers | `logs/oscmarkers_20260730_060345.csv` |
| Validation plot + script | `reports/validation.png`, `reports/validate.py` |
| Diagrams | `reports/pipeline_diagram.md`, `reports/topology_diagram.md` |
| Published release | github.com/EricXieQ/rl_sar_g1_deployment → `capture-20260730` |

**Date note:** the filenames say `20260730` because the Jetson's clock is set to
Asia/Shanghai (UTC+8). The capture actually happened **Wednesday 29 July, 6:04 PM**
Buffalo time. Use the 29th in the talk.
