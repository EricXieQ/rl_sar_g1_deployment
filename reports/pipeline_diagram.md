# G1 + Vicon capture pipeline (current — 2026-07-31)

Supersedes the earlier version of this diagram, which was wrong in three ways:
it showed the link as `169.254.x` (that addressing caused the DDS locator fault and
was removed), it included a marker-repair step (abandoned — it made the pose worse),
and it stopped at the merged CSV (the cleaning/calibration stage was missing).

## Current pipeline

```mermaid
flowchart TD
    subgraph VPC["VICON PC — 192.168.123.50"]
        NEX["Vicon Nexus<br/>4 labelled pelvis markers<br/>PELVLF / PELVRF / PELVLB / PELVRB"]
        OSC["OSC out, port 7000<br/>100 Hz, millimetres"]
        NEX --> OSC
    end

    OSC -->|"private wired link, 192.168.123.x<br/>(address via DHCP reservation on the Jetson)"| LIS

    subgraph JET["JETSON — one clock: time.time()"]
        LIS["osc_listener.py<br/>stamped on arrival"]
        KAB["Kabsch rigid fit"]
        MK["oscmarkers CSV<br/>raw labelled markers"]
        PS["oscpose CSV<br/>pelvis 6-DOF, 100 Hz"]

        HW["G1 hardware<br/>DDS rt/lowstate"]
        POL["Policy<br/>in-process"]
        REC["RecordRollout()<br/>stamped at the policy step"]
        ROL["rollout CSV<br/>state, action, IMU, 50 Hz"]

        LIS --> MK
        LIS --> KAB --> PS
        HW --> REC
        POL --> REC
        REC --> ROL
    end

    subgraph OFF["OFFLINE"]
        MRG["merge_rollout_vicon.py<br/>match on t_wall, interpolate onto steps"]
        MERGED["merged CSV<br/>+ vicon_cover flag"]
        CLN["clean_dataset.py<br/>keep complete live dab reps<br/>apply Vicon → URDF calibration"]
        FINAL["dab_clean CSV<br/>69 reps · 20,556 transitions"]
        VAL["validate.py<br/>Vicon vs IMU cross-check"]

        MRG --> MERGED --> CLN --> FINAL
        FINAL --> VAL
    end

    PS --> MRG
    ROL --> MRG
```

## What changed from the previous diagram

| | was | now |
|---|---|---|
| Link addressing | `169.254.x` link-local | **`192.168.123.x`** — Vicon PC gets `.50` by DHCP reservation |
| Marker repair step | `repair_markers.py` in the chain | **removed** — abandoned |
| Final output | merged CSV | **`dab_clean` CSV** after calibration + rep extraction |

**Why the addressing changed.** The `169.254.100.1` address lived on the same
interface as the robot's `192.168.123.164`, so the Jetson advertised *two* DDS
locators. The robot intermittently sent `rt/lowstate` to the unreachable one, which
looked like random comms loss and forced repeated power cycles. The interface now
carries a single address and the Vicon PC sits on the robot's own subnet.

**Why the repair step went away.** It relabelled frames whose marker geometry didn't
match a rigid reference. On a near-symmetric cluster a 180°-rotated labelling fits
just as well, so it "fixed" frames into the flipped labelling and made the pose worse
(39 → 134 orientation jumps). Redefining the Nexus subject removed the problem at
source instead: 655 flips → 0.

## Sync, in one line

Both streams are timestamped by the **same machine**, so there is only one clock —
alignment is timestamp matching, not offset estimation. Verified: cross-correlating
Vicon-derived and IMU angular velocity peaks at **0 ms lag**.
