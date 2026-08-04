#!/usr/bin/env python3
"""
osc_listener.py -- log Vicon Nexus's OSC marker stream on the ROBOT JETSON.

Nexus (on the Vicon PC) streams OSC over the private wired link:
    /vicon/frame                      [frame, rate, flag]
    /vicon/marker/<subject>/<NAME>    [x, y, z]   (millimetres)

Because this runs on the Jetson, every frame is stamped with the SAME
time.time() clock that rl_sar writes as `t_wall` -> shared clock, no offline
clock alignment needed.

Two outputs per run (same timestamp in the filename):
  1. oscmarkers_<ts>.csv -- raw labelled marker positions (metres), one row per
     marker per frame. This is the ground truth to compare against the cleaned
     C3D you export from Nexus after post-editing.
  2. oscpose_<ts>.csv    -- pelvis 6-DOF pose per frame, in the SAME column
     format as the old vicon_*.csv (t_wall,frame,object,x,y,z,qw,qx,qy,qz,...)
     so merge_rollout_vicon.py works on it unchanged.

The pose is computed with the Umeyama/Kabsch least-squares rigid fit against a
reference marker configuration captured from the first fully-visible frame.
Because Nexus streams *labelled* markers, marker identity is fixed -> no 180
degree flips (the failure mode of Tracker's real-time solve).

Usage:
  python3 osc_listener.py                      # port 7000, subject auto-detect
  python3 osc_listener.py --subject g1_pelvis
  python3 osc_listener.py --port 7000 --out /path/to/logs
"""
import argparse, csv, math, os, signal, socket, struct, time


# ------------------------------------------------------------------ OSC parse
def _read_string(data, off):
    end = data.index(b"\x00", off)
    s = data[off:end].decode("ascii", "replace")
    off = end + 1
    off += (4 - (off % 4)) % 4
    return s, off


def _parse_msg(data, out):
    try:
        addr, off = _read_string(data, 0)
        if off >= len(data):
            return
        tt, off = _read_string(data, off)
        args = []
        for t in (tt[1:] if tt.startswith(",") else ""):
            if t == "f":   args.append(struct.unpack(">f", data[off:off + 4])[0]); off += 4
            elif t == "i": args.append(struct.unpack(">i", data[off:off + 4])[0]); off += 4
            elif t == "d": args.append(struct.unpack(">d", data[off:off + 8])[0]); off += 8
            elif t == "s": s, off = _read_string(data, off); args.append(s)
            elif t == "b":
                n = struct.unpack(">i", data[off:off + 4])[0]
                off += 4 + n + ((4 - (n % 4)) % 4); args.append(None)
            elif t in "TFN": args.append({"T": True, "F": False, "N": None}[t])
        out.append((addr, args))
    except Exception:
        pass


def parse_packet(data, out):
    if data[:8] == b"#bundle\x00":
        off = 16
        while off + 4 <= len(data):
            size = struct.unpack(">i", data[off:off + 4])[0]
            off += 4
            if size <= 0 or off + size > len(data):
                break
            parse_packet(data[off:off + size], out)
            off += size
    else:
        _parse_msg(data, out)


# ------------------------------------------------------- rigid fit (Umeyama)
def centroid(pts):
    n = len(pts)
    return [sum(p[i] for p in pts) / n for i in range(3)]


def kabsch(P, Q):
    """Rotation R (3x3, list of rows) that best maps centred P onto centred Q.
    Least-squares optimal (Kabsch/Umeyama), via Jacobi eigen-decomposition of
    the 4x4 quaternion form -- pure stdlib, no numpy needed on the Jetson."""
    # build 3x3 covariance H = sum p q^T
    H = [[sum(P[k][i] * Q[k][j] for k in range(len(P))) for j in range(3)] for i in range(3)]
    # Horn's quaternion method: build symmetric 4x4 N from H
    Sxx, Sxy, Sxz = H[0]; Syx, Syy, Syz = H[1]; Szx, Szy, Szz = H[2]
    N = [
        [Sxx + Syy + Szz, Syz - Szy,       Szx - Sxz,       Sxy - Syx],
        [Syz - Szy,       Sxx - Syy - Szz, Sxy + Syx,       Szx + Sxz],
        [Szx - Sxz,       Sxy + Syx,      -Sxx + Syy - Szz, Syz + Szy],
        [Sxy - Syx,       Szx + Sxz,       Syz + Szy,      -Sxx - Syy + Szz],
    ]
    # largest eigenvector of the symmetric 4x4 N -> quaternion (w,x,y,z).
    # Jacobi eigen-decomposition (exact, stdlib-only).
    A = [row[:] for row in N]
    V = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for _ in range(60):
        # find largest off-diagonal magnitude
        p, q, off = 0, 1, 0.0
        for i in range(4):
            for j in range(i + 1, 4):
                if abs(A[i][j]) > off:
                    off = abs(A[i][j]); p, q = i, j
        if off < 1e-12:
            break
        app, aqq, apq = A[p][p], A[q][q], A[p][q]
        theta = 0.5 * math.atan2(2.0 * apq, aqq - app) if abs(aqq - app) > 1e-18 else math.pi / 4
        c, s = math.cos(theta), math.sin(theta)
        for k in range(4):
            akp, akq = A[k][p], A[k][q]
            A[k][p] = c * akp - s * akq
            A[k][q] = s * akp + c * akq
        for k in range(4):
            apk, aqk = A[p][k], A[q][k]
            A[p][k] = c * apk - s * aqk
            A[q][k] = s * apk + c * aqk
        for k in range(4):
            vkp, vkq = V[k][p], V[k][q]
            V[k][p] = c * vkp - s * vkq
            V[k][q] = s * vkp + c * vkq
    best = max(range(4), key=lambda i: A[i][i])
    return [V[k][best] for k in range(4)]  # quaternion w,x,y,z mapping P -> Q


def quat_norm(q):
    n = math.sqrt(sum(x * x for x in q)) or 1.0
    q = [x / n for x in q]
    if q[0] < 0:                      # canonical sign
        q = [-x for x in q]
    return q


def quat_to_R(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ]


def rot_apply(R, p):
    return [sum(R[i][j] * p[j] for j in range(3)) for i in range(3)]


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7000)
    ap.add_argument("--subject", default=None, help="subject name (default: first seen)")
    ap.add_argument("--out", default=None, help="output dir (default rl_sar/logs)")
    args = ap.parse_args()

    out_dir = args.out or os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "logs"))
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    mpath = os.path.join(out_dir, "oscmarkers_%s.csv" % stamp)
    ppath = os.path.join(out_dir, "oscpose_%s.csv" % stamp)

    mf = open(mpath, "w", newline=""); mw = csv.writer(mf)
    mw.writerow(["t_wall", "frame", "subject", "marker", "x", "y", "z"])
    pf = open(ppath, "w", newline=""); pw = csv.writer(pf)
    pw.writerow(["t_wall", "frame", "object", "x", "y", "z",
                 "qw", "qx", "qy", "qz", "n_markers", "rms_mm"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    print("[osc] listening on 0.0.0.0:%d" % args.port)
    print("[osc] markers -> %s" % mpath)
    print("[osc] pose    -> %s" % ppath)

    run = {"go": True}
    signal.signal(signal.SIGINT, lambda *_: run.update(go=False))
    signal.signal(signal.SIGTERM, lambda *_: run.update(go=False))

    ref = None          # {name: [x,y,z]} reference configuration (metres, centred)
    ref_names = []
    ref_c = None
    frames = 0; posed = 0; last = time.time()
    subject = args.subject

    while run["go"]:
        sock.settimeout(0.5)
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break
        t_wall = time.time()

        msgs = []
        parse_packet(data, msgs)
        frame = None
        marks = {}
        for addr, a in msgs:
            if addr == "/vicon/frame" and a:
                frame = a[0]
            elif addr.startswith("/vicon/marker/") and len(a) >= 3:
                parts = addr.split("/")
                # /vicon/marker/<subject>/<NAME>  (preferred, 5 parts)
                if len(parts) >= 6:
                    subj, name = parts[4], parts[5]
                else:
                    subj, name = "", parts[-1]
                if subject is None and subj:
                    subject = subj
                    print("[osc] subject: %s" % subject)
                if subj and subject and subj != subject:
                    continue
                marks[name] = [a[0] / 1000.0, a[1] / 1000.0, a[2] / 1000.0]   # mm -> m

        if frame is None or not marks:
            continue
        frames += 1
        for nm, p in marks.items():
            mw.writerow(["%.6f" % t_wall, frame, subject or "", nm,
                         "%.6f" % p[0], "%.6f" % p[1], "%.6f" % p[2]])

        # establish the reference configuration from the first good frame
        if ref is None and len(marks) >= 3:
            ref_names = sorted(marks.keys())
            pts = [marks[n] for n in ref_names]
            ref_c = centroid(pts)
            ref = {n: [marks[n][i] - ref_c[i] for i in range(3)] for n in ref_names}
            print("[osc] reference set from %d markers: %s" % (len(ref_names), ", ".join(ref_names)))

        # pose: fit current markers to the reference (labelled -> no flips)
        if ref is not None:
            common = [n for n in ref_names if n in marks]
            if len(common) >= 3:
                cur = [marks[n] for n in common]
                cc = centroid(cur)
                P = [ref[n] for n in common]
                # `ref` is centred on ALL reference markers, so a SUBSET of it does
                # not have zero mean -- but Q below is centred on the subset that is
                # actually visible. Kabsch assumes both clouds share an origin, so
                # feeding it that mismatch biases the rotation AND leaves the reported
                # position on the subset centroid, which sits |m|/3 ~= 29mm away from
                # the body origin when one of four markers drops out. Re-centre the
                # reference subset, then map the centroid back through the rotation.
                # Costs nothing on full frames (p0 == 0) and takes the fit residual on
                # 3-marker frames from ~29mm to ~1.7mm.
                p0 = centroid(P)
                P = [[P[k][i] - p0[i] for i in range(3)] for k in range(len(common))]
                Q = [[cur[k][i] - cc[i] for i in range(3)] for k in range(len(common))]
                q = quat_norm(kabsch(P, Q))
                R = quat_to_R(q)
                # body origin = subset centroid displaced back by the rotated offset
                rp0 = rot_apply(R, p0)
                org = [cc[i] - rp0[i] for i in range(3)]
                # fit residual (mm) -- how rigid the cluster looked this frame
                err = 0.0
                for k, n in enumerate(common):
                    pr = rot_apply(R, P[k])
                    err += sum((pr[i] - Q[k][i]) ** 2 for i in range(3))
                rms = math.sqrt(err / len(common)) * 1000.0
                pw.writerow(["%.6f" % t_wall, frame, subject or "pelvis",
                             "%.6f" % org[0], "%.6f" % org[1], "%.6f" % org[2],
                             "%.6f" % q[0], "%.6f" % q[1], "%.6f" % q[2], "%.6f" % q[3],
                             len(common), "%.2f" % rms])
                posed += 1

        if t_wall - last >= 2.0:
            last = t_wall
            mf.flush(); pf.flush()
            print("[osc] %d frames, %d poses (markers/frame=%d)" % (frames, posed, len(marks)))

    mf.close(); pf.close(); sock.close()
    print("\n[osc] stopped. %d frames, %d poses" % (frames, posed))
    print("      markers: %s" % mpath)
    print("      pose:    %s" % ppath)


if __name__ == "__main__":
    main()
