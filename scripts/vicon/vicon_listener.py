#!/usr/bin/env python3
"""
vicon_listener.py  --  standalone Vicon Tracker UDP Object Stream logger.

Runs on the ROBOT JETSON, next to rl_sar, so every pose is stamped with the
SAME `time.time()` clock that rl_sar writes into its rollout CSV (`t_wall`).
No numpy / ROS / Vicon SDK needed -- pure Python stdlib, works on aarch64.

It is a PASSIVE listener: Vicon Tracker (on the Windows PC) streams
continuously on its own; this just receives and logs. It does NOT trigger,
start, or gate the mocap in any way.

------------------------------------------------------------------------
Enabling the stream in Vicon Tracker 3.10 (do this once on the Vicon PC):
  System / Settings -> "UDP Object Stream" (a.k.a. Real-Time / Object
  stream). Tick it ON. Note the port (default 51001). Set the destination
  to the Jetson's IP (unicast) or leave broadcast on the mocap subnet.
  Create an Object from the pelvis marker cluster and name it, e.g.
  `g1_pelvis` -- that name appears in `object` column below.
------------------------------------------------------------------------

Packet layout (Vicon Tracker UDP Object Stream, little-endian):
  header:  FrameNumber uint32 | ItemsInBlock uint8
  per item (ItemID 0x02 = object, ItemDataSize = 72):
     ItemID uint8 | ItemDataSize uint16 | ItemName char[24]
     TransX,Y,Z   double (millimetres)
     RotX,Y,Z     double (axis-angle / rotation vector, radians)
  => 5-byte header + 75 bytes per object.

Output CSV columns (position in METRES, world frame):
  t_wall,frame,object,x,y,z,qw,qx,qy,qz,rvx,rvy,rvz
  - t_wall : time.time() at packet arrival on the Jetson (shared clock)
  - frame  : Vicon frame number (for dropout detection)
  - q*     : quaternion converted from the axis-angle rotation vector
  - rv*    : the raw axis-angle rotation vector (kept for debugging)

Usage:
  python3 vicon_listener.py                       # log all objects, port 51001
  python3 vicon_listener.py --object g1_pelvis     # only the pelvis object
  python3 vicon_listener.py --port 51001 --debug   # print & hexdump 1st packet
  python3 vicon_listener.py --out /path/to/logs    # override output dir

Stop with Ctrl+C; the CSV is flushed and closed cleanly.
"""
import argparse
import csv
import math
import os
import signal
import socket
import struct
import sys
import time

HEADER = struct.Struct("<IB")          # FrameNumber, ItemsInBlock
ITEM_HDR = struct.Struct("<BH")        # ItemID, ItemDataSize
ITEM_BODY = struct.Struct("<24s6d")    # name(24) + 6 doubles
ITEM_SIZE = ITEM_HDR.size + ITEM_BODY.size   # 3 + 72 = 75


def axis_angle_to_quat(rx, ry, rz):
    """Rotation vector (axis*angle, radians) -> quaternion (w, x, y, z)."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-8:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(theta / 2.0) / theta
    return (math.cos(theta / 2.0), rx * s, ry * s, rz * s)


def parse_packet(data):
    """Yield (frame, name, x,y,z [m], qw,qx,qy,qz, rx,ry,rz) per object."""
    if len(data) < HEADER.size:
        return
    frame, n_items = HEADER.unpack_from(data, 0)
    off = HEADER.size
    for _ in range(n_items):
        if off + ITEM_HDR.size > len(data):
            break
        item_id, item_size = ITEM_HDR.unpack_from(data, off)
        off += ITEM_HDR.size
        if off + ITEM_BODY.size > len(data):
            break
        name_b, tx, ty, tz, rx, ry, rz = ITEM_BODY.unpack_from(data, off)
        off += item_size if item_size >= ITEM_BODY.size else ITEM_BODY.size
        name = name_b.split(b"\x00", 1)[0].decode("ascii", "replace")
        qw, qx, qy, qz = axis_angle_to_quat(rx, ry, rz)
        # mm -> m
        yield (frame, name, tx / 1000.0, ty / 1000.0, tz / 1000.0,
               qw, qx, qy, qz, rx, ry, rz)


def main():
    ap = argparse.ArgumentParser(description="Vicon Tracker UDP Object Stream logger")
    ap.add_argument("--port", type=int, default=51001, help="UDP port (Tracker default 51001)")
    ap.add_argument("--bind", default="0.0.0.0", help="local bind address")
    ap.add_argument("--object", default=None, help="only log this object name (default: all)")
    ap.add_argument("--out", default=None,
                    help="output dir (default: rl_sar/logs next to this script)")
    ap.add_argument("--debug", action="store_true", help="hexdump + print the first packet")
    args = ap.parse_args()

    out_dir = args.out or os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "logs"))
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, "vicon_%s.csv" % stamp)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    sock.bind((args.bind, args.port))

    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_wall", "frame", "object", "x", "y", "z",
                "qw", "qx", "qy", "qz", "rvx", "rvy", "rvz"])

    print("[vicon] listening on %s:%d  ->  %s" % (args.bind, args.port, path))
    if args.object:
        print("[vicon] filtering to object '%s'" % args.object)
    print("[vicon] Ctrl+C to stop.")

    running = {"go": True}
    def stop(*_):
        running["go"] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    rows = 0
    first = True
    seen = set()
    last_report = time.time()
    while running["go"]:
        try:
            sock.settimeout(0.5)
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break
        t_wall = time.time()

        if first and args.debug:
            first = False
            print("[vicon] first packet: %d bytes" % len(data))
            print("[vicon] hex:", data[:64].hex())
            try:
                fr, ni = HEADER.unpack_from(data, 0)
                print("[vicon] frame=%d items=%d (expected len=%d)"
                      % (fr, ni, HEADER.size + ni * ITEM_SIZE))
            except struct.error as e:
                print("[vicon] header parse error:", e)

        for rec in parse_packet(data):
            frame, name = rec[0], rec[1]
            if args.object and name != args.object:
                continue
            if name not in seen:
                seen.add(name)
                print("[vicon] streaming object: '%s'" % name)
            w.writerow([("%.6f" % t_wall), frame, name] +
                       ["%.6f" % v for v in rec[2:]])
            rows += 1

        if t_wall - last_report >= 2.0:
            last_report = t_wall
            f.flush()
            print("[vicon] %d rows logged (objects: %s)"
                  % (rows, ", ".join(sorted(seen)) or "none yet"))

    f.flush()
    f.close()
    sock.close()
    print("\n[vicon] stopped. %d rows -> %s" % (rows, path))


if __name__ == "__main__":
    main()
