#!/usr/bin/env python3
"""
osc_dump.py -- pure-stdlib OSC receiver that prints Nexus's messages so we can
see the format before writing the real logger. Listens UDP on --port (default
7000). Point Nexus's OSC output at this Jetson (169.254.100.1:7000).
"""
import argparse, socket, struct, time

def read_string(data, off):
    end = data.index(b'\x00', off)
    s = data[off:end].decode('ascii', 'replace')
    off = end + 1
    off += (4 - (off % 4)) % 4
    return s, off

def parse_msg(data, out):
    try:
        addr, off = read_string(data, 0)
        if off >= len(data):
            out.append((addr, [])); return
        tt, off = read_string(data, off)
        args = []
        for t in tt[1:] if tt.startswith(",") else "":
            if t == 'f':   args.append(struct.unpack(">f", data[off:off+4])[0]); off += 4
            elif t == 'i': args.append(struct.unpack(">i", data[off:off+4])[0]); off += 4
            elif t == 'd': args.append(struct.unpack(">d", data[off:off+8])[0]); off += 8
            elif t == 's': s, off = read_string(data, off); args.append(s)
            elif t == 'b':
                n = struct.unpack(">i", data[off:off+4])[0]; off += 4 + n + ((4-(n%4))%4)
                args.append("<blob %d>" % n)
            elif t in "TFN": args.append({'T':True,'F':False,'N':None}[t])
        out.append((addr, args))
    except Exception as e:
        out.append(("<parse-error: %s>" % e, []))

def parse_packet(data, out):
    if data[:8] == b"#bundle\x00":
        off = 16
        while off + 4 <= len(data):
            size = struct.unpack(">i", data[off:off+4])[0]; off += 4
            if size <= 0 or off+size > len(data): break
            parse_packet(data[off:off+size], out); off += size
    else:
        parse_msg(data, out)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=7000)
    ap.add_argument("--packets", type=int, default=6, help="how many packets to fully dump")
    args = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", args.port))
    print("[osc] listening on 0.0.0.0:%d  (point Nexus OSC at 169.254.100.1:%d)" % (args.port, args.port))
    n = 0; seen_addr = set()
    while True:
        data, addr = s.recvfrom(65535)
        out = []
        parse_packet(data, out)
        if n < args.packets:
            print("\n--- packet %d from %s (%d bytes, %d msgs) ---" % (n+1, addr[0], len(data), len(out)))
            for a, ar in out:
                print("  %-40s %s" % (a, ar))
        else:
            for a, _ in out:
                if a not in seen_addr:
                    seen_addr.add(a); print("[osc] new address: %s" % a)
        n += 1
        if n == args.packets:
            print("\n[osc] (further packets: only NEW addresses printed; %d msgs/packet) ..." % len(out))

if __name__ == "__main__":
    main()
