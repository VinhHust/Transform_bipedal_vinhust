#!/usr/bin/env python3
# BUOC 1 CUA HUONG PUB/SUB: NGHE THU MOT CHAN, CHUA FUSION GI CA.
#
#   python3 sub_imu_test.py                          # nghe chan phai (mobile1:5755)
#   python3 sub_imu_test.py --host mobile2.local --port 5556   # chan trai
#   python3 sub_imu_test.py -d 20                    # chay 20s roi tu ket luan
#
# Muc dich: chung minh server (leg_server_pubsub/leg_Server_*.py) dang PHAT
# duoc ~50 msg/s va seq tang lien tuc, TRUOC KHI dong vao IMUFusion.
# Neu file nay khong nhan duoc gi thi loi o server/port/wifi, khong phai o
# imu_pubsub.py.

import argparse
import sys
import time

import zmq

IMU_PUB_PORT_OFFSET = 200  # phai trung voi server va imu_pubsub.py


def main():
    ap = argparse.ArgumentParser(description="Nghe thu IMU PUB cua mot leg server")
    ap.add_argument("--host", default="mobile1.local")
    ap.add_argument("--port", type=int, default=5555, help="port REQ/REP cua server; PUB = port+200")
    ap.add_argument("-d", "--duration", type=float, default=10.0, help="giay (0 = den Ctrl+C)")
    ap.add_argument("--conflate", action="store_true",
                    help="chi giu mau moi nhat (giong imu_pubsub.py). Mac dinh TAT de dem du moi mau")
    args = ap.parse_args()

    pub_port = args.port + IMU_PUB_PORT_OFFSET
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    if args.conflate:
        sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.RCVTIMEO, 200)
    sock.connect(f"tcp://{args.host}:{pub_port}")
    print(f"SUB -> tcp://{args.host}:{pub_port}   (conflate={'ON' if args.conflate else 'OFF'})")
    print("Doi mau dau tien (SUB 'cham chan' 100-300ms la binh thuong)...\n")

    n = 0
    dropped = 0
    last_seq = None
    gaps = []  # khoang cach (ms) giua 2 lan NHAN lien tiep
    t_first = None
    t_last_rx = None
    t_start = time.monotonic()
    t_print = 0.0

    try:
        while True:
            if args.duration > 0 and time.monotonic() - t_start >= args.duration:
                break
            try:
                msg = sock.recv_json()
            except zmq.Again:
                if t_last_rx is not None and time.monotonic() - t_last_rx > 1.0:
                    print(f"  !! im lang {time.monotonic() - t_last_rx:.1f}s")
                    t_last_rx = time.monotonic()  # de khong spam
                continue

            now = time.monotonic()
            n += 1
            if t_first is None:
                t_first = now
                print(f"  mau dau tien sau {now - t_start:.2f}s: {msg}\n")
            else:
                gaps.append((now - t_last_rx) * 1000.0)
            t_last_rx = now

            seq = msg.get("seq")
            if seq is not None and last_seq is not None and seq > last_seq + 1:
                dropped += seq - last_seq - 1
            last_seq = seq

            if now - t_print >= 0.5:
                t_print = now
                q = msg.get("quat", [0, 0, 0, 0])
                g = msg.get("gyro", [0, 0, 0])
                print(f"  seq={seq:>7}  q=[{q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f}]  "
                      f"gyro=[{g[0]:+.3f} {g[1]:+.3f} {g[2]:+.3f}]  nhan={n}  roi={dropped}")
    except KeyboardInterrupt:
        print("\n  Da dung.")
    finally:
        sock.close()
        ctx.term()

    print("\n" + "=" * 70)
    if n == 0:
        print("  KHONG NHAN DUOC MAU NAO.")
        print("  - Server dang chay la leg_server_pubsub/ hay leg_server/ (ban cu khong phat)?")
        print(f"  - Log server co dong 'IMU PUB socket bound to port {pub_port}' khong?")
        print(f"  - Tu laptop: nc -zv {args.host} {pub_port}")
        sys.exit(1)

    span = (t_last_rx - t_first) if t_first is not None and n > 1 else 0.0
    rate = (n - 1) / span if span > 0 else 0.0
    print(f"  Nhan {n} mau trong {span:.1f}s  ->  {rate:.1f} msg/s  (server phat 50Hz)")
    print(f"  Roi (seq nhay coc): {dropped}  ({100.0 * dropped / max(n + dropped, 1):.2f}%)")
    if gaps:
        gaps_sorted = sorted(gaps)
        print(f"  Khoang cach nhan: trung vi {gaps_sorted[len(gaps) // 2]:.1f}ms, "
              f"p99 {gaps_sorted[int(len(gaps) * 0.99)]:.1f}ms, max {gaps_sorted[-1]:.1f}ms")
    ok = rate >= 45.0 and dropped / max(n + dropped, 1) < 0.01 if not args.conflate else rate >= 40.0
    print(f"  [{'DAT ' if ok else 'HONG'}] Buoc 1 - server phat on dinh")
    print("=" * 70)


if __name__ == "__main__":
    main()
