# CHAY IMUFUSION TRUC TIEP TU IMU THAT - doc cam bien song, in goc ra ngay.
# Khac voi run_imufusion.py (doc lai LOG CU, offline) - file nay KHONG dung
# log, doc thang tu chip qua qwiic_icm20948.
#
#   python3 run_imufusion_live.py
#   python3 run_imufusion_live.py --out xactruc.csv    # ghi log de xem lai sau
#   python3 run_imufusion_live.py --axis-check         # xac dinh trục, xem duoi
#
# Ctrl+C de dung. Trong luc chay, nhan ENTER de danh dau moc thoi gian - moc ghi
# vao cot "marker" trong CSV.
#
# --axis-check: XAC DINH TRUC BANG SO HOC, KHONG XOAY TAY (Bay 4, docs/fusion.md)
# Giu yen tung truc huong thang len troi ~2s, so raw do duoc voi SM^-1*(AB +- G*u).
# Chinh xac hon xoay tay va tranh sach 2 thu tung lam hong 2 dot test dau:
#   - gimbal lock (Euler mat phan biet roll/yaw khi pitch -> +-90 do)
#   - loi tich phan dt (Bay 5)
# Ket qua da xac minh cho IMU '2' (2026-09-01):
#   mui ten X = chip +X  |  mui ten Y = chip -Y (IN NGUOC)  |  nam phang = chip +Z
# Chip thuan tay phai va DUNG - KHONG lat dau truc nao trong code. Xem Bay 4.

import argparse
import csv
import json
import select
import sys
import time
from pathlib import Path

import imufusion
import numpy as np
import qwiic_icm20948 as Q

HERE = Path(__file__).parent
G = 9.80665
RATE = 50.0
CALIBFULL = HERE.parent.parent / "src/leg_server/calibfull.json"

DLPF = {
    "off": None,
    "low": (Q.acc_d23bw9_n34bw4, Q.gyr_d23bw9_n35bw9),      # ~24 Hz, hop 50 Hz
    "mid": (Q.acc_d111bw4_n136bw, Q.gyr_d119bw5_n154bw3),   # ~115 Hz
}


def load_calib():
    c = json.load(open(CALIBFULL))
    SM = np.array(c["accel"]["SM"])
    AB = np.array(c["accel"]["bias"])
    GB = np.array([c["gyro"]["gx_bias"], c["gyro"]["gy_bias"], c["gyro"]["gz_bias"]])
    GS = c["gyro"]["gyro_sensitivity"]
    return SM, AB, GB, GS


def make_imu(dlpf):
    IMU = Q.QwiicIcm20948()
    if not IMU.connected:
        sys.exit("Khong tim thay IMU (i2cdetect -y 1 phai thay 0x69)")
    IMU.begin()
    IMU.setFullScaleRangeAccel(Q.gpm4)
    IMU.setFullScaleRangeGyro(Q.dps500)
    if DLPF[dlpf] is not None:
        ca, cg = DLPF[dlpf]
        IMU.setDLPFcfgAccel(ca)
        IMU.setDLPFcfgGyro(cg)
        IMU.enableDlpfAccel(True)
        IMU.enableDlpfGyro(True)
    return IMU


def enter_pressed():
    return bool(select.select([sys.stdin], [], [], 0)[0])


def axis_check(IMU, SM, AB):
    """Xac dinh truc bang so hoc - khong xoay, khong fusion, khong tich phan.

    Giu yen tung truc huong len troi, so raw do duoc voi SM^-1*(AB +- G*u).
    Khong dung dt nen mien nhiem Bay 5; khong dung Euler nen mien nhiem gimbal lock.
    """
    poses = [("X (mui ten X huong THANG LEN TROI)", 0),
             ("Y (mui ten Y huong THANG LEN TROI)", 1),
             ("Z (board NAM PHANG, mat linh kien len tren)", 2)]
    ket_qua = []

    for ten, i in poses:
        input(f"\n  Dat: {ten}\n  Giu YEN roi bam ENTER de do 2 giay... ")
        mau = []
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            if IMU.dataReady():
                IMU.getAgmt()
                mau.append([IMU.axRaw, IMU.ayRaw, IMU.azRaw])
            time.sleep(0.002)
        do_duoc = np.mean(np.array(mau, float), axis=0)

        u = np.zeros(3)
        u[i] = 1.0
        du_doan_duong = np.linalg.solve(SM, AB + G * u)   # neu chip +truc huong len
        du_doan_am = np.linalg.solve(SM, AB - G * u)      # neu chip -truc huong len

        d_duong = abs(do_duoc[i] - du_doan_duong[i])
        d_am = abs(do_duoc[i] - du_doan_am[i])
        dau = "+" if d_duong < d_am else "-"
        lech = min(d_duong, d_am)
        ket_qua.append((ten.split()[0], i, do_duoc[i], du_doan_duong[i],
                        du_doan_am[i], dau, lech, len(mau)))
        print(f"    raw[{i}] = {do_duoc[i]:+8.0f}   ({len(mau)} mau)")

    print(f"\n{'='*74}")
    print(" KET QUA - truc that cua con chip nay")
    print(f"{'='*74}")
    print(f" {'Tu the':10s} {'raw do':>9s} {'neu +':>9s} {'neu -':>9s} {'lech':>7s}  Ket luan")
    for ten, i, do, dp, dn, dau, lech, n in ket_qua:
        kl = f"huong len = chip {dau}{ten}"
        canh_bao = "  <== IN NGUOC" if dau == "-" else ""
        print(f" {ten:10s} {do:+9.0f} {dp:+9.0f} {dn:+9.0f} {lech:7.0f}  {kl}{canh_bao}")

    n_am = sum(1 for r in ket_qua if r[5] == "-")
    print()
    if n_am == 0:
        print(" Ca 3 mui ten khop chieu duong cua chip. Khong co gi bat thuong.")
    else:
        print(f" {n_am} mui ten in NGUOC chieu duong that cua chip.")
        print(" Day la loi HINH IN TREN NHUA, khong phai loi chip.")
        print(" TUYET DOI KHONG lat dau trong code - du lieu chip dang dung va")
        print(" thuan tay phai; lat 1 truc bien no thanh nghich tay -> imufusion")
        print(" (luon thuan tay phai) se xu ly sai chieu xoay. Xem Bay 4.")
    if max(r[6] for r in ket_qua) > 500:
        print("\n !! Co tu the lech > 500 LSB - co the giu chua that thang/that yen.")
    print(f"{'='*74}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dlpf", choices=["off", "low", "mid"], default="low",
                     help="phai khop DLPF luc calib (xem meta.dlpf trong calibfull.json)")
    ap.add_argument("--gain", type=float, default=0.5)
    ap.add_argument("--accel-rej", type=float, default=10.0)
    ap.add_argument("--hz", type=float, default=5.0, help="tan suat IN ra man hinh")
    ap.add_argument("--out", help="ghi log CSV ra file nay, de xac minh truc sau khi xoay")
    args = ap.parse_args()

    SM, AB, GB, GS = load_calib()

    IMU = make_imu(args.dlpf)

    ahrs_settings = imufusion.AhrsSettings(
        RATE, imufusion.CONVENTION_NWU, args.gain, 500.0, args.accel_rej, 0.0, 5.0)
    ahrs = imufusion.Ahrs()
    ahrs.set_settings(ahrs_settings)

    bias_settings = imufusion.BiasSettings(RATE, 3.0, 3.0)
    bias = imufusion.Bias()
    bias.set_settings(bias_settings)
    bias.set_offset(GB / GS)  # warm-start bang calib da co, xem docs/fusion.md Bay 2

    print(f"IMU ket noi OK. DLPF={args.dlpf}  gain={args.gain}  "
          f"acceleration_rejection={args.accel_rej}")
    print("Quy uoc tilt() de doi chieu: roll+ <=> +Y len, pitch+ <=> +X xuong.")
    if args.out:
        print(f"Dang ghi log: {args.out}  -- nhan ENTER de danh dau moc thoi gian")
    print("Ctrl+C de dung.\n")

    fout = wout = None
    if args.out:
        fout = open(args.out, "w", newline="")
        wout = csv.writer(fout)
        wout.writerow(["t", "roll", "pitch", "yaw", "norm_g", "accel_ignored",
                        "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
                        "marker"])

    t_start = time.monotonic()
    t_prev = None
    t_print = 0.0
    marker = 0

    try:
        while True:
            if enter_pressed():
                sys.stdin.readline()
                marker += 1
                print(f"-- moc {marker} --")

            if not IMU.dataReady():
                time.sleep(0.001)
                continue
            IMU.getAgmt()
            t_now = time.monotonic()

            araw = np.array([IMU.axRaw, IMU.ayRaw, IMU.azRaw], float)
            graw = np.array([IMU.gxRaw, IMU.gyRaw, IMU.gzRaw], float)

            acc = (SM @ araw - AB) / G
            gyr = bias.update(graw / GS)

            if t_prev is None:
                dt = 1.0 / RATE
            else:
                dt = t_now - t_prev
            t_prev = t_now
            dt_clamped = min(max(dt, 0.5 / RATE), 2.0 / RATE)
            ahrs.set_sample_period(dt_clamped)

            ahrs.update_no_magnetometer(gyr, acc)

            q = ahrs.get_quaternion()
            roll, pitch, yaw = imufusion.quaternion_to_euler(q)
            st = ahrs.get_internal_states()
            norm_g = float(np.linalg.norm(acc))

            if wout is not None:
                wout.writerow([f"{t_now - t_start:.4f}", f"{roll:.3f}", f"{pitch:.3f}",
                                f"{yaw:.3f}", f"{norm_g:.4f}", int(st.accelerometer_ignored),
                                *araw.astype(int), *graw.astype(int), marker])

            if t_now - t_print >= 1.0 / args.hz:
                t_print = t_now
                print(f"roll={roll:+7.2f}  pitch={pitch:+7.2f}  yaw={yaw:+7.2f}  "
                      f"||a||={norm_g:.3f}g  accel_ignored={bool(st.accelerometer_ignored)}  "
                      f"dt={dt*1000:.1f}ms")

            # cho dung nhip ~RATE Hz - vong lap nay nhe, chay nhanh hon nhieu so
            # voi RATE danh nghia (~285Hz do khong co viec gi khac canh tranh
            # CPU nhu leg_server_left.py). Neu khong cho, ~100% mau se bi KEP
            # LEN gap ~3 lan boi dt_clamped o tren, thoi phong sai tich phan
            # gyro - da phat hien qua du lieu that, xem docs/fusion.md.
            t_target = t_prev + 1.0 / RATE
            sleep_s = t_target - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\nDa dung.")
    finally:
        if fout is not None:
            fout.close()
            print(f"Da ghi: {args.out}")


if __name__ == "__main__":
    main()
