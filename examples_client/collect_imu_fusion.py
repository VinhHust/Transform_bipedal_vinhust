#!/usr/bin/env python3
# THU THAP + KIEM TRA IMU FUSION 2 CHAN, TRUOC KHI AP VAO policy_run.py
#
#   python3 collect_imu_fusion.py                          # chi xem live
#   python3 collect_imu_fusion.py --out fuse.csv           # ghi log de xem lai
#   python3 collect_imu_fusion.py --out fuse.csv -d 60     # chay 60 giay roi tu dung
#
# Ctrl+C de dung. Trong luc chay, nhan ENTER de danh dau moc (cot "marker"),
# giong run_imufusion_live.py - tien de danh dau "bat dau nghieng", "dat lai".
#
# TAI SAO CO FILE NAY
# Script KHONG tu tinh fusion. No goi dung class IMUFusion trong
# bipedal_nam/src/bipedal_robot/sensors/imu.py - dung y chang duong ma
# policy_run.py se di (policy_run -> TransformerAPI._imu_background_loop ->
# IMUFusion.get_fused_imu). Nen neu script nay chay dep thi policy cung dep;
# neu script nay lech thi policy se lech y het. Do la muc dich.
#
# PHEP THU QUAN TRONG NHAT: "DO LECH 2 CON"
# Theo URDF, hai IMU deu gan cung mot khoi baselink (cach nhau 3cm, cung cao
# 0.0851m) - tuc la chung LUON quay giong het nhau, khong bao gio lech that.
# Sau khi transform ve baselink, huong cua con trai va con phai phai TRUNG NHAU.
# => Do goc lech giua left_q va right_q chinh la thuoc do "transform + calib
#    dung hay sai", va no dung duoc CA KHI ROBOT DANG DONG DAY, khong can
#    do chuan nao ben ngoai. Lech lon = sai, khong phai nhieu.

import argparse
import csv
import importlib.util
import math
import select
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
IMU_PY = REPO / "bipedal_nam" / "src" / "bipedal_robot" / "sensors" / "imu.py"


def _load_imufusion_class():
    """Nap dung file imu.py, KHONG di qua package bipedal_robot.

    Ly do: bipedal_robot/__init__.py keo theo lerobot (thu vien dieu khien
    dong co, chi co tren Pi). May tinh chay client khong can lerobot chi de
    doc IMU, nen nap thang file theo duong dan de script chay duoc o moi may.
    Class lay ra van la CUNG mot class ma policy_run.py dung.
    """
    if not IMU_PY.exists():
        sys.exit(f"Khong thay {IMU_PY}")
    spec = importlib.util.spec_from_file_location("_imu_fusion_mod", IMU_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.IMUFusion


IMUFusion = _load_imufusion_class()

RAD2DEG = 180.0 / math.pi


# ==========================================================================
# HAM DUY NHAT CAN COPY SANG policy_run.py
# ==========================================================================
def get_imu_obs(fusion: IMUFusion):
    """Tra ve dung cap (orient, gyro) ma policy_run.py dang can.

        orient = (roll, pitch, yaw)  radian   -> scale_imu() lay [:2]
        gyro   = [gx, gy, gz]        rad/s    -> scale_imu() lay [:3]

    Tra ve (None, None) neu doc hong, de ben goi tu quyet dinh giu mau cu
    hay bo nhip - KHONG tu tra ve so 0, vi so 0 gia se lam policy tuong
    robot dang dung thang.
    """
    data = fusion.get_fused_imu()
    if data is None:
        return None, None
    return data["fused_euler"], data["fused_gyro"]


# ==========================================================================
# Cac ham do dac phuc vu kiem tra
# ==========================================================================
def quat_angle_deg(q1, q2) -> float:
    """Goc quay nho nhat de di tu huong q1 sang huong q2, tinh bang do.

    Giai thich de: hai quaternion don vi cham nhau (dot product) cang gan 1
    thi hai huong cang trung. Cong thuc 2*acos(|dot|) doi con so do thanh
    so DO cho de doc. Lay tri tuyet doi vi q va -q la CUNG mot huong.
    """
    d = abs(float(np.dot(np.asarray(q1, float), np.asarray(q2, float))))
    d = min(1.0, d)  # chan sai so lam tron day dot > 1 -> acos vo nghia
    return 2.0 * math.acos(d) * RAD2DEG


def enter_pressed() -> bool:
    """True khi nguoi dung vua bam ENTER.

    Chi hoat dong khi stdin la terminal that. Neu chay qua pipe hoac nohup,
    stdin dong -> select() bao "san sang" lien tuc o EOF -> dem moc loan len
    hang tram. Da dinh phai loi nay khi chay thu qua pipe.
    """
    if not sys.stdin.isatty():
        return False
    if not select.select([sys.stdin], [], [], 0)[0]:
        return False
    return sys.stdin.readline() != ""


def verdict(label: str, ok: bool, detail: str) -> None:
    print(f"  [{'DAT ' if ok else 'HONG'}] {label:28s} {detail}")


def main():
    ap = argparse.ArgumentParser(
        description="Thu thap va kiem tra IMU fusion 2 chan qua IMUFusion")
    ap.add_argument("--left-host", default="mobile2.local")
    ap.add_argument("--left-port", type=int, default=5556)
    ap.add_argument("--right-host", default="mobile1.local")
    ap.add_argument("--right-port", type=int, default=5555)
    ap.add_argument("--rate", type=float, default=20.0,
                    help="tan so lay mau (Hz). policy_run chay 20Hz (period 0.05s)")
    ap.add_argument("--hz-print", type=float, default=5.0, help="tan suat in ra man hinh")
    ap.add_argument("-d", "--duration", type=float, default=0.0,
                    help="chay bao nhieu giay roi tu dung (0 = chay den khi Ctrl+C)")
    ap.add_argument("--out", help="ghi log CSV ra file nay")
    args = ap.parse_args()

    print("=" * 78)
    print(" THU THAP IMU FUSION 2 CHAN")
    print("=" * 78)
    print(f"  LEFT  : {args.left_host}:{args.left_port}")
    print(f"  RIGHT : {args.right_host}:{args.right_port}")
    print(f"  Rate  : {args.rate} Hz")

    try:
        fusion = IMUFusion(
            left_host=args.left_host, left_port=args.left_port,
            right_host=args.right_host, right_port=args.right_port)
    except Exception as e:
        sys.exit(f"\nKhong ket noi duoc 2 server: {e}\n"
                 f"Kiem tra leg_server_left.py / leg_server_right.py da chay chua.")

    fout = wout = None
    if args.out:
        fout = open(args.out, "w", newline="")
        wout = csv.writer(fout)
        wout.writerow([
            "t", "marker", "ok", "dt_ms",
            # fused - day la thu policy_run se an
            "roll_deg", "pitch_deg", "yaw_deg", "gx", "gy", "gz",
            # tung con SAU khi transform ve baselink - de soi con nao lech
            "L_roll_deg", "L_pitch_deg", "L_yaw_deg",
            "R_roll_deg", "R_pitch_deg", "R_yaw_deg",
            "disagree_deg",
            # quaternion tho de tinh lai offline neu can
            "Lq0", "Lq1", "Lq2", "Lq3", "Rq0", "Rq1", "Rq2", "Rq3",
            "Fq0", "Fq1", "Fq2", "Fq3",
            "Lgx", "Lgy", "Lgz", "Rgx", "Rgy", "Rgz",
        ])
        print(f"  Log   : {args.out}   (nhan ENTER de danh dau moc)")

    print("\n  Ctrl+C de dung.\n")
    print(f"  {'roll':>8s} {'pitch':>8s} {'yaw':>8s} | {'gx':>7s} {'gy':>7s} {'gz':>7s} "
          f"| {'lech L-R':>9s} | {'dt':>6s}")
    print("  " + "-" * 74)

    # thong ke gom lai de ket luan o cuoi
    n_ok = n_fail = 0
    marker = 0
    dts, disagrees = [], []
    gyro_mags = []
    roll_v, pitch_v, yaw_v = [], [], []

    t_start = time.monotonic()
    t_prev = None
    t_print = 0.0
    t_next = time.monotonic()

    try:
        while True:
            if args.duration > 0 and time.monotonic() - t_start >= args.duration:
                break

            if enter_pressed():
                marker += 1
                print(f"  -- moc {marker} --")

            data = fusion.get_fused_imu()
            t_now = time.monotonic()
            dt = 0.0 if t_prev is None else t_now - t_prev
            t_prev = t_now

            if data is None:
                n_fail += 1
                print("  !! get_fused_imu() tra ve None")
            else:
                n_ok += 1
                roll, pitch, yaw = data["fused_euler"]
                gyro = data["fused_gyro"]
                lq, rq, fq = data["left_quat"], data["right_quat"], data["fused_quat"]
                lg, rg = data["left_gyro"], data["right_gyro"]

                dis = quat_angle_deg(lq, rq)
                lr, lp, ly = fusion.quat_to_euler(lq)
                rr, rp, ry = fusion.quat_to_euler(rq)

                if dt > 0:
                    dts.append(dt)
                disagrees.append(dis)
                gyro_mags.append(float(np.linalg.norm(gyro)))
                roll_v.append(roll * RAD2DEG)
                pitch_v.append(pitch * RAD2DEG)
                yaw_v.append(yaw * RAD2DEG)

                if wout is not None:
                    wout.writerow([
                        f"{t_now - t_start:.4f}", marker, 1, f"{dt * 1000:.2f}",
                        f"{roll * RAD2DEG:.3f}", f"{pitch * RAD2DEG:.3f}",
                        f"{yaw * RAD2DEG:.3f}",
                        f"{gyro[0]:.5f}", f"{gyro[1]:.5f}", f"{gyro[2]:.5f}",
                        f"{lr * RAD2DEG:.3f}", f"{lp * RAD2DEG:.3f}", f"{ly * RAD2DEG:.3f}",
                        f"{rr * RAD2DEG:.3f}", f"{rp * RAD2DEG:.3f}", f"{ry * RAD2DEG:.3f}",
                        f"{dis:.3f}",
                        *[f"{x:.6f}" for x in lq], *[f"{x:.6f}" for x in rq],
                        *[f"{x:.6f}" for x in fq],
                        *[f"{x:.5f}" for x in lg], *[f"{x:.5f}" for x in rg],
                    ])

                if t_now - t_print >= 1.0 / args.hz_print:
                    t_print = t_now
                    canh_bao = "  <== LECH LON" if dis > 15.0 else ""
                    print(f"  {roll * RAD2DEG:+8.2f} {pitch * RAD2DEG:+8.2f} "
                          f"{yaw * RAD2DEG:+8.2f} | {gyro[0]:+7.3f} {gyro[1]:+7.3f} "
                          f"{gyro[2]:+7.3f} | {dis:8.2f}d | {dt * 1000:5.1f}ms{canh_bao}")

            # ngu TOI MOC dich, khong phai ngu them - xem bai hoc trong imu_loop
            # cua leg_server_left.py: moi vong con ton thoi gian 2 lan REQ/REP,
            # neu ngu them thi chu ky that phinh ra khong kiem soat duoc.
            t_next += 1.0 / args.rate
            sleep_s = t_next - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                t_next = time.monotonic()  # da tre - bat lai tu bay gio, khong doi bu

    except KeyboardInterrupt:
        print("\n  Da dung.")
    finally:
        if fout is not None:
            fout.close()
            print(f"  Da ghi: {args.out}")
        fusion.close()

    # ======================================================================
    # KET LUAN
    # ======================================================================
    t_total = time.monotonic() - t_start
    n_tot = n_ok + n_fail
    print("\n" + "=" * 78)
    print(" KET LUAN")
    print("=" * 78)

    if n_ok == 0:
        print("  Khong lay duoc mau nao. Kiem tra 2 server con song khong.")
        return

    rate_that = n_ok / t_total if t_total > 0 else 0.0
    ty_le_hong = n_fail / n_tot * 100.0
    dis = np.array(disagrees)
    gm = np.array(gyro_mags)
    dt_ms = np.array(dts) * 1000.0 if dts else np.array([0.0])

    print(f"  Mau: {n_ok} tot / {n_fail} hong trong {t_total:.1f}s\n")

    verdict("Ty le doc hong", ty_le_hong < 1.0, f"{ty_le_hong:.2f}%  (can < 1%)")
    verdict("Tan so that", rate_that >= args.rate * 0.9,
            f"{rate_that:.1f} Hz  (dat {args.rate:.0f} Hz)")
    verdict("Do on dinh nhip", float(dt_ms.max()) < 2000.0 / args.rate,
            f"dt trung binh {dt_ms.mean():.1f}ms, lon nhat {dt_ms.max():.1f}ms")
    verdict("Do lech 2 con IMU", float(dis.mean()) < 5.0,
            f"trung binh {dis.mean():.2f}d, lon nhat {dis.max():.2f}d  (can < 5d)")

    print(f"\n  Bien do goc fused:  roll [{min(roll_v):+7.2f} .. {max(roll_v):+7.2f}]d"
          f"   pitch [{min(pitch_v):+7.2f} .. {max(pitch_v):+7.2f}]d")
    print(f"  Yaw troi:           {yaw_v[0]:+.2f}d -> {yaw_v[-1]:+.2f}d "
          f"({(yaw_v[-1] - yaw_v[0]) / max(t_total, 1e-9) * 60:+.2f} d/phut)")
    print(f"  |gyro| trung binh:  {gm.mean():.4f} rad/s  (dung yen thi nen < 0.02)")

    print("\n  DOC KET QUA:")
    if dis.mean() >= 5.0:
        print("   - 'Do lech 2 con IMU' HONG la loi nghiem trong nhat. Hai con gan")
        print("     cung khoi baselink nen sau transform PHAI trung nhau. Lech nhieu")
        print("     nghia la mot trong hai: (a) transform_quat_to_baselink sai goc")
        print("     gan, (b) mot con chua calib. Xem ghi chu ben duoi ve chan phai.")
    if rate_that < args.rate * 0.9:
        print("   - Tan so thap la do moi vong phai cho 2 lan REQ/REP noi tiep nhau.")
        print("     Neu policy can 20Hz on dinh, phai doc 2 chan SONG SONG (2 thread)")
        print("     thay vi noi tiep nhu IMUFusion.get_fused_imu hien tai.")
    if gm.mean() >= 0.02:
        print("   - |gyro| lon khi dung yen: bias gyro chua tru het, hoac robot rung.")
    print("=" * 78)


if __name__ == "__main__":
    main()
