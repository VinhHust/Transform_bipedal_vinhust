# CHAY IMUFUSION TREN LOG DA GHI SAN + KIEM CHUNG BO SO CALIB.
# Khong dung robot, khong so Madgwick. Tu ap calib tu RAW (khong doc cot
# ax/ay/az co san trong log), dung dung 2 nguon trong calibfull.json:
#   - accel: SM/bias (ellipsoid fit)
#   - gyro : gx/gy/gz_bias (lan calib dau tien)
#
#   python3 run_imufusion.py logs/angles_20260831_183200
#
# HAI PHEP KIEM CHUNG CALIB (khong phai chi chay ra so la xong):
#   1. Accel: ||a|| luc dung yen phai ~9.80665 m/s^2 (1g), o moi huong nghieng.
#      Day la kiem tra VAT LY thuan tuy, khong qua thuat toan nao.
#   2. Gyro: cho imufusion.Bias TU HOC bias tu du lieu tho (KHONG moi san dap
#      an gx_bias cu), roi so ket qua no tu hoc duoc voi gx_bias da calib.
#      Hai phuong phap doc lap ra cung mot so la bang chung manh ca hai dung.
#
# Cot roll/pitch co san trong log (tinh bang atan2 truc tiep, xem imu_soak.py
# ham tilt()) chi dung de kiem LOGIC SCRIPT nay doc file/nhan ma tran dung
# chua - KHONG phai kiem calib, vi ca hai dung chung 1 bo so calib nen khong
# doc lap voi nhau.

import argparse
import csv
import json
import math
from pathlib import Path

import imufusion
import numpy as np

HERE = Path(__file__).parent
G = 9.80665
RATE = 50.0
CALIBFULL = HERE.parent.parent / "src/leg_server/calibfull.json"


def load_calib():
    c = json.load(open(CALIBFULL))
    SM = np.array(c["accel"]["SM"])                 # ellipsoid fit
    AB = np.array(c["accel"]["bias"])                # m/s^2
    GB = np.array([c["gyro"]["gx_bias"], c["gyro"]["gy_bias"],
                    c["gyro"]["gz_bias"]])            # LSB, lan calib dau
    GS = c["gyro"]["gyro_sensitivity"]
    return SM, AB, GB, GS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_dir", help="thu muc chua all.csv, vd logs/angles_20260831_183200")
    ap.add_argument("--gain", type=float, default=0.5)
    ap.add_argument("--accel-rej", type=float, default=10.0)
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    csv_path = log_dir / "all.csv"
    out_path = log_dir / "fusion_out.csv"

    SM, AB, GB, GS = load_calib()
    gyro_bias_calib = GB / GS  # A2 - dung de SO SANH, KHONG mom cho Bias

    ahrs_settings = imufusion.AhrsSettings(
        RATE, imufusion.CONVENTION_NWU, args.gain, 500.0, args.accel_rej, 0.0, 5.0)
    ahrs = imufusion.Ahrs()
    ahrs.set_settings(ahrs_settings)

    bias_settings = imufusion.BiasSettings(RATE, 3.0, 3.0)
    bias = imufusion.Bias()
    bias.set_settings(bias_settings)
    # KHONG set_offset() - de trong = 0, bat no tu hoc tu con so 0. Bias tho
    # do duoc (~0.05-0.5 deg/s) duoi nguong stationary_threshold=3 deg/s nen
    # an toan, xem docs/fusion.md Bay 2.

    rows_out = []
    n = 0
    n_clamped = 0
    n_ignored = 0
    roll_err_sq = pitch_err_sq = 0.0
    norm_err_sq = 0.0
    norm_max_err = 0.0

    with open(csv_path) as f:
        for row in csv.DictReader(f):
            n += 1

            araw = np.array([float(row["ax_raw"]), float(row["ay_raw"]), float(row["az_raw"])])
            acc = (SM @ araw - AB) / G                # m/s^2 -> g

            norm_g = float(np.linalg.norm(acc))       # KIEM CHUNG 1: phai ~1.0
            e = abs(norm_g - 1.0)
            norm_err_sq += e * e
            norm_max_err = max(norm_max_err, e)

            graw = np.array([float(row["gx_raw"]), float(row["gy_raw"]), float(row["gz_raw"])])
            gyr_raw = graw / GS                       # deg/s, CHUA tru bias
            gyr = bias.update(gyr_raw)                 # KIEM CHUNG 2: xem cuoi

            dt = float(row["dt"])
            dt_clamped = min(max(dt, 0.5 / RATE), 2.0 / RATE)
            if dt_clamped != dt:
                n_clamped += 1
            ahrs.set_sample_period(dt_clamped)

            ahrs.update_no_magnetometer(gyr, acc)     # tra ve self, bo qua

            q = ahrs.get_quaternion()
            roll, pitch, yaw = imufusion.quaternion_to_euler(q)
            st = ahrs.get_internal_states()

            if st.accelerometer_ignored:
                n_ignored += 1

            roll_tilt = float(row["roll"])
            pitch_tilt = float(row["pitch"])
            roll_err_sq += (roll - roll_tilt) ** 2
            pitch_err_sq += (pitch - pitch_tilt) ** 2

            rows_out.append([row["t"], roll, pitch, yaw, roll_tilt, pitch_tilt, norm_g,
                              int(st.accelerometer_ignored)])

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "roll_fusion", "pitch_fusion", "yaw_fusion",
                    "roll_tilt", "pitch_tilt", "norm_g", "accel_ignored"])
        w.writerows(rows_out)

    gyro_bias_learned = bias.get_offset()
    gyro_diff = np.abs(gyro_bias_learned - gyro_bias_calib)

    print(f"Log   : {csv_path}  ({n} mau)")
    print(f"Calib : {CALIBFULL}")
    print()
    print("== KIEM CHUNG 1 - accel: ||a|| phai ~1.000 g ==")
    print(f"  RMS lech   : {math.sqrt(norm_err_sq/n)*100:.3f} %")
    print(f"  Max lech   : {norm_max_err*100:.3f} %")
    print()
    print("== KIEM CHUNG 2 - gyro: Bias tu hoc (tu 0, KHONG mom dap an) so voi calib cu ==")
    print(f"  Calib cu (A2)   : {np.round(gyro_bias_calib, 4)} deg/s")
    print(f"  Bias tu hoc duoc: {np.round(gyro_bias_learned, 4)} deg/s")
    print(f"  Lech            : {np.round(gyro_diff, 4)} deg/s")
    print()
    print("== Tham khao them (khong phai kiem calib, kiem logic script) ==")
    print(f"  roll RMS vs tilt() : {math.sqrt(roll_err_sq/n):.3f} deg")
    print(f"  pitch RMS vs tilt(): {math.sqrt(pitch_err_sq/n):.3f} deg")
    print(f"  dt bi kep          : {n_clamped}/{n} ({100*n_clamped/n:.2f}%)")
    print(f"  accel_ignored      : {n_ignored}/{n} ({100*n_ignored/n:.2f}%)")
    print()
    print(f"Da ghi: {out_path}")


if __name__ == "__main__":
    main()
