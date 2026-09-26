# GOP HE SO CALIB THANH MOT FILE
#
# Truoc day calibfull.json duoc CHEP TAY tu 2 file roi -> khong ai biet so trong
# do lay tu lan calib nao. File nay gop tu dong va ghi lai nguon goc vao "meta".
#
# CHAY:  python3 make_calibfull.py --unit 2
#        python3 make_calibfull.py --unit 2 --install   (nap luon cho leg_server)
#
# Xem docs/plan.md buoc A3.

import argparse
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
READ = HERE.parent / "READ"
LEG_SERVER = HERE.parent.parent / "src" / "leg_server" / "calibfull.json"

ACCEL_NEW = READ / "vinh_accel_calib_ellipsoid.json"
ACCEL_OLD = READ / "vinh_accel_calib.json"
GYRO = READ / "vinhgyrocalib.json"

G = 9.80665
GYRO_SENS_EXPECT = 65.5      # LSB/(deg/s) cho FSR dps500


def load(p):
    if not p.exists():
        sys.exit(f"Thieu file: {p}")
    return json.loads(p.read_text()), datetime.fromtimestamp(p.stat().st_mtime)


def main():
    ap = argparse.ArgumentParser()
    # --unit la ten CON CHIP, khong phai ten chan robot. He so calib la dau van
    # tay cua tung con chip (bias X 372 mg chi dung cho DUNG con do). Gan chip
    # nao vao chan nao la viec cua --install, quyet dinh sau, doi duoc.
    ap.add_argument("--unit", required=True,
                    help="nhan dan tren con IMU, vd: 1, 2, A, B")
    ap.add_argument("--accel", choices=["ellipsoid", "6mat"], default="ellipsoid")
    ap.add_argument("--install", action="store_true",
                    help="chep vao src/leg_server/calibfull.json cua CHINH Pi nay"
                         " - chi dung khi con chip nay dang cam vao Pi nay")
    args = ap.parse_args()

    apath = ACCEL_NEW if args.accel == "ellipsoid" else ACCEL_OLD
    acc, a_when = load(apath)
    gyr, g_when = load(GYRO)

    SM = np.array(acc["accel"]["SM"], float)
    bias = np.array(acc["accel"]["bias"], float)
    if SM.shape != (3, 3) or bias.shape != (3,):
        sys.exit(f"Hinh dang he so accel sai: SM{SM.shape} bias{bias.shape}")

    # Kiem tra nhat quan - sai o day la sai lang le, khong bao gio bao loi luc chay
    warn = []
    meta_in = acc.get("meta", {})
    if meta_in.get("fsr_accel", "gpm4") != "gpm4":
        warn.append(f"FSR accel = {meta_in['fsr_accel']}, code doc dang dung gpm4")
    if abs(gyr["gyro_sensitivity"] - GYRO_SENS_EXPECT) > 1:
        warn.append(f"gyro_sensitivity = {gyr['gyro_sensitivity']}"
                    f", khong khop dps500 ({GYRO_SENS_EXPECT})")
    if args.accel == "6mat":
        warn.append("dang dung calib accel CU (6 mat) - co sai so ga dat")
    if meta_in.get("dlpf") not in (None, "low"):
        warn.append(f"accel calib o DLPF={meta_in['dlpf']}, code doc dang dung low")
    dt_h = abs((a_when - g_when).total_seconds()) / 3600
    if dt_h > 24:
        warn.append(f"accel va gyro calib cach nhau {dt_h:.0f}h"
                    f" - bias gyro troi theo thoi gian")

    out = {
        "accel": {"SM": SM.tolist(), "bias": bias.tolist()},
        "gyro": {k: gyr[k] for k in
                 ("gx_bias", "gy_bias", "gz_bias", "gyro_sensitivity")},
        "meta": {
            "unit": args.unit,
            "merged_at": datetime.now().isoformat(timespec="seconds"),
            "accel_from": apath.name,
            "accel_calib_at": a_when.isoformat(timespec="seconds"),
            "accel_method": meta_in.get("method", "6mat"),
            "accel_poses": meta_in.get("poses"),
            "accel_resid_max_pct": meta_in.get("resid_max_pct"),
            "gyro_from": GYRO.name,
            "gyro_calib_at": g_when.isoformat(timespec="seconds"),
            "dlpf": meta_in.get("dlpf", "?"),
            "fsr": "gpm4 / dps500",
        },
    }

    dst = READ / f"calib_imu_{args.unit}.json"
    dst.write_text(json.dumps(out, indent=2))

    gb = np.array([gyr["gx_bias"], gyr["gy_bias"], gyr["gz_bias"]])
    print(f"\n IMU dan nhan '{args.unit}'")
    print(f"   accel  {apath.name}  ({meta_in.get('method','6mat')},"
          f" {meta_in.get('poses','?')} tu the, {a_when:%d/%m %H:%M})")
    print(f"     bias      = {np.round(bias, 4)} m/s2"
          f"  ({np.round(np.linalg.solve(SM, bias) / 8192 * 1000, 1)} mg)")
    r = meta_in.get("resid_max_pct")
    print(f"     sai ||a|| = " + (f"{r:.3f} %" if r is not None else "khong ghi"))
    print(f"   gyro   {GYRO.name}  ({g_when:%d/%m %H:%M})")
    print(f"     bias      = {np.round(gb, 2)} LSB"
          f"  = {np.round(gb / gyr['gyro_sensitivity'], 4)} deg/s")
    print(f"     khong tru bias -> troi"
          f" {np.abs(gb).max() / gyr['gyro_sensitivity'] * 60:.1f} deg sau 60s")

    for w in warn:
        print(f"   !! {w}")

    print(f"\n -> {dst}")
    if args.install:
        if LEG_SERVER.exists():
            old_unit = json.loads(LEG_SERVER.read_text()).get("meta", {}).get("unit")
            if old_unit is not None and old_unit != args.unit:
                print(f"    !! Pi nay dang chay he so cua IMU '{old_unit}',"
                      f" sap thay bang '{args.unit}'.")
            shutil.copy2(LEG_SERVER, LEG_SERVER.with_suffix(".json.bak"))
        shutil.copy2(dst, LEG_SERVER)
        print(f" -> {LEG_SERVER}  (ban cu luu o calibfull.json.bak)")
        print(f"    Pi nay gio chay he so cua IMU '{args.unit}'."
              f" Cam nham con khac vao la sai am tham, khong bao loi.")
    else:
        print(f"    Chua nap cho leg_server. Them --install khi con chip nay"
              f" dang cam vao Pi nay.")


if __name__ == "__main__":
    main()
