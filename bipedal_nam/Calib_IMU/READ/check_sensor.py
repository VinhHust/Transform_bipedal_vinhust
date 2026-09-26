# Kiem tra tung khoi cam bien rieng le - KHONG dung Madgwick.
# Muc dich: tach bach loi phan cung / loi calib / loi fusion.
#
#   python3 check_sensor.py [accel|tilt|gyro|spin]
#   accel: do FSR dung | tilt: goc nghieng LIVE | gyro: drift | spin: scale gyro

import qwiic_icm20948
import numpy as np
import json
import math
import time
import sys
from pathlib import Path

HERE = Path(__file__).parent          # sửa lỗi đường dẫn tương đối: luôn đọc cạnh file này
G = 9.80665

# ---------- NẠP CALIB ----------
with open(HERE / "vinh_accel_calib.json") as f:
    _a = json.load(f)["accel"]
    SM, ACCEL_BIAS = np.array(_a["SM"]), np.array(_a["bias"])

with open(HERE / "vinhgyrocalib.json") as f:
    _g = json.load(f)
    GYRO_BIAS = np.array([_g["gx_bias"], _g["gy_bias"], _g["gz_bias"]])
    GYRO_SENS = _g["gyro_sensitivity"]

# ---------- KHỞI TẠO ----------
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    sys.exit("Không tìm thấy IMU! Kiểm tra: i2cdetect -y 1  (phải thấy 0x69)")
IMU.begin()

ACCEL_FSR = {"gpm2": (qwiic_icm20948.gpm2, 2), "gpm4": (qwiic_icm20948.gpm4, 4),
             "gpm8": (qwiic_icm20948.gpm8, 8), "gpm16": (qwiic_icm20948.gpm16, 16)}
GYRO_FSR = {"dps250": (qwiic_icm20948.dps250, 131.0), "dps500": (qwiic_icm20948.dps500, 65.5),
            "dps1000": (qwiic_icm20948.dps1000, 32.8), "dps2000": (qwiic_icm20948.dps2000, 16.4)}


def read_raw(n, settle=0.05):
    """Đọc n mẫu raw, trả về (accel_mean, gyro_mean, accel_std)."""
    time.sleep(settle)
    a, g = [], []
    while len(a) < n:
        if IMU.dataReady():
            IMU.getAgmt()
            a.append([IMU.axRaw, IMU.ayRaw, IMU.azRaw])
            g.append([IMU.gxRaw, IMU.gyRaw, IMU.gzRaw])
        time.sleep(0.005)
    a, g = np.array(a, float), np.array(g, float)
    return a.mean(0), g.mean(0), a.std(0)


def tilt_from_accel(a):
    """Góc nghiêng tính TRỰC TIẾP từ trọng lực - không gyro, không filter."""
    ax, ay, az = a
    return (math.degrees(math.atan2(ay, az)),                 # roll
            math.degrees(math.atan2(-ax, math.hypot(ay, az))))  # pitch


# ================= TEST 1: DÒ FSR ĐÚNG CHO ACCEL =================
def test_accel():
    print("\n" + "=" * 66)
    print(" TEST ACCEL - Dò FSR khớp với file calib")
    print(" Đặt IMU NẰM YÊN ở tư thế bất kỳ, giữ im.")
    print("=" * 66)
    input(" Nhấn [ENTER] để đo...")

    print(f"\n {'FSR':>7} | {'raw (x,y,z)':>24} | {'‖a‖ (m/s²)':>11} | {'sai số':>8} | KL")
    print(" " + "-" * 72)
    best = None
    for name, (mode, _) in ACCEL_FSR.items():
        IMU.setFullScaleRangeAccel(mode)
        araw, _, _ = read_raw(200)
        a = SM @ araw - ACCEL_BIAS
        norm = np.linalg.norm(a)
        err = abs(norm - G)
        ok = "  <== ĐÚNG" if err < 0.3 else ""
        if best is None or err < best[1]:
            best = (name, err, mode, a)
        print(f" {name:>7} | {str(araw.round(0).astype(int)):>24} | {norm:11.3f} | {err:7.3f} | {ok}")

    name, err, mode, a = best
    print("\n" + "-" * 74)
    if err < 0.3:
        print(f" ✅ File calib của bạn được đo ở FSR = {name}")
        print(f"    -> gyroacce.py VÀ leg_server_left.py đều phải gọi:")
        print(f"       IMU.setFullScaleRangeAccel(qwiic_icm20948.{name})")
        r, p = tilt_from_accel(a)
        print(f"\n    Góc hiện tại (accel thuần): roll = {r:+.2f}°   pitch = {p:+.2f}°")
    else:
        print(f" ❌ Không FSR nào cho ‖a‖ ≈ 9.81 (gần nhất: {name}, lệch {err:.2f})")
        print("    -> File calib hỏng. Chạy lại calib_accel.py.")
    IMU.setFullScaleRangeAccel(mode)


# ================= TEST 2: ĐỌC GÓC LIVE (ACCEL THUẦN) =================
def test_tilt():
    print("\n" + "=" * 66)
    print(" ĐỌC GÓC NGHIÊNG LIVE - chỉ dùng trọng lực, KHÔNG gyro, KHÔNG Madgwick")
    print(" Không có độ trễ hội tụ: nghiêng bao nhiêu hiện ngay bấy nhiêu.")
    print(" ‖a‖ phải luôn ≈ 9.81. Nghiêng thử 30° / 45° / 90°.  Ctrl+C để thoát.")
    print("=" * 66 + "\n")
    fa = None
    try:
        while True:
            if IMU.dataReady():
                IMU.getAgmt()
                a = SM @ np.array([IMU.axRaw, IMU.ayRaw, IMU.azRaw], float) - ACCEL_BIAS
                fa = a if fa is None else 0.15 * a + 0.85 * fa
                r, p = tilt_from_accel(fa)
                n = np.linalg.norm(fa)
                flag = "OK  " if abs(n - G) < 0.3 else "SAI!"
                print(f"\r roll={r:+7.2f}°  pitch={p:+7.2f}°  |  "
                      f"a=({fa[0]:+6.2f},{fa[1]:+6.2f},{fa[2]:+6.2f})  "
                      f"‖a‖={n:6.3f} [{flag}]   ", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n")


# ================= TEST 3: DRIFT GYRO KHI ĐỨNG YÊN =================
def test_gyro():
    print("\n" + "=" * 66)
    print(" TEST GYRO - Đứng yên. Vận tốc góc phải ≈ 0 °/s")
    print("=" * 66)
    input(" Đặt IMU bất động rồi nhấn [ENTER]...")

    print(f"\n {'FSR':>8} | {'sens':>6} | {'gx':>8} {'gy':>8} {'gz':>8} (°/s) | trôi sau 60s")
    print(" " + "-" * 70)
    for name, (mode, sens_true) in GYRO_FSR.items():
        IMU.setFullScaleRangeGyro(mode)
        _, graw, _ = read_raw(300)
        d = (graw - GYRO_BIAS) / GYRO_SENS
        worst = np.abs(d).max()
        mark = "  <== ĐÚNG" if worst < 0.5 else ""
        note = "" if abs(sens_true - GYRO_SENS) < 0.1 else f"  (sens phải là {sens_true})"
        print(f" {name:>8} | {GYRO_SENS:6.1f} | {d[0]:+8.3f} {d[1]:+8.3f} {d[2]:+8.3f}       |"
              f" {worst*60:6.1f}°{mark}{note}")
    print("\n  Lưu ý: gyro_sensitivity trong JSON phải khớp FSR:")
    print("         dps250->131.0   dps500->65.5   dps1000->32.8   dps2000->16.4")


# ================= TEST 4: SCALE GYRO BẰNG CÁCH XOAY =================
def test_spin():
    print("\n" + "=" * 66)
    print(" TEST SCALE GYRO - tích phân vận tốc góc khi xoay tay")
    print(" Nhấn ENTER, xoay IMU đúng 90° quanh 1 trục rồi đặt yên, nhấn ENTER lại.")
    print(" Nếu ra ~180° -> FSR sai gấp đôi. Nếu ra ~45° -> sai nửa.")
    print("=" * 66)
    for name, (mode, sens_true) in GYRO_FSR.items():
        if abs(sens_true - GYRO_SENS) < 0.1:
            IMU.setFullScaleRangeGyro(mode)
            print(f"\n Đang dùng FSR = {name} (khớp sensitivity {GYRO_SENS} trong JSON)")
            break
    else:
        print(f"\n ⚠ sensitivity {GYRO_SENS} không khớp FSR chuẩn nào!")
        IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)

    input("\n Nhấn [ENTER] để bắt đầu tích phân...")
    ang = np.zeros(3)
    t0 = last = time.time()
    import select
    while not select.select([sys.stdin], [], [], 0)[0]:
        if IMU.dataReady():
            IMU.getAgmt()
            now = time.time()
            dt, last = now - last, now
            d = (np.array([IMU.gxRaw, IMU.gyRaw, IMU.gzRaw], float) - GYRO_BIAS) / GYRO_SENS
            ang += d * dt
            print(f"\r  góc tích phân: X={ang[0]:+7.2f}°  Y={ang[1]:+7.2f}°  Z={ang[2]:+7.2f}°"
                  f"   (t={now-t0:.1f}s)  [ENTER để dừng]", end="", flush=True)
        time.sleep(0.005)
    sys.stdin.readline()
    print(f"\n\n  Kết quả: X={ang[0]:+.1f}°  Y={ang[1]:+.1f}°  Z={ang[2]:+.1f}°")
    print("  Trục bạn vừa xoay phải đọc ≈ ±90°.")


TESTS = {"accel": test_accel, "tilt": test_tilt, "gyro": test_gyro, "spin": test_spin}

if __name__ == "__main__":
    print(f"\n Calib đang nạp: SM[0][0]={SM[0][0]:.6g}  bias={ACCEL_BIAS.round(3)}")
    print(f"                 gyro_bias={GYRO_BIAS.round(2)}  sens={GYRO_SENS}")
    if len(sys.argv) > 1 and sys.argv[1] in TESTS:
        TESTS[sys.argv[1]]()
    else:
        print("\n  1) accel - dò FSR đúng cho accel  (CHẠY CÁI NÀY TRƯỚC)")
        print("  2) tilt  - đọc góc live, accel thuần, không trễ")
        print("  3) gyro  - drift khi đứng yên")
        print("  4) spin  - kiểm tra scale gyro bằng xoay 90°")
        c = input("\n Chọn (1-4): ").strip()
        TESTS[["accel", "tilt", "gyro", "spin"][int(c) - 1]]()
