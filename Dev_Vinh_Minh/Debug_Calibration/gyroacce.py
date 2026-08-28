# FILE NÀY DÙNG ĐỂ CHẠY MADGWICK FUSION GYRO VÀ ACCELEROMETER
# FILE NÀY DÙNG ĐỂ FUSION GYRO VÀ ACCEL

import qwiic_icm20948
import math
import numpy as np
from ahrs.filters import Madgwick
import json
import time
import logging
import sys
import csv
from pathlib import Path

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# BỎ QUA SENSIVITY VÀ G VÌ CALIB ACCEL ĐÃ TỰ LÀM
# SENSITIVITY = 16384.0  # Accelerometer
# G = 9.81 #gia tốc trọng trường của trái đất

# LẤY FILE CALIB JSON CỦA GYRO
try:
    with open("vinhgyrocalib.json", "r") as f:
        gyro_calib = json.load(f)
        gx_bias = gyro_calib.get("gx_bias", 0)
        gy_bias = gyro_calib.get("gy_bias", 0)
        gz_bias = gyro_calib.get("gz_bias", 0)
        GYRO_SENSITIVITY = gyro_calib.get("gyro_sensitivity", 65.536)  # ±500°/s range
except FileNotFoundError:
    logger.error("Không tìm thấy file vinhgyrocalib.json")
    logger.info(" Tải thành công Gyro Calib")
    sys.exit(1)

# LẤY FILE CALIB JSON CỦA ACCEL
try:
    with open("vinh_accel_calib.json", "r") as f:
        accel_calib_data = json.load(f)
        SM = np.array(accel_calib_data["accel"]["SM"])
        accel_bias = np.array(accel_calib_data["accel"]["bias"])
        logger.info("✓ Tải thành công Accel Calib")
except FileNotFoundError:
    logger.error("Không tìm thấy file vinh_accel_calib.json")
    sys.exit(1)


# Low-pass filter
ACCEL_ALPHA = 1.0
GYRO_ALPHA = 1.0

filtered_ax, filtered_ay, filtered_az = 0.0, 0.0, 0.0
filtered_gx, filtered_gy, filtered_gz = 0.0, 0.0, 0.0

# Initialize IMU
logger.info("Initializing ICM-20948 IMU...")
IMU = qwiic_icm20948.QwiicIcm20948()

if not IMU.connected:
    logger.error("IMU not found!")
    exit(1)

IMU.begin()

IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)  # hằng số ở cấp module, KHÔNG phải IMU.dps500
logger.info("✓ ICM-20948 initialized")
logger.info(f"✓ Gyroscope Range: ±500°/s (SENSITIVITY = {GYRO_SENSITIVITY})")

# Initialize Madgwick filter
madgwick = Madgwick(beta=0.033)
madgwick.q0 = np.array([1.0, 0.0, 0.0, 0.0])

logger.info("✓ Madgwick filter initialized\n")

# ==============================
# Ghi log ra CSV (để phân tích / vẽ đồ thị sau)
# Lưu vào ./logs/ cạnh file này, tên có timestamp cho khỏi đè
# ==============================
LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
csv_path = LOG_DIR / f"gyroacce_{time.strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_path, "w", newline="", encoding="utf-8")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(
    [
        "t",  # thời gian tương đối (s) từ lúc bắt đầu
        "dt",  # khoảng thời gian giữa 2 mẫu (s)
        "gx_raw",
        "gy_raw",
        "gz_raw",  # gyro thô (LSB)
        "ax_raw",
        "ay_raw",
        "az_raw",  # accel thô (LSB)
        "gx_deg",
        "gy_deg",
        "gz_deg",  # gyro sau calib (°/s)
        "ax",
        "ay",
        "az",  # accel sau calib (m/s²)
        "qw",
        "qx",
        "qy",
        "qz",  # quaternion Madgwick
        "roll",
        "pitch",
        "yaw",  # Euler (độ)
    ]
)
csv_file.flush()
logger.info(f"✓ Ghi log CSV: {csv_path}\n")

try:
    sample_count = 0
    t_start = time.time()
    last_time = t_start
    while True:
        if IMU.dataReady():
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            IMU.getAgmt()

            # Đọc dữ liệu thô từ cảm biến
            ax_raw, ay_raw, az_raw = IMU.axRaw, IMU.ayRaw, IMU.azRaw
            gx_raw, gy_raw, gz_raw = IMU.gxRaw, IMU.gyRaw, IMU.gzRaw

            raw_accel = np.array([ax_raw, ay_raw, az_raw])
            corrected_accel = np.dot(SM, raw_accel) - accel_bias

            ax = corrected_accel[0]
            ay = corrected_accel[1]
            az = corrected_accel[2]

            #  Calibrate and convert gyroscope (CORRECT WAY)
            gx_deg = (gx_raw - gx_bias) / GYRO_SENSITIVITY
            gy_deg = (gy_raw - gy_bias) / GYRO_SENSITIVITY
            gz_deg = (gz_raw - gz_bias) / GYRO_SENSITIVITY

            gx = gx_deg * math.pi / 180
            gy = gy_deg * math.pi / 180
            gz = gz_deg * math.pi / 180

            # Low-pass filter
            filtered_ax = ACCEL_ALPHA * ax + (1 - ACCEL_ALPHA) * filtered_ax
            filtered_ay = ACCEL_ALPHA * ay + (1 - ACCEL_ALPHA) * filtered_ay
            filtered_az = ACCEL_ALPHA * az + (1 - ACCEL_ALPHA) * filtered_az

            filtered_gx = GYRO_ALPHA * gx + (1 - GYRO_ALPHA) * filtered_gx
            filtered_gy = GYRO_ALPHA * gy + (1 - GYRO_ALPHA) * filtered_gy
            filtered_gz = GYRO_ALPHA * gz + (1 - GYRO_ALPHA) * filtered_gz

            # Update Madgwick filter
            q_new = madgwick.updateIMU(
                q=madgwick.q0,
                gyr=np.array([filtered_gx, filtered_gy, filtered_gz]),
                acc=np.array([filtered_ax, filtered_ay, filtered_az]),
                dt=dt,
            )
            madgwick.q0 = q_new

            qw, qx, qy, qz = q_new

            # Tính Roll (X-axis) - Nghiêng trái/phải
            sinr_cosp = 2 * (qw * qx + qy * qz)
            cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

            # Tính Pitch (Y-axis) - Ngả tới/lui
            sinp = 2 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = math.copysign(90.0, sinp)  # Giới hạn ở 90 độ
            else:
                pitch = math.asin(sinp) * (180.0 / math.pi)

            # Tính Yaw (Z-axis) - Xoay vòng quanh trục
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

            sample_count += 1

            # ✅ Ghi 1 dòng số liệu vào CSV
            csv_writer.writerow(
                [
                    f"{current_time - t_start:.4f}",
                    f"{dt:.4f}",
                    gx_raw,
                    gy_raw,
                    gz_raw,
                    ax_raw,
                    ay_raw,
                    az_raw,
                    f"{gx_deg:.4f}",
                    f"{gy_deg:.4f}",
                    f"{gz_deg:.4f}",
                    f"{ax:.4f}",
                    f"{ay:.4f}",
                    f"{az:.4f}",
                    f"{qw:.6f}",
                    f"{qx:.6f}",
                    f"{qy:.6f}",
                    f"{qz:.6f}",
                    f"{roll:.4f}",
                    f"{pitch:.4f}",
                    f"{yaw:.4f}",
                ]
            )
            csv_file.flush()  # flush ngay để đọc được log khi script vẫn đang chạy

            # Nhịp tim nhẹ: in mỗi 50 mẫu để biết còn chạy (không làm chậm vòng lặp)
            if sample_count % 50 == 0:
                print(
                    f"#{sample_count}  roll={roll:+6.1f}  pitch={pitch:+6.1f}  yaw={yaw:+6.1f}",
                    flush=True,
                )

            time.sleep(0.02)

except KeyboardInterrupt:
    logger.info("\nStopped by user")
    print(f"\n{'='*70}")
    print(f"Total samples read: {sample_count}")
    print(f"{'='*70}")

finally:
    csv_file.close()
    logger.info(f"✓ Đã lưu {sample_count} mẫu vào {csv_path}")
