import qwiic_icm20948
import time
import json
import numpy as np
import sys

# --- BƯỚC 1: TẢI THÔNG SỐ TỪ FILE JSON ---
CALIB_FILE = "vinh_accel_calib.json"

try:
    with open(CALIB_FILE, "r") as f:
        calib_data = json.load(f)
        
        # Lấy dữ liệu dạng mảng list và ép nó thành ma trận numpy (để tính toán cho lẹ)
        SM = np.array(calib_data["accel"]["SM"])
        Bias = np.array(calib_data["accel"]["bias"])
        
    print(" Đã load thành công file calib!")
except FileNotFoundError:
    print(f" Không tìm thấy file {CALIB_FILE}. Hãy chạy chương trình calib trước nhé!")
    sys.exit(1)


# --- BƯỚC 2: KHỞI ĐỘNG CẢM BIẾN ---
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print(" Không tìm thấy IMU ICM20948!")
    sys.exit(1)
    
IMU.begin()
print(" Cảm biến đã sẵn sàng. Đang đọc dữ liệu...\n")


# --- BƯỚC 3: VÒNG LẶP ĐỌC VÀ BÙ TRỪ DỮ LIỆU ---
try:
    while True:
        if IMU.dataReady():
            IMU.getAgmt()
            
            # 1. Đọc giá trị raw (đơn vị đang là LSB gốc)
            ax_raw = IMU.axRaw
            ay_raw = IMU.ayRaw
            az_raw = IMU.azRaw
            
            # Đóng gói thành một vector cột bằng numpy
            raw_accel = np.array([ax_raw, ay_raw, az_raw])
            
            # 2. CÔNG THỨC TOÁN HỌC ÁP DỤNG CALIB
            # Corrected = (SM nhân với Raw) - Bias
            # Hàm np.dot() sinh ra để nhân ma trận 3x3 với vector 3x1
            corrected_accel = np.dot(SM, raw_accel) - Bias
            
            # Tách dữ liệu ra các biến x, y, z để dễ sử dụng
            ax_clean = corrected_accel[0]
            ay_clean = corrected_accel[1]
            az_clean = corrected_accel[2]
            
            # 3. In ra màn hình kiểm tra (làm tròn 3 số thập phân cho đẹp)
            print(f"Accel (m/s^2) -> X: {ax_clean:7.3f} | Y: {ay_clean:7.3f} | Z: {az_clean:7.3f}", end="\r")
            
        time.sleep(0.05) # Ngủ 50ms (Đọc ở tốc độ 20Hz)

except KeyboardInterrupt:
    print("\n\nĐã dừng chương trình. Chúc bạn code bipedal vui vẻ!")