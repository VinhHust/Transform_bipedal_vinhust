import qwiic_icm20948
import json #thư viện giúp tạo và đọc file JSON
import time
import math
from pathlib import Path

#CHẠY CALIB:python3 calib_gyro.py

print(" KEEP IMU COMPLETELY STILL FOR 5 SECONDS!")
print("Don't touch or move it at all!\n")

IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    print("IMU not found!")
    exit(1)

IMU.begin()

#cố định FSR Gyro về dps 500 để khớp với gyroacce.ơy
IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)  # hằng số ở cấp module, KHÔNG phải IMU.dps500
GYRO_SENSITIVITY = 65.5   # LSB/(°/s) — cố định theo bảng datasheet cho ±500 dps

# DLPF phần cứng: bắt buộc trùng với lúc ĐỌC. begin() tắt DLPF -> ODR ~4.5kHz
# -> torn read. Bật 'low' hạ ODR về ~1.1kHz, đã chứng minh khử torn read và
# giảm nhiễu 4 lần (xem docs/analysis.md).
IMU.setFullScaleRangeAccel(qwiic_icm20948.gpm4)
IMU.setDLPFcfgAccel(qwiic_icm20948.acc_d23bw9_n34bw4)
IMU.setDLPFcfgGyro(qwiic_icm20948.gyr_d23bw9_n35bw9)
IMU.enableDlpfAccel(True)
IMU.enableDlpfGyro(True)

# Đợi 5 giây để IMU ổn định
for i in range(5, 0, -1):
    print(f"Starting calibration in {i} seconds...")
    time.sleep(1)

SAMPLES = 1000 #Đọc dữ liệu 1000 lần
SAMPLE_RATE = 0.02 # mỗi lần đọc cách nhau 20ms (50Hz)
#tổng cộng mất 20 giây để đọc dữ liệu

STD_MAX = 40.0   # LSB — ngưỡng coi là bị động đậy (nền nhiễu thật ~18 LSB)
P2P_MAX = 500.0  # LSB — một cú chạm tay hiện ra ở đây rõ hơn ở std

print(f"\n Calibrating gyroscope (reading {SAMPLES} samples, ~{SAMPLES * SAMPLE_RATE:.0f}s)...\n")
 
gx_sum, gy_sum, gz_sum = 0, 0, 0 #biến sum để cộng dồn tổng dữ liệu
#biến min max để lưu giá trị lớn nhấtt và nhỏ nhất của dữ liệu  
gx_min, gx_max = float('inf'), float('-inf')
gy_min, gy_max = float('inf'), float('-inf')
gz_min, gz_max = float('inf'), float('-inf')

gx_data, gy_data, gz_data = [], [], [] #tạo mảng để đưa các con số đọc được vào đấy 

for i in range(SAMPLES):
    if IMU.dataReady():
        IMU.getAgmt()
        #Lấy giá trị raw của gyrscope 
        gx_raw = IMU.gxRaw
        gy_raw = IMU.gyRaw
        gz_raw = IMU.gzRaw
        #Cộng dồn giá trị raw của gyro 
        gx_sum += gx_raw
        gy_sum += gy_raw
        gz_sum += gz_raw

        #lưu giá trị raw vào mảng ở trên 
        gx_data.append(gx_raw)
        gy_data.append(gy_raw)
        gz_data.append(gz_raw)

        #hàm min max để tìm giá trị lớn nhất và nhỏ nhất của dữ liệu trong toàn bộ quá trình đọc dữ liêu, bằng cách so sánh gtri hiện tại với min max của vòng trc đó 
        gx_min, gx_max = min(gx_min, gx_raw), max(gx_max, gx_raw)
        gy_min, gy_max = min(gy_min, gy_raw), max(gy_max, gy_raw)
        gz_min, gz_max = min(gz_min, gz_raw), max(gz_max, gz_raw)

        #gom đủ 100 mẫu thì mới in lên 1 lần, tránh việc in quá nhiều lần làm chậm chương trình
        if (i + 1) % 100 == 0:
            percent = (i + 1) / SAMPLES * 100
            elapsed = (i + 1) * SAMPLE_RATE
            print(f"  {i + 1}/{SAMPLES} samples ({percent:.0f}%) - {elapsed:.1f}s elapsed...", end='\r')
    
    time.sleep(SAMPLE_RATE) #bắt chương trình nghỉ 0.02 giây giữa mỗi vòng lặp for 

print(f"\n   Sampling complete!                                            ")

#Chia cho SỐ MẪU THỰC, không phải SAMPLES: dataReady() có thể trả False và
#vòng lặp bỏ qua mẫu đó, chia cho SAMPLES sẽ kéo bias về 0 một cách âm thầm.
N = len(gx_data)
if N == 0:
    raise SystemExit("Không đọc được mẫu nào - kiểm tra I2C.")

#Tính trung bình cộng giá trị của dữ liệu đọc được, đây chính là bias của gyro
gx_bias = gx_sum / N
gy_bias = gy_sum / N
gz_bias = gz_sum / N

#PHẦN NÀY ĐỂ TÍNH VARIANCE, KHÔNG LIÊN QUAN TỚI BIAS, CHỈ ĐỂ XEM DỮ LIỆU CÓ ỔN ĐỊNH HAY KHÔNG
gx_variance = sum((x - gx_bias)**2 for x in gx_data) / N
gy_variance = sum((y - gy_bias)**2 for y in gy_data) / N
gz_variance = sum((z - gz_bias)**2 for z in gz_data) / N

gx_std = math.sqrt(gx_variance)
gy_std = math.sqrt(gy_variance)
gz_std = math.sqrt(gz_variance)

print(f"\n✅ Gyroscope Bias calculated  ({N}/{SAMPLES} mẫu đọc được):")
print(f"           bias LSB    bias dps     std LSB   p2p LSB")
ok = True
for ax, b, s, lo, hi in (("gx", gx_bias, gx_std, gx_min, gx_max),
                         ("gy", gy_bias, gy_std, gy_min, gy_max),
                         ("gz", gz_bias, gz_std, gz_min, gz_max)):
    p2p = hi - lo
    bad = s > STD_MAX or p2p > P2P_MAX
    ok = ok and not bad
    print(f"  {ax}   {b:+10.2f} {b/GYRO_SENSITIVITY:+11.4f}   {s:9.2f} {p2p:9.0f}"
          f"   {'<-- ĐỘNG ĐẬY' if bad else ''}")

#Nền nhiễu đo được ở soak DLPF=low: std [18.1, 8.8, 3.6] LSB, p2p [344,112,29].
#Vượt xa mức đó nghĩa là IMU bị chạm/rung trong lúc lấy mẫu -> bias sai.
if not ok:
    print(f"\n⚠️  KHÔNG ĐẠT - std > {STD_MAX} hoặc p2p > {P2P_MAX} LSB.")
    print(f"   IMU bị chạm hoặc rung trong lúc lấy mẫu. Chạy lại, đừng đụng vào.")
    print(f"   (nền nhiễu bình thường: std ~[18, 9, 4] LSB)")
else:
    print(f"\n   ĐẠT - đứng yên trong suốt {N * SAMPLE_RATE:.0f}s lấy mẫu.")

#Bias không sửa thì tích phân thẳng thành góc trôi - đây là số đo mức độ nghiêm trọng
d60 = max(abs(gx_bias), abs(gy_bias), abs(gz_bias)) / GYRO_SENSITIVITY * 60
print(f"   Nếu KHÔNG trừ bias: trôi {d60:.1f}° sau 60s tích phân.")

# TẠO MỚI DỮ LIỆU CALIB TỪ ĐẦU (Không đọc file cũ)
calib = {
    "gx_bias": gx_bias,
    "gy_bias": gy_bias,
    "gz_bias": gz_bias,
    "gyro_sensitivity": GYRO_SENSITIVITY
}

#Ghi theo đường dẫn tuyệt đối tính từ vị trí file script, KHÔNG theo thư mục
#đang đứng - trước đây chạy từ thư mục khác là file rơi nhầm chỗ.
OUTPUT_FILE = Path(__file__).parent.parent / "READ" / "vinhgyrocalib.json"
with open(OUTPUT_FILE, "w") as f:
    json.dump(calib, f, indent=2)

print(f"\n✓ Calibration saved to {OUTPUT_FILE}")
print(f"\n Summary:")
print(f"  • Samples: {N}/{SAMPLES}")
print(f"  • Duration: {N * SAMPLE_RATE:.1f}s")
print(f"  • Gyro Sensitivity: {GYRO_SENSITIVITY}")
