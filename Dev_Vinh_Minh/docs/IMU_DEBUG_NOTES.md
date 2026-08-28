# Sổ tay Debug IMU (ICM-20948 + Madgwick)

> Tổng hợp các vấn đề đã gặp khi debug IMU và cách xử lý. Giải thích kiểu dễ hiểu.
> Cảm biến: **ICM-20948** (thư viện Python `qwiic_icm20948`). Bộ lọc: **Madgwick**
> (`ahrs.filters.Madgwick`, chỉ dùng gyro + accel, KHÔNG magnetometer).
> File liên quan: `Debug_Calibration/gyroacce.py`, `gyroaccedebug.py`,
> `calib_gyro_only.py`, `calib_accel_only.py`, `plot_imu.py`.

---

## 1. Lỗi `AttributeError: 'QwiicIcm20948' object has no attribute 'dps500'`

**Nguyên nhân:** hằng số FSR (`dps250/dps500/dps1000/dps2000`, `gpm2/gpm4/gpm8/gpm16`)
nằm ở **cấp module**, KHÔNG nằm trong object `IMU`.

- ❌ Sai: `IMU.setFullScaleRangeGyro(IMU.dps500)`
- ✅ Đúng: `IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)`

**Cách nhớ:** hằng số là của **cả thư viện** (`qwiic_icm20948.…`), lấy từ "cái tủ lớn",
không phải "ngăn kéo" của một cục cảm biến (`IMU.…`).

**Kiểm chứng thư viện thật trên Pi** (đừng đoán, hãy soi):
```bash
python3 -c "import qwiic_icm20948 as q; print([n for n in dir(q) if any(k in n.lower() for k in ('dps','gpm'))])"
python3 -c "import qwiic_icm20948 as q; print([n for n in dir(q.QwiicIcm20948) if 'ullscale' in n.lower()])"
```
Kết quả đã xác nhận: `dps*/gpm*` ở cấp module; hàm `setFullScaleRangeAccel/Gyro` có thật.

### FSR ↔ Sensitivity (BẮT BUỘC khớp giữa lúc calib và lúc chạy)

| Gyro FSR | Sensitivity (LSB/(°/s)) | Accel FSR | Sensitivity (LSB/g) |
|---|---|---|---|
| ±250 dps | 131 | ±2g | 16384 |
| ±500 dps | **65.5** | ±4g | 8192 |
| ±1000 dps | 32.8 | ±8g | 4096 |
| ±2000 dps | 16.4 | ±16g | 2048 |

- Đang dùng **gyro ±500 dps → sensitivity 65.5**.
- **Quy tắc vàng:** FSR lúc calib PHẢI bằng FSR lúc chạy. Lệch FSR → mọi số gyro
  sai theo tỉ lệ. Nếu code KHÔNG gọi `setFullScaleRange…`, cảm biến dùng **mặc định
  của `begin()`** (gyro mặc định ±250 → sensitivity 131, không phải 65.5!).

---

## 2. Roll (hoặc pitch) bị lệch — khoanh vùng lỗi

### Phân loại triệu chứng trước

| Triệu chứng | Vùng nghi ngờ |
|---|---|
| Đứng yên mà roll ≠ 0, lệch **cố định** | 🎯 Accel (calib / lắp đặt) |
| Roll **trôi dần** dù đứng yên | Gyro bias / beta Madgwick |
| Roll **nhảy loạn**, nhiễu | Accel rung / beta lớn / lỗi timing |
| Nghiêng **pitch** mà roll đổi theo | Trục lắp lệch / cross-axis (SM sai) |

**Nguyên lý:** đứng yên → Madgwick dùng **trọng lực** (accel) làm mốc cho roll/pitch.
Nên lệch **tĩnh** = lỗi vector trọng lực mà accel đo ra → thuộc **nửa accel**, không phải gyro.
Gyro bias chỉ gây **trôi**, không gây lệch cố định.

### 🔪 Mẹo chia đôi: tách "accel" khỏi "Madgwick"

Từ CSV, tính roll **thuần accel** (không qua Madgwick) rồi so với cột `roll`:
```
roll_acc = atan2(ay, az) * 180/pi
```
- `roll_acc` **cũng lệch** → lỗi ở **nửa accel** (calib SM/bias, hoặc lắp vênh).
- `roll_acc` **đúng**, chỉ `roll` (Madgwick) lệch → lỗi ở **nửa Madgwick** (beta, dt, gyro).

### Hai số cần soi khi đứng yên (nếu nghi accel)

1. Độ lớn trọng lực: `√(ax²+ay²+az²)` phải ≈ **9.81**. Lệch xa → SM/bias sai hoặc FSR sai.
2. Trên mặt cân: trục đứng ≈ ±9.81, **2 trục ngang ≈ 0**. Trục ngang ≠ 0 → lắp vênh
   hoặc SM nuốt sai misalignment.

> FSR sai **đều cả 3 trục** thì roll KHÔNG lệch (vì `atan2` là tỉ lệ, scale chung triệt tiêu).
> FSR chỉ gây lệch roll khi 3 trục scale **khác nhau**.

---

## 3. Bug "45° rồi tụt loạn ngẫu nhiên" (random, lúc bị lúc không)

**Chốt bản chất:** random = lỗi **timing / I2C**, KHÔNG phải calib (calib sai thì sai đều).

**Cơ chế (2 kiểu đứt "neo trọng lực"):**
1. **`dt` spike:** vòng lặp khựng 1 nhịp → `dt` từ 0.02s vọt lên 0.2–0.5s → Madgwick lấy
   `gyro × dt_to` → tưởng vừa quay lớn → bắn góc loạn.
2. **Accel trả rác** (I2C đọc lỗi → 0 hoặc số cũ) → mất mốc trọng lực → Madgwick chạy
   **gyro-only** → gyro có bias → góc **trôi tụt dần**; mẫu tốt kế tiếp accel giật lại → hỗn loạn.

**Phân biệt quan trọng về `dt`:**
- `dt` thay đổi **nhẹ** mỗi vòng (0.019–0.021) → **BÌNH THƯỜNG**. Madgwick sinh ra để nhận
  `dt` đo thật; đưa `dt` đo được là đúng.
- `dt` thỉnh thoảng **vọt to** → **ĐÂY mới là thủ phạm**.

**Xác nhận (từ CSV, đúng đoạn loạn):** cột `dt` có số ≥ 0.1 không? `√(ax²+ay²+az²)` có
tụt về ~0 hay nhảy vọt không?

### Phương án fix (ưu tiên trên xuống)
1. **Bỏ `csv_file.flush()` mỗi mẫu** (nghi phạm #1). Ép ghi thẻ SD mỗi 20ms → thẻ SD trên
   Pi thỉnh thoảng treo vài trăm ms → chính là `dt` spike. Đổi thành flush mỗi ~50 mẫu +
   flush lần cuối ở `finally`.
2. **Chặn `dt` xấu:** nếu `dt` > ngưỡng (vd 0.1s) thì bỏ mẫu / kẹp `dt` về trần.
3. **Chặn accel rác:** tính norm; nếu ngoài khoảng hợp lý (vd 0.5g–1.5g) thì skip update.
   Bọc `getAgmt()` trong try/except; lỗi I2C → bỏ mẫu.

---

## 4. Hội tụ chậm lúc khởi động (cold-start ramp)

**Hiện tượng thấy trong log:** bật khi IMU đã nằm ở 45° → đồ thị **bò dần 0 → góc thật
trong ~10 giây** rồi mới chốt. Đây KHÔNG phải cảm biến di chuyển, cũng KHÔNG phải lỗi chất
lượng cảm biến.

**Madgwick có 2 "đường" biết hướng:**
- **Gyro** = đo tốc độ quay → bám chuyển động **gần tức thời** (đường **nhanh**).
- **Accel** = đo trọng lực chỉ đâu → sửa sai, nhưng nhân `beta=0.033` rất nhỏ → kéo **rất chậm**.

**Vì sao ramp 10s:** Madgwick khởi tạo `q=[1,0,0,0]` = đoán **đang phẳng**. Bật sẵn ở 45°
→ lệch to giữa "nó nghĩ" (0°) và "sự thật" (45°). Nhưng IMU **nằm im** → gyro ~0 → đường
nhanh vô dụng → chỉ còn accel chậm rề rề kéo lên → 10 giây. Đây là **cold-start**, xảy ra
đúng 1 lần lúc bật.

**Khi bật ở phẳng rồi mới nghiêng thì KHÔNG bị ramp:**
- Bật ở phẳng: filter đoán phẳng, thật cũng phẳng → khớp ngay.
- Nghiêng tay tới góc: đây là **cú quay thật** → gyro thấy ngay → Madgwick bám kịp thời gian thực.
- ⇒ Ramp chậm **chỉ sinh ra khi có lệch mà gyro không thấy** (bật nguội ở tư thế nghiêng).
  Robot chạy thật nghiêng bằng chuyển động → luôn có gyro → không gặp.

**Cách test đúng:** luôn **bật ở phẳng, chờ ~5–10s cho ổn, RỒI mới nghiêng** lên góc.
Đừng bật sẵn ở góc.

**Diệt hẳn cold-start (tùy chọn):** thay vì `q=[1,0,0,0]`, tính roll/pitch từ **mẫu accel
đầu tiên** rồi đặt `q0` bằng đúng hướng đó → filter bắt đầu đã đúng chỗ, không ramp dù bật
ở góc nào. *(Chưa áp dụng vào code.)*

---

## 5. Tần số lấy mẫu — cái gì ghì nó lại (trong `gyroaccedebug.py`)

Tần số **không set cố định ở đâu**; `dt` đo thật. Tần số thật = vòng lặp chạy được sau khi
trừ các phanh sau (nặng → nhẹ):

1. **`time.sleep(0.02)`** → trần cứng ~**50Hz**.
2. **Khối `print` khổng lồ mỗi mẫu** (~25 dòng) → phanh nặng + gây **spike `dt`** (in ra
   SSH/terminal chậm và hay khựng). `gyroacce.py` đã bỏ, chỉ còn heartbeat mỗi 50 mẫu.
3. **`IMU.dataReady()`** → phụ thuộc **ODR nội bộ** cảm biến (do `begin()` đặt) + DLPF.
4. **Thời gian đọc I2C `getAgmt()`** + tốc độ bus (100k/400kHz).

---

## 6. Ghi log CSV & vẽ đồ thị

- **Ghi CSV:** chỉ có trong `gyroacce.py`. Tên file đặt ở dòng ~82:
  `csv_path = LOG_DIR / f"gyroacce_{time.strftime('%Y%m%d_%H%M%S')}.csv"` — timestamp để
  không đè file cũ, lưu trong `logs/` cạnh script. 21 cột: `t, dt, *_raw, *_deg, ax/ay/az,
  quaternion, roll/pitch/yaw`.
- **Vẽ:** `plot_imu.py` (chạy trên laptop). Chỉ vẽ roll/pitch/yaw theo thời gian, không thêm gì.
  ```bash
  python3 plot_imu.py                 # vẽ tất cả CSV trong logs/
  python3 plot_imu.py logs/45deg.csv  # vẽ file chỉ định
  ```
  Xuất PNG cùng tên cạnh CSV. (Cần `matplotlib`; laptop đã cài.)

---

## 7. Khác biệt giữa 2 file runtime (cần gộp về 1 chuẩn)

| | `gyroacce.py` | `gyroaccedebug.py` |
|---|---|---|
| Ghi CSV | ✅ có | ❌ không |
| Khối print to | ❌ đã bỏ (chỉ heartbeat) | ✅ vẫn còn (phanh nặng) |
| `setFullScaleRangeAccel(gpm4)` | ❌ chưa có | ✅ có |
| ALPHA low-pass | 1.0 (tắt lọc) | 0.2 (có lọc) |

> Nên chọn 1 bản làm chuẩn rồi đồng bộ (CSV + không print nặng + set đủ FSR accel + ALPHA thống nhất).

---

## 8. Việc còn tồn (TODO)

- [ ] **Calib lại accel** trên mặt phẳng chuẩn (log `0deg` đang lệch: pitch −11.5°, roll −4°).
- [ ] **Sai số accuracy:** log 45° chốt ở ~36° (thiếu ~9°) → soi scale/calib sau khi calib lại.
- [ ] **Pin accel FSR** (`setFullScaleRangeAccel(qwiic_icm20948.gpm4)`) vào cả `gyroacce.py`
      và `calib_accel_only.py`, rồi calib lại accel (đổi FSR ⇒ SM cũ vô nghĩa).
- [ ] Cân nhắc: bỏ flush mỗi mẫu + chặn `dt`/accel xấu (mục 3) để diệt bug nhảy loạn.
- [ ] Cân nhắc: khởi tạo `q0` từ accel đầu tiên để bỏ cold-start (mục 4).
- [ ] Gộp `gyroacce.py` / `gyroaccedebug.py` về 1 chuẩn (mục 7).

Xem thêm: [IMU_TEST_PLAN.md](IMU_TEST_PLAN.md) (kế hoạch test precision/accuracy/drift + fusion 2 IMU).