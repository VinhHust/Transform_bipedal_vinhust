# Bản đồ code — file nào làm gì

> Chi tiết từng dòng nằm trong comment ngay tại file code. File này chỉ trả
> lời: **file này tồn tại để làm gì, ai gọi nó, nó đẻ ra cái gì.**
>
> Lý thuyết đằng sau phần calib: xem `wonder.md`.

---

## 1. Luồng dữ liệu tổng thể

```
                    ICM-20948 (I2C 0x69)
                            │  raw LSB
                            v
        ┌───────────────────────────────────┐
        │  CALIB  (chay 1 lan, tao tham so) │
        │                                   │
        │  calib_gyro.py            ────────┼──> vinhgyrocalib.json
        │  calib_accel_ellipsoid.py ────────┼──> vinh_accel_calib.json
        └───────────────────────────────────┘
                            │
                            │  (gop tay lai)
                            v
                     calibfull.json
                            │
        ┌───────────────────┴───────────────────┐
        │                                       │
        v                                       v
  READ/ (tren ban, de kiem tra)          leg_server_left.py
  - imu_soak.py    chan doan             (chay tren Pi, phuc vu socket)
  - check_sensor.py kiem tung khoi               │
  - gyroacce.py    fusion 1 IMU                  │ TCP
  - readaccel/readgyro  doc tho                  v
                                         sensors/imu.py
                                         IMUController + IMUFusion
                                         (chay tren laptop, gop 2 IMU)
                                                 │
                                                 v
                                         bipedal.py / balance / gait
```

---

## 2. `Calib/CALIB/` — tạo tham số hiệu chuẩn

| File | Vai trò | Đẻ ra | Trạng thái |
|---|---|---|---|
| `calib_gyro.py` | Đo bias gyro khi đứng yên | `vinhgyrocalib.json` | Dùng được, cần chạy lại với `dlpf low` |
| `calib_accel.py` | Calib accel 6 mặt | `vinh_accel_calib.json` | **Nên bỏ** — xem `wonder.md` §2 |
| `calib_accel_ellipsoid.py` | Calib accel ellipsoid fit | `vinh_accel_calib_ellipsoid.json` | **Dùng cái này** |

### `calib_gyro.py` (109 dòng)

Đặt IMU đứng yên, đọc 1000 mẫu trong 20 s, lấy trung bình. Trung bình đó
chính là bias — vì khi đứng yên, vận tốc góc thật bằng 0, nên mọi thứ đọc
được đều là sai lệch.

Cố định `dps500` (65.5 LSB/(°/s)). Cũng tính std để bạn biết dữ liệu có ổn
không, nhưng std **không** dùng vào công thức.

**Điểm yếu:** chưa bật DLPF, và gyro bias trôi theo nhiệt độ nên số này chỉ
đúng ở nhiệt độ lúc calib.

### `calib_accel.py` (132 dòng) — ĐÃ LỖI THỜI

Phương pháp 6 mặt. Dựng hệ `M·x = R` với 12 ẩn (9 phần tử `SM` + 3 `bias`),
giải bằng `numpy.linalg.lstsq`.

**Vấn đề:** dòng 78 khai cứng hướng trọng lực → sai số gá đặt chui thẳng vào
`SM`. Đo được **0.79°** trên IMU #1. Giữ file lại để đối chiếu, đừng dùng nữa.

### `calib_accel_ellipsoid.py` (~230 dòng) — MỚI

Cùng mục đích, khác **ràng buộc**: chỉ khai `‖a‖ = 9.80665`, không khai hướng.
Bàn nghiêng bao nhiêu cũng không ảnh hưởng.

Cách chạy: lăn board qua ~30 tư thế rải đều (6 mặt + 12 cạnh + 8 góc). Script
tự phát hiện lúc đứng yên (std raw accel và gyro dưới ngưỡng trong 25 mẫu) và
tự lấy mẫu. Tư thế mới phải cách tư thế cũ ≥15° mới được nhận.

Các hàm chính:

| Hàm | Làm gì |
|---|---|
| `fit_ellipsoid(P)` | Giải quadric bằng SVD → `(SM, bias)`. Lấy căn bậc hai **đối xứng** của Q để chốt nghiệm |
| `coverage(P, r0)` | Trị riêng nhỏ nhất của ma trận tán xạ hướng. `>0.10` là phủ đủ; đồng phẳng cho ~0.000 |
| `describe(SM, bias)` | Tách polar decomposition → in ra scale, cross-axis, **phần xoay bị nuốt vào SM**, bias theo mg |

Cuối phiên tự in bảng so sánh **mới vs cũ** để bạn thấy 0.79° có biến mất không.

Mặc định ghi ra file **riêng** (`..._ellipsoid.json`), không ghi đè file cũ.
Đổi sang dùng thật thì `cp` tay.

Đã kiểm chứng trên dữ liệu tổng hợp biết trước đáp án: 30 tư thế → sai `SM`
0.022%, sai `bias` 0.07 mg. Xoay cả bộ dữ liệu 5° → kết quả không đổi.

---

## 3. `Calib/READ/` — đọc và kiểm tra

| File | Vai trò | Khi nào dùng |
|---|---|---|
| `imu_soak.py` | **Công cụ chẩn đoán chính** | Nghiệm thu, săn lỗi ngẫu nhiên |
| `check_sensor.py` | Kiểm từng khối riêng lẻ | Khi nghi hỏng phần cứng |
| `acceltest.py` | In raw thô, không calib | Kiểm trục, kiểm đấu dây |
| `readaccel.py` | Accel đã calib, in liên tục | Xem nhanh |
| `readgyro.py` | Gyro đã calib, in liên tục | Xem nhanh |
| `gyroacce.py` | Fusion Madgwick 1 IMU | Bản gốc mà `leg_server` sao chép từ đó |
| `gyroaccedebug.py` | Như trên + ghi CSV | Bắt lỗi fusion |

### `imu_soak.py` (682 dòng) — quan trọng nhất

Dùng **đúng các tầng xử lý** như `gyroacce.py`, nhưng **dừng lại trước
Madgwick**. Góc tính trực tiếp từ trọng lực bằng `atan2`. Mục đích: tách bạch
lỗi cảm biến / lỗi calib khỏi lỗi fusion.

Bảy tầng:

```
1  FSR            gpm4 / dps500
2  DLPF phan cung --dlpf off|low|mid
3  doc raw        getAgmt()
4  calib accel    a = SM @ raw - bias
5  calib gyro     g = (raw - bias) / sens
6  EMA            f = 0.2*x + 0.8*f
7  goc            roll  = atan2(ay, az)
                  pitch = atan2(-ax, hypot(ay, az))
```

**Hai chế độ:**

```bash
# A) SOAK - chay dai, tu bat bat thuong ngau nhien
python3 imu_soak.py --minutes 480 --full

# B) ANGLES - do sai so roll/pitch theo goc that ban tu dat
python3 imu_soak.py --angles --dlpf low
```

Chế độ `--angles`: bạn nhập góc mong đợi `roll,pitch` → ENTER bắt đầu ghi →
ENTER lần nữa kết thúc đoạn → nhập góc tiếp. Ctrl+C cũng chốt đoạn đang ghi
(không mất dữ liệu).

**Sinh ra** `logs/<mode>_<timestamp>/`:

| File | Nội dung |
|---|---|
| `all.csv` | Mọi mẫu, 38 cột (raw, đã calib, đã lọc, góc, cờ) |
| `events.csv` | Từng sự kiện bất thường + 40 mẫu trước/sau. Lọc `pos == "EVENT"` để lấy đúng sự kiện |
| `segments.csv` | Một dòng mỗi góc đã đo (chế độ `--angles`) |
| `summary.csv` | Thống kê mỗi giây (chế độ soak) |
| `report.txt` | Tổng kết cuối phiên |

**Các loại sự kiện nó bắt:**

| Cờ | Nghĩa |
|---|---|
| `NORM` | `‖a‖` lệch khỏi 9.81 quá 0.40 m/s² |
| `JUMP` / `GJUMP` | Raw accel/gyro nhảy quá 1200/1500 LSB trong 1 mẫu |
| `TEAR` / `GTEAR` | JUMP mà độ lớn ≈ **bội số của 256** → torn read |
| `ANGLE` | Góc nhảy quá 3° trong 1 mẫu (tính trên góc **chưa lọc**) |
| `GYRO` | `‖ω‖` > 15 °/s khi lẽ ra đang đứng yên |
| `DT` | Chu kỳ lấy mẫu giãn quá 4× median |
| `STALE` | Nhiều mẫu raw giống hệt nhau liên tiếp → I2C treo |
| `MAGOVF` | Từ kế tràn — **vô hại**, dự án không dùng từ kế |

Phát hiện sự kiện chạy trên luồng **chưa lọc**, cố ý: EMA làm nhoè một glitch
1 mẫu ra ~10 mẫu và che mất dấu hiệu 256.

Nhưng số liệu trong `segments.csv` và `report.txt` lấy từ góc **đã lọc**
(`froll` / `fpitch`).

### `check_sensor.py` (195 dòng)

Bốn chế độ con, không dùng Madgwick:

```bash
python3 check_sensor.py accel   # do FSR dung cho accel
python3 check_sensor.py tilt    # goc nghieng LIVE, accel thuan
python3 check_sensor.py gyro    # drift gyro khi dung yen
python3 check_sensor.py spin    # scale gyro bang cach xoay tay
```

### `gyroacce.py` (171 dòng) và `gyroaccedebug.py` (231 dòng)

Fusion Madgwick cho **một** IMU. `gyroaccedebug.py` là bản có thêm ghi CSV.

Đây là bản gốc mà `leg_server_left.py` sao chép logic từ đó. Nếu sửa công thức
ở một nơi thì phải sửa cả nơi kia — **hiện hai nơi đang lệch nhau**, xem §6.

---

## 4. Các file JSON tham số

| File | Chứa gì | Ai tạo | Ai đọc |
|---|---|---|---|
| `Calib/READ/vinhgyrocalib.json` | `gx/gy/gz_bias`, `gyro_sensitivity` | `calib_gyro.py` | READ/*, check_sensor |
| `Calib/READ/vinh_accel_calib.json` | `accel.SM` (3×3), `accel.bias` (3) | `calib_accel*.py` | READ/*, check_sensor |
| `src/leg_server/calibfull.json` | Gộp cả hai ở trên | **Chép tay** | `leg_server_left.py` |

> **Cạm bẫy:** `calibfull.json` là bản sao chép tay. Calib lại mà quên cập
> nhật nó thì `leg_server` vẫn chạy tham số cũ, im lặng, không báo lỗi.
> (Thời điểm viết file này: đã kiểm tra, ba file đang khớp nhau.)

---

## 5. Bên tiêu thụ

### `src/leg_server/leg_server_left.py` (695 dòng)

Chạy **trên Pi**. Đọc IMU + điều khiển motor, phục vụ trạng thái qua socket TCP.

Phần IMU (dòng 268–320): nạp `calibfull.json`, đặt FSR `gpm4`/`dps500`, lọc
EMA, đẩy vào `Madgwick(frequency=50, beta=0.1)`, lưu quaternion vào
`state_data["imu"]`.

### `src/bipedal_robot/sensors/imu.py` (382 dòng)

Chạy **trên laptop**. Hai lớp:

| Lớp | Vai trò |
|---|---|
| `IMUController` | Client TCP tới một `leg_server`, nhận quaternion |
| `IMUFusion` | Nối **hai** IMU, chuyển cả hai về hệ `base_link`, gộp lại |

`IMUFusion` là nơi phép xoay lắp đặt (bước **C** trong `wonder.md`) phải được
nhập vào — `transform_quat_to_baselink()`. **Hiện chưa có số thật ở đó.**

### Phần còn lại

| Thư mục | Nội dung |
|---|---|
| `src/bipedal_robot/motor_control/` | `kinematics.py` (IK/FK), `gait_controller.py` (sinh dáng đi) |
| `src/bipedal_robot/balance/` | `stability.py` — kiểm tra ổn định |
| `src/bipedal_robot/bipedal.py` | Lớp robot tổng, `bipedal_left.py` bản một chân |
| `src/bipedal_robot/transformer.py` | 769 dòng, phép biến đổi hệ toạ độ |
| `examples/` | Script rời: dò motor, đọc vị trí, replay quỹ đạo, chạy policy |
| `tests/` | `test_gait.py`, `test_kinematics.py` |
| `config/` | Tham số mặc định |

---

## 6. Các vấn đề đã biết, chưa sửa

| # | Vấn đề | Ở đâu | Ảnh hưởng |
|---|---|---|---|
| 1 | `calib_accel.py` khai cứng hướng g | `calib_accel.py:78` | 0.79° chui vào `SM` — đã có `calib_accel_ellipsoid.py` thay |
| 2 | DLPF chưa bật ở script calib và `leg_server` | `calib_*.py`, `leg_server_left.py` | Torn read + nhiễu gấp 4 lần |
| 3 | `beta` Madgwick lệch nhau | `gyroacce.py` 0.033 vs `leg_server_left.py` 0.1 | Hai nơi cho kết quả khác nhau trên cùng dữ liệu |
| 4 | Madgwick khởi tạo `q = [1,0,0,0]` | cả hai nơi | Mất vài giây mới hội tụ; nên khởi tạo từ accel |
| 5 | `updateIMU()` không nhận `dt` thật | `leg_server_left.py:315` | Dùng `frequency=50` cố định; nhịp trượt là tích phân sai |
| 6 | `calibfull.json` chép tay | `src/leg_server/` | Calib lại mà quên chép → chạy tham số cũ, im lặng |
| 7 | Phép xoay lắp đặt chưa đo | `sensors/imu.py` | Sai số 3.4° khi tháo lắp board — xem `wonder.md` §8 mục C |
| 8 | Gá board chưa lặp lại được | cơ khí | Chặn bước C. Cần 2 vít + lỗ định vị |
