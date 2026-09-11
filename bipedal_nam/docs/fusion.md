# Giai đoạn D — chuyển sang imufusion

> Chi tiết của `plan.md` §D. Lý thuyết so sánh thuật toán: `madgwick-vs-fusion.md`.
>
> **Phạm vi hiện tại: chỉ offline.** Chưa đụng vào `leg_server_left.py`.

| Bước | Nội dung | Trạng thái |
|---|---|---|
| D1 | Cài + xác minh API | ⬜ |
| D2 | Công cụ replay offline | ⬜ |
| D3 | Chạy trên log có sẵn, tune | ⬜ |
| D4 | Nghiệm thu so với Madgwick | ⬜ |
| D5 | Đưa vào `leg_server_left.py` | ⬜ — **để sau, không thuộc đợt này** |

Không cần thu dữ liệu mới. Log `angles_20260831_183200` đã có sẵn: **712 giây
liên tục ở DLPF=low**, 35 024 mẫu, gần như đứng yên (`roll_std 0.139°`) và có
lẫn ít chuyển động. Đủ cho D3 và phần lớn D4.

---

## Vì sao đổi, nói gọn

Madgwick chương 3 có **một núm duy nhất** là `beta`, và núm đó phải làm ba việc
mâu thuẫn nhau: hội tụ nhanh lúc khởi động, chống trôi gyro lúc chạy ổn định,
chống nhiễu do gia tốc lúc robot bước. Không tồn tại giá trị `beta` tốt cho cả
ba.

imufusion (thuật toán chương 7 — cùng tác giả, **lõi toán khác hẳn**) tách ba
việc đó thành ba cơ chế riêng: `gain` lo trạng thái xác lập, *startup ramp* lo
hội tụ, *acceleration rejection* lo lúc động.

Với robot 2 chân, cơ chế đáng giá nhất không phải** tốc độ hội tụ mà là
**acceleration rejection**. Gia tốc kế đo *specific force* = trọng lực **cộng**
gia tốc chuyển động. Mỗi bước chân là một xung gia tốc. Madgwick vẫn tin gia tốc
kế như thường và kéo ước lượng đi sai.

### Đổi sang imufusion xoá luôn 2/3 lỗi đang treo ở §D

| Lỗi cũ trong `plan.md` §D | Sau khi đổi |
|---|---|
| 1. `beta` lệch nhau giữa các script (0.033 vs 0.1) | **Biến mất** — không còn `beta` |
| 2. Quaternion khởi tạo `[1,0,0,0]` | **Được lo** — startup ramp; thêm `set_quaternion()` nếu muốn chắc |
| 3. `updateIMU()` không nhận `dt` thật | **Vẫn phải tự lo** — dùng `set_sample_period()`, xem §Bẫy 3 |
**
Lỗi 3 đã xác nhận là thật: [leg_server_left.py:321](../src/leg_server/leg_server_left.py#L321)
gọi `updateIMU()` **không truyền `dt`**, nên `ahrs` 0.4.0 dùng `self.Dt = 1/50`
cố định. Nhưng `dt` đo thật có `p99 = 41.5 ms` — gấp đôi danh nghĩa, và **8.92 %
số mẫu lệch quá 20 %**.

### Cái imufusion KHÔNG chữa được

1. **Yaw vẫn không quan sát được.** Trọng lực chỉ ràng buộc 2 trong 3 bậc tự do
   quay. Vật lý, không phải giới hạn thuật toán. Phân tích ở `plan.md` §D vẫn
   nguyên giá trị: bias `gz` còn dư 0.039 °/s → **2.3° trôi yaw mỗi phút**. Bias
   algorithm làm chậm lại, không chặn được.
2. **Vẫn phải tự calib.** Thư viện *áp dụng* tham số, không *xác định* tham số.
3. **`dt` jitter lớn do OS.** `set_sample_period()` bù được jitter *nhỏ*. Không
   cứu được cú trễ 72 ms.

---

## D1. Cài + xác minh API

```bash
pip install imufusion
```

Đã kiểm tra: PyPI có **imufusion 1.3.3** với wheel dựng sẵn
`cp312-manylinux_2_17_aarch64` — khớp `Python 3.12.3 / aarch64` của Pi này,
**không phải biên dịch**.

### Chữ ký API thật của 1.3.3 — khác tài liệu trên mạng

Tài liệu và blog thường viết `Settings` và `Offset`. Bản 1.3.3 **không có hai
tên đó**. Danh sách dưới lấy trực tiếp từ file stub trong wheel:

| Tên đúng | Tên sai hay gặp |
|---|---|
| `imufusion.AhrsSettings` | ~~`Settings`~~ |
| `imufusion.Bias` / `BiasSettings` | ~~`Offset`~~ |
| `ahrs.update_no_magnetometer(gyr, acc)` | ~~`update(...)`~~ (bản 3 tham số, có mag) |

```python
AhrsSettings(sample_rate, convention, gain, gyroscope_range,
             acceleration_rejection, magnetic_rejection, rejection_timeout)

BiasSettings(sample_rate, stationary_threshold, stationary_period)

Bias:  update(gyr) -> gyr_đã_trừ_offset  |  get_offset()  |  set_offset(v)
Ahrs:  update_no_magnetometer(gyr, acc)  |  set_sample_period(dt)
       get_quaternion() | set_quaternion(q) | get_gravity()
       get_linear_acceleration() | get_earth_acceleration()
       get_internal_states() | get_flags() | restart() | skip_startup()

imufusion.quaternion_to_euler(q) -> [roll, pitch, yaw] (độ)
imufusion.CONVENTION_NWU / _ENU / _NED
```

**Tiêu chí đạt**: đã xác minh (2026-08-31, venv `bipedal_nam/venv`).

> `inspect.signature()` **vô dụng** ở đây — 1.3.3 là extension biên dịch
> (pybind11) không kèm stub, trả về `(self, /, *args, **kwargs)` cho mọi hàm.
> Xác minh bằng cách gọi thật, không phải đọc chữ ký:

```python
s  = imufusion.AhrsSettings(50.0, imufusion.CONVENTION_NWU, 0.5, 500.0, 10.0, 0.0, 5.0)
# thứ tự đúng: sample_rate, convention, gain, gyroscope_range,
#              acceleration_rejection, magnetic_rejection, rejection_timeout
bs = imufusion.BiasSettings(50.0, 3.0, 3.0)
# thứ tự đúng: sample_rate, stationary_threshold, stationary_period
```

Cả hai constructor nhận đúng 7 và 3 tham số **vị trí**, đúng thứ tự nêu ở
D1 — kiểm bằng cách đọc lại `s.sample_rate`, `s.convention`, ... (đều là
data descriptor, không phải qua `__init__`).

---

## D2. Công cụ replay offline

### Ý tưởng

`imu_soak.py` đã ghi `all.csv` chứa **đủ mọi thứ cần thiết**: `ax_raw…gz_raw`,
`dt`, `t`, `tempC`. Nên cho **cùng một chuỗi số** chạy qua cả Madgwick lẫn
Fusion được, offline, không cần phần cứng.

Lợi ích, nói thẳng: **nhanh và lặp lại được**. Tune tham số mất vài giây thay vì
phải dựng lại board mỗi lần; chạy lại tuần sau ra đúng kết quả cũ nên so sánh
được qua thời gian; chạy trên laptop, không chiếm Pi.

### Giao diện đề xuất

`Calib/READ/replay.py`

```bash
python3 Calib/READ/replay.py logs/angles_20260831_183200

# quét tham số
python3 Calib/READ/replay.py logs/... --gain 0.3 0.5 1.0 --accel-rej 5 10 20

# thử hội tụ: cố tình khởi tạo quaternion SAI
python3 Calib/READ/replay.py logs/... --init-tilt 90
```

Ra `replay/<tên log>/`: `compare.csv` (roll/pitch/yaw của cả hai theo thời
gian), `flags.csv` (cờ nội bộ Fusion), `report.txt`.

`--init-tilt` là chi tiết đáng giá nhất: **phép thử hội tụ không cần thu dữ liệu
mới.** Chỉ cần khởi tạo quaternion lệch 90° so với thực tế rồi đo bao lâu nó về
đúng. Làm được trên bất kỳ bản ghi nào.

### Khung vòng lặp — các lời gọi API chính xác

> Đã chạy thử thật (không phải chép tài liệu) — 4 chỗ khác với bản nháp ban
> đầu, ghi lại để khỏi lặp lại:
> 1. `Ahrs`/`Bias` **không nhận `Settings` qua constructor**. Tạo rỗng rồi
>    `set_settings(s)`.
> 2. `update_no_magnetometer(gyr, acc)` **trả về chính đối tượng `Ahrs`**
>    (kiểu builder), **không trả quaternion**. Phải gọi `get_quaternion()`
>    sau đó.
> 3. Không có class `Quaternion`, không có `.to_euler()`. Dùng hàm module
>    `imufusion.quaternion_to_euler(q)`.
> 4. `internal_states` / `flags` là **method** (`get_internal_states()`,
>    `get_flags()`), không phải property.

```python
import imufusion, numpy as np

G, GS = 9.80665, 65.5

ahrs_settings = imufusion.AhrsSettings(50.0, imufusion.CONVENTION_NWU, 0.5,
                                        500.0, 10.0, 0.0, 5.0)   # xem D3
ahrs = imufusion.Ahrs()
ahrs.set_settings(ahrs_settings)

bias_settings = imufusion.BiasSettings(50.0, 3.0, 3.0)
bias = imufusion.Bias()
bias.set_settings(bias_settings)
bias.set_offset(GB / GS)                          # ← hệ số A2, đơn vị °/s

for row in rows:
    gyr = np.array([gx_raw, gy_raw, gz_raw]) / GS             # °/s, CHƯA trừ bias
    acc = (SM @ np.array([ax_raw, ay_raw, az_raw]) - B) / G    # đơn vị g

    gyr = bias.update(gyr)                        # Bias sở hữu offset
    ahrs.set_sample_period(dt_clamped)            # xem Bẫy 3
    ahrs.update_no_magnetometer(gyr, acc)          # trả về self, bỏ qua

    q     = ahrs.get_quaternion()                  # ndarray [w,x,y,z]
    euler = imufusion.quaternion_to_euler(q)        # [roll, pitch, yaw] độ
    st, fl = ahrs.get_internal_states(), ahrs.get_flags()
```

**Tiêu chí đạt**: replay một log cũ, nhánh **`tilt()`** trong replay (atan2
thuần từ trọng lực, xem [imu_soak.py:88](../Calib/READ/imu_soak.py#L88)) cho
ra roll/pitch **trùng với cột `roll`/`pitch` đã ghi trong `all.csv`**, sai lệch
< 0.01°. Đây **không phải Madgwick** — cột đó vốn được tính bằng `tilt()`, có
docstring ghi rõ "Không gyro, không Madgwick". Trùng được nghĩa là đường dẫn
dữ liệu (đọc CSV → áp `SM`/`bias` → `atan2`) đã đúng; mọi khác biệt sau đó
giữa Madgwick và Fusion là do thuật toán, không do lỗi đọc file. Nghiệm thu
Madgwick riêng: replay phải cho Madgwick **≈ giống** giá trị `ahrs` 0.4.0
đã ghi trong log gốc (không cần trùng tuyệt đối vì trạng thái nội bộ khác).

---

## D3. Cấu hình + bốn cái bẫy

### Tham số khởi điểm

| Tham số | Giá trị | Vì sao |
|---|---|---|
| `sample_rate` | `50.0` | Khớp vòng lặp hiện tại |
| `convention` | `CONVENTION_NWU` | Hệ Z-hướng-lên. **Phải xác minh** — Bẫy 4 |
| `gain` | `0.5` | Mặc định khuyến nghị |
| `gyroscope_range` | `500.0` | Khớp `dps500` đang dùng |
| `acceleration_rejection` | `10.0` | Độ. **Chưa tune được** — xem D4 |
| `magnetic_rejection` | `0.0` | Không dùng từ kế |
| `rejection_timeout` | `5.0` | Giây |
| `stationary_threshold` | `3.0` | °/s. Nền nhiễu đo được chỉ 0.27 °/s → thoải mái |
| `stationary_period` | `3.0` | Giây |

> **Cảnh báo dây chuyền**: `gyroscope_range = 500` khớp FSR `dps500` hiện tại.
> Nếu sau này thấy cờ `overrange_recovery` bật, phải nới FSR lên `dps1000` /
> `dps2000` — và **đổi FSR gyro là phải chạy lại A2**, vì `gyro_sensitivity`
> đổi theo (65.5 → 32.8 → 16.4). Calib accel không bị ảnh hưởng.

### Bẫy 1 — đơn vị. Lỗi số một khi đổi thư viện

**imufusion dùng °/s và g. Đường dẫn hiện tại của bạn dùng rad/s và m/s².**

| | Hiện tại (Madgwick) | Cần cho Fusion |
|---|---|---|
| Gyro | `(raw − GB)/65.5 × π/180` → rad/s | `raw/65.5` → **°/s**, để `Bias` trừ offset |
| Accel | `SM @ raw − B` → m/s² | `(SM @ raw − B) / 9.80665` → **g** |

Quên chia `9.80665` thì `‖a‖ = 9.81` thay vì `1.0`, và toàn bộ
`acceleration_rejection` tính theo góc trở nên vô nghĩa — **nhưng vẫn chạy, vẫn
ra số**. Đúng loại lỗi tốn nhiều ngày nhất.

> Thư viện có `imufusion.model_inertial(uncalibrated, misalignment, sensitivity,
> offset)` áp mô hình `M·s·(u − b)`. Muốn dùng thì `misalignment = SM`,
> `sensitivity = [1,1,1]`, `offset = SM⁻¹·B` (tâm ellipsoid, đã tính sẵn:
> `[3051.9, −1088.5, −353.5]` LSB). **Khuyến nghị bỏ qua** — dòng numpy hiện tại
> đã đúng và đã nghiệm thu, thêm một lớp là thêm chỗ sai.

### Bẫy 2 — vì sao vẫn phải giữ calib gyro tĩnh

Không bỏ được. Cơ chế `Bias`:

```
v = gyro − offset_hiện_tại            ← trừ ước lượng đang có TRƯỚC
nếu bất kỳ trục nào |v| > stationary_threshold:
    timer = 0 ; bỏ qua                ← coi như đang chuyển động
timer += 1
nếu timer >= stationary_period:
    offset += v × hệ_số_lọc           ← chỉ ở đây mới học
```

Ngưỡng áp lên giá trị **đã trừ offset đang có**. Lúc mới bật, `offset = 0`. Nếu
bias thô vượt 3 °/s, timer không bao giờ tích đủ → **không bao giờ học** → kẹt
vĩnh viễn, không báo lỗi gì.

Bias đo ở A2 là `[−0.048, +0.353, −0.503]` °/s, dưới ngưỡng → tự khởi động
được. Nhưng đó là **con chip này, hôm nay**. ZRO (*zero-rate output* — giá trị
gyro đọc khi thực sự đứng yên) của ICM-20948 có dung sai xuất xưởng cỡ vài °/s.
Con IMU '1' chưa calib có thể rơi trên ngưỡng.

→ `bias.set_offset()` với hệ số A2 là **bảo hiểm khởi động**, không phải thừa.

### Bẫy 3 — dt, chặn giá trị ngoại lai

`dt` đo thật: `median 20.00 ms`, `p99 41.50 ms`, `max 72.53 ms`.

Gọi `set_sample_period(dt)` mỗi vòng, nhưng **kẹp lại** trước khi truyền — một
mẫu 72 ms cho tích phân gyro nhảy gấp 3.6 lần:

```python
dt = min(max(dt_measured, 0.5 / RATE), 2.0 / RATE)   # kẹp trong [10, 40] ms
```

Đếm số lần bị kẹp và in ra báo cáo. Kẹp nhiều nghĩa là vòng lặp Python không
giữ được nhịp — đó là dữ liệu để quyết định thời điểm chuyển sang STM32, không
phải thứ để giấu đi.

### Bẫy 5 — vùng kẹp `dt` phải khớp tốc độ vòng lặp THẬT

> Bẫy này **đã cắn thật** ngày 2026-09-01, tốn hai đợt test tay và suýt dẫn tới
> một bản vá sai (đảo dấu trục Y). Ghi lại đầy đủ.

**`RATE = 50.0` không phải tốc độ phần cứng ép buộc.** Không chỗ nào cấu hình
chip chạy 50 Hz cả — đó chỉ là tốc độ mà vòng lặp Python **bận** của
`leg_server_left.py` tình cờ chạy tới (vì còn phải lo mạng, động học, ghi log).

Vòng lặp **nhẹ** thì chạy nhanh hơn nhiều. `run_imufusion_live.py` bản đầu chỉ
đọc + fusion, không làm gì khác → **~285 Hz**, `dt` thật **3.6 ms**.

Hậu quả: sàn kẹp `0.5/RATE = 10 ms` cao hơn `dt` thật → **100% số mẫu** bị kẹp
**lên**, mỗi bước tích phân gyro tưởng đã trôi qua 10 ms trong khi thật ra 3.6 ms:

| | Giá trị |
|---|---|
| Xoay tay thật | ~90° |
| Tích phân bị phồng 2.8× | ~247° |
| Vượt mốc 180° → **quấn ngược** | **−113°** |
| `yaw` thực tế in ra | **−110.5°** |

Replay lại cùng dữ liệu với `dt` thật: **+90.32°** — đúng chiều, đúng lượng, và
xác nhận luôn `gyro_sensitivity = 65.5` (sai 0.35%).

**Kiểu hỏng nguy hiểm nhất**: vẫn chạy, vẫn ra số trông hợp lý, chỉ **sai dấu** —
trông y hệt triệu chứng "sai quy ước trục", dẫn thẳng tới việc đi sửa nhầm chỗ.

Hai việc bắt buộc:

1. **Ép nhịp vòng lặp** về đúng `RATE` (`time.sleep` cho đủ chu kỳ), thay vì để
   nó chạy hết tốc lực. Vừa khớp `sample_rate` đã cấu hình, vừa khớp tốc độ thật
   `leg_server_left.py` sẽ chạy sau này.
2. **Cảnh báo khi tỉ lệ kẹp vượt ~20%.** Kẹp là cơ chế chặn *ngoại lai*; kẹp gần
   như mọi mẫu nghĩa là vùng kẹp đặt sai, không phải dữ liệu xấu. Có sẵn cảnh báo
   này thì bug trên đã lộ ngay lần chạy đầu.

### Bẫy 4 — quy ước trục: ĐÃ XÁC MINH XONG cho IMU '2'

**Kết quả (2026-09-01) — hệ trục con chip ĐÚNG, thuận tay phải, không cần sửa gì
trong code.** Nhưng **mũi tên Y in trên board chỉ ngược** chiều dương thật.

| Tư thế | raw đo được | Dự đoán nếu **+trục** lên | Dự đoán nếu **−trục** lên | Kết luận |
|---|---|---|---|---|
| Mũi tên X lên trời | +11257 | **+11256** | −5152 | mũi tên X = chip **+X** |
| Mũi tên Y lên trời | −9280 | +7116 | **−9293** | mũi tên Y = chip **−Y** ⚠ |
| Nằm phẳng | +7988 | **+7955** | −8662 | phẳng = chip **+Z** |

Chip Z không được in trên board — nhưng đo ra `+Z hướng lên khi nằm phẳng`, nên
bộ ba {mũi tên X, mũi tên Y, hướng lên} là **nghịch tay**. Đó là lỗi của **hình
in trên nhựa**, không phải của chip.

#### Cách xác định trục — dùng số học, ĐỪNG xoay tay

Giữ yên từng trục hướng thẳng lên trời ~2 giây, so raw đo được với:

```
raw_dự_đoán = SM⁻¹ · (AB ± G·û)        # û = vector đơn vị của trục đang kiểm
```

Khớp tới **1–33 LSB** — chính xác hơn hẳn xoay tay, và tránh sạch hai thứ đã làm
hỏng hai đợt test đầu: **gimbal lock** (Euler mất phân biệt roll/yaw khi
`pitch → ±90°`) và **lỗi tích phân `dt`** (Bẫy 5). Chạy bằng
`run_imufusion_live.py --axis-check`.

Không xoay, không fusion, không tích phân → không có gì để sai.

#### Nếu dữ liệu có vẻ cần lật đúng MỘT trục — thì bạn nhận dạng trục sai

`imufusion.remap(sensor, ALIGNMENT_*)` có **24 hằng số, tất cả đều là phép xoay**
— đã liệt kê hết, **không tồn tại** lựa chọn nào lật 1 trục mà giữ nguyên 2 trục
kia. Đây là **có chủ đích**, không phải thiếu sót:

> Lật đúng 1 trục là **phép soi gương** (đổi thuận tay phải → thuận tay trái).
> Cảm biến lắp lệch chỉ có thể bị **xoay**, không bao giờ bị **soi gương**.

Nên: **đừng tự đảo dấu bằng tay.** Dữ liệu chip đang thuận tay phải; lật 1 trục
biến nó thành nghịch tay, mà `CONVENTION_NWU/ENU/NED` đều thuận tay phải → sai
chiều xoay, tự tay tạo ra đúng con bug đang muốn tránh.

Lệch giữa hệ trục chip và hệ trục robot **đã có cơ chế xử lý riêng**: C2
zero-at-home, `plan.md` §C. Cơ chế đó lo được mọi phép xoay.

Quy ước của `tilt()`, để đối chiếu: **roll dương** ⟺ **+Y hướng LÊN**;
**pitch dương** ⟺ **+X chúc XUỐNG**.

### Câu hỏi mở — EMA phần mềm: bỏ hay giữ?

`leg_server_left.py` hiện lọc EMA `alpha = 0.15` cho **cả accel lẫn gyro** trước
khi vào Madgwick. Ở 50 Hz, hằng số thời gian ≈ `1/(0.15×50) = 0.13 s` — độ trễ
rất lớn cho một robot đang giữ thăng bằng.

Với Fusion, lọc trước có thể **phản tác dụng**: gyro là đường nhanh của bộ lọc,
làm trễ gyro là làm trễ toàn bộ ước lượng; và `acceleration_rejection` so sánh
**độ nghiêng accel tức thời** với ước lượng hiện tại, làm mượt accel trước là
bôi nhoè chính tín hiệu nó cần. DLPF phần cứng 24 Hz đã lọc rồi.

**Nghiêng về bỏ, nhưng đây là biến — replay chạy cả hai rồi hãy chốt.**

---

## D4. Nghiệm thu

Chạy replay trên `angles_20260831_183200` (712 s, DLPF=low).

### Đo được ngay, offline

| # | Chỉ số | Ngưỡng |
|---|---|---|
| 1 | Hội tụ từ `--init-tilt 90` | **< 5 s** (Madgwick `beta=0.1`: ~16 s) |
| 2 | Trôi yaw qua 712 s | Thấp hơn Madgwick |
| 3 | `bias.get_offset()` hội tụ về đâu | Ổn định < 60 s, **gần `[−0.048, +0.353, −0.503]` °/s** |
| 4 | Sai số roll/pitch ở đoạn đứng yên | **Không tệ hơn** Madgwick |
| 5 | `accelerometer_ignored` | **< 50 %** số mẫu |

Chỉ số 3 là phép thử đẹp nhất: bias algorithm chạy hoàn toàn độc lập, **có tự
tìm lại được con số của A2 không?** Hai phương pháp độc lập cùng ra một kết quả
là bằng chứng mạnh cho cả hai. Lệch nhiều thì một trong hai sai, và phải tìm ra
cái nào trước khi đi tiếp.

Chỉ số 5 dễ bị bỏ sót. Nếu `accelerometer_ignored` bật gần như liên tục,
`acceleration_rejection` đang quá chặt và bạn đã lặng lẽ biến hệ thành gyro
thuần — trôi tự do. Đây chính là loại chẩn đoán Madgwick chương 3 **không hề
có**, và là lý do phải log `internal_states` + `flags` ngay từ đầu.

### Chưa đo được — hoãn tới giai đoạn C

`acceleration_rejection` và `overrange_recovery` chỉ có ý nghĩa khi có **xung
gia tốc thật từ bước chân**. Lắc tay không thay thế được, và robot chưa lắp
xong. Để `acceleration_rejection = 10` (mặc định), ghi nhận là **chưa tune**, và
tune lại ở giai đoạn C bằng dữ liệu đi thật.

Đừng giả vờ đã nghiệm thu phần này.

> **Đừng dùng `gain = 0` để "thử xem gyro có tốt không".** `gain = 0` tắt luôn
> cả startup ramp lẫn toàn bộ rejection, biến nó thành gyro thuần. Tương tự
> `gyroscope_range = 0` và `rejection_timeout = 0`.

**Nếu không thắng ở chỉ số nào**: dừng, ghi lại. Có thể tham số chưa hợp, mà
cũng có thể bài toán này chưa cần Fusion — cả hai đều là kết quả hợp lệ.

---

## D5. Đưa vào leg_server — để sau

Không thuộc đợt này. Ghi lại để khỏi quên, làm khi D4 đạt **và** giai đoạn C
xong:

1. Thay `Madgwick` bằng `imufusion.Ahrs` + `imufusion.Bias`
2. Đổi đơn vị: gyro → °/s (bỏ `× π/180`), accel → g (thêm `/ 9.80665`)
3. Bỏ EMA phần mềm (nếu D3 xác nhận)
4. `set_sample_period(dt_kẹp)` mỗi vòng
5. Ghi `internal_states` + `flags` vào log — **giữ vĩnh viễn**
6. Lưu `bias.get_offset()` ra file lúc thoát, `set_offset()` lại lúc khởi động

Giao diện ra ngoài **không đổi**: `state_data["imu"]` vẫn là quaternion
`[w,x,y,z]`.

---

## Việc còn treo

- **Ghi nhiệt độ khi calib.** `calib_accel_ellipsoid.py` không ghi `tmpRaw` vào
  `poses.csv`. Hệ quả cụ thể: A4 đo được ‖a‖ lệch +0.354 % so với lúc calib, rất
  giống trôi nhiệt, **nhưng không xác nhận được** vì không có nhiệt độ lúc calib.
  Thêm một cột là xong.
- **`accel.bias` vẫn là m/s² trong `calibfull.json`.** Sau D5, người đọc dễ
  tưởng đơn vị đã là `g`. **Không dùng khoá `meta.unit`** — khoá đó đã có
  nghĩa khác: nó là **tên con chip IMU** (`--unit` trong
  [make_calibfull.py:40](../Calib/CALIB/make_calibfull.py#L40)), dùng để
  cảnh báo khi nạp nhầm hệ số của IMU khác lên Pi
  ([dòng 119-122](../Calib/CALIB/make_calibfull.py#L119-L122)). Đè lên sẽ phá
  cơ chế cảnh báo đó. Thêm khoá riêng, ví dụ `accel.bias_unit: "m/s^2"`.
- Khi port sang STM32F4: định nghĩa `FUSION_USE_NORMAL_SQRT` trong
  `FusionConfig.h`. Thư viện mặc định dùng *fast inverse square root*, tối ưu cho
  MCU **không có FPU**; STM32F4 **có** FPU nên bản thường vừa chính xác hơn vừa
  không chậm hơn.
