# Kế hoạch calib → fusion

> Lý thuyết: `wonder.md`. Phương pháp đo: `methods.md`. Bản đồ code: `analysis.md`.
>
> File này là **việc phải làm, theo thứ tự**. Mỗi bước có tiêu chí nghiệm thu.
> Chưa đạt tiêu chí thì không được sang bước sau.

| Giai đoạn | Nội dung | Trạng thái |
|---|---|---|
| A | Calib xong IMU **'2'** (con đang cầm) | A1 ✅ · A2 ✅ · A3 ✅ · A4 ⬜ |
| B | Calib xong IMU **'1'** (con còn lại) | ⬜ |
| C | Lắp lên robot + calib lắp đặt | ⬜ (bị chặn bởi cơ khí) |
| D | Fusion → **imufusion**, xem `fusion.md` | D1 ⬜ · D2 ⬜ · D3 ⬜ · D4 ⬜ |

> ## ⚠️ Dán nhãn vật lý lên hai con IMU, ngay bây giờ
>
> Hệ số calib là **dấu vân tay của từng con chip**, không phải của cái khe cắm.
> Bias 372 mg trên trục X chỉ đúng cho **đúng con** vừa calib. Hai con
> ICM-20948 nhìn y hệt nhau.
>
> Cắm nhầm con → mọi phép đo sai 372 mg trên X, **không có thông báo lỗi nào**.
> Góc vẫn ra số, robot vẫn chạy, chỉ là sai. Đây là loại lỗi tốn nhiều ngày nhất
> để tìm ra.
>
> Lấy bút xóa hoặc băng dính, viết `1` và `2` lên hai con. Tên trong `--unit`
> phải khớp đúng chữ trên nhãn đó.
>
> Quy ước tách làm hai lớp, cố ý:
> - **`--unit`** = con chip nào. Cố định vĩnh viễn theo nhãn dán.
> - **`--install`** = con chip đó đang cắm vào Pi nào. Đổi được, và **phải chạy
>   lại mỗi lần tráo chip giữa hai Pi**.

---

## Giai đoạn A — IMU '2'

### A1. Calib accel bằng ellipsoid fit ✅

```bash
python3 Calib/CALIB/calib_accel_ellipsoid.py
```

Kết quả `2026-08-31 19:06` — log `Calib/CALIB/logs/ellipsoid_20260831_185750/`,
hệ số `Calib/READ/vinh_accel_calib_ellipsoid.json`.

| | MỚI (ellipsoid, 26 tư thế) | CŨ (6 mặt) |
|---|---|---|
| ‖a‖ lệch max trên chính bộ dữ liệu | **0.063 %** | 1.549 % |
| ‖a‖ std | 0.0024 m/s² | 0.0644 m/s² |
| cross-axis | [0.012, 0.14, 0.063]° | [−0.045, 0.738, −0.546]° |
| chênh scale max | 1.276 % | 1.220 % |
| bias | [372.5, −132.9, −43.2] mg | [366.4, −129.0, −47.9] mg |
| độ phủ mặt cầu | 0.2454 (cần > 0.10) | — |

Kiểm tra độ vững (đã chạy, không cần làm lại):

- **Leave-one-out** — bỏ lần lượt từng tư thế rồi fit lại: bias xê dịch
  0.01–0.05 mg, scale xê dịch 0.006 %. Không tư thế nào chi phối kết quả.
- **Bỏ 5 tư thế rung nhất** rồi fit lại: bias đổi 0.03 mg. Không đáng kể.
- Chỉ phủ 16/26 hướng chuẩn, độ lệch so với hướng chuẩn 5–27°. **Không sao** —
  ellipsoid fit không cần tư thế đẹp, chỉ cần rải đều.

**Kết luận: ĐẠT.** Hai điều cần nhớ:

1. Dòng `phần xoay bị nuốt vào SM = 0.000 deg` của phương pháp mới là **hiển
   nhiên đúng theo cấu tạo**, không phải kết quả đo. Ta ép SM đối xứng, mà ma
   trận đối xứng thì phần xoay trong phân tích cực (polar decomposition) luôn
   bằng đơn vị. Ý nghĩa thật: phương pháp mới **về mặt cấu trúc không thể**
   nhét sai số gá đặt vào SM, còn phương pháp cũ đã nhét 0.791° vào.

2. **Bias 372 mg trên trục X là thật.** Hai phương pháp độc lập cho cùng con số.
   Kiểm tra thô cũng xác nhận: tâm dữ liệu trục X nằm ở raw +3190 LSB, trong khi
   Y ở −1095 và Z ở −301; bán kính X 8067 LSB so với danh định 8192.
   Datasheet ICM-20948 ghi offset điển hình ±25 mg — con này lệch gấp ~15 lần.
   Nguyên nhân nhiều khả năng là **ứng suất cơ khí lên đế chip** (hàn, vênh
   board, kẹp chặt) — đây là cơ chế đã biết gây lệch hàng trăm mg ở MEMS accel.
   Calib khử được nó, nên **không chặn tiến độ**, nhưng ghi lại vì:
   offset lớn do ứng suất thường **trôi theo nhiệt độ mạnh hơn** offset nhỏ.
   Nếu sau này thấy góc trôi khi robot nóng lên, đây là nghi phạm số một.

### A2. Calib lại gyro ✅

```bash
python3 Calib/CALIB/calib_gyro.py
```

Kết quả `2026-08-31 19:21` → `Calib/READ/vinhgyrocalib.json`.

| | gx | gy | gz |
|---|---|---|---|
| calib mới (19:21) | −3.15 | +23.12 | −32.94 LSB |
| soak cùng ngày (18:32–18:44) | −4.02 | +28.78 | −35.50 LSB |
| calib cũ (tháng 7) | −14.66 | +35.90 | −31.45 LSB |

Ba sửa đổi trong file trước khi chạy:

1. **Lỗi thật** — vòng lặp chỉ cộng dồn khi `dataReady()` trả True nhưng lại chia
   tổng cho `SAMPLES` cố định. Đọc hụt 10 mẫu là bias bị kéo về 0 đúng 1%, âm
   thầm. Giờ chia cho số mẫu thực.
2. **Đường dẫn ghi ra là tương đối** → file rơi vào thư mục đang đứng, không phải
   `Calib/READ/`. Giờ tính tuyệt đối từ vị trí script.
3. **Không hề kiểm tra đứng yên** — script vẫn tính `std` nhưng không in ra bao
   giờ. Giờ in std/p2p và phán ĐẠT / KHÔNG ĐẠT, ngưỡng lấy từ nền nhiễu đo thật
   ở soak DLPF=low: std `[18.1, 8.8, 3.6]` LSB.

**Phát hiện: bias gyro trôi ~5.7 LSB (0.086 °/s) trong 40 phút.** Sai số đo khi
trung bình 1000 mẫu chỉ `[0.57, 0.28, 0.11]` LSB, nên đây là trôi thật, không
phải nhiễu. Hệ quả xếp theo trục — xem §D.

### A3. Gộp hệ số vào một file ✅

```bash
python3 Calib/CALIB/make_calibfull.py --unit 2 --install
```

Chấm dứt việc chép tay `calibfull.json` (`analysis.md` §6). Script:

- gộp `vinh_accel_calib_ellipsoid.json` + `vinhgyrocalib.json`
- ghi `Calib/READ/calib_imu_<unit>.json` (nguồn chính thức, mỗi con chip một file)
- `--install` chép vào `src/leg_server/calibfull.json` của **chính Pi này**, sao
  lưu bản cũ ra `.bak`, và báo động nếu Pi này đang chạy hệ số của con chip khác
- ghi **nguồn gốc** vào `meta`: lấy từ file nào, calib lúc nào, phương pháp gì,
  bao nhiêu tư thế, DLPF nào — để sau này luôn truy được số ở đâu ra
- cảnh báo khi không nhất quán: sai FSR, sai `gyro_sensitivity`, đang dùng accel
  calib cũ, hoặc accel và gyro calib cách nhau > 24h

Đã chạy `--install` lúc 19:59. Bản chép tay cũ nằm ở `calibfull.json.bak`.

### A1->A3

### A4. Nghiệm thu IMU '2' ⬜

```bash
python3 Calib/READ/imu_soak.py --angles --dlpf low
```

`imu_soak.py` đọc hệ số từ `src/leg_server/calibfull.json` (`imu_soak.py:67`) —
đã được A3 ghi đè, nên nó đang chấm bộ hệ số mới. Đó cũng là lý do A3 phải chạy
trước A4, không đảo ngược được.

Chạy ~10 phút, để yên. Tiêu chí đạt **cả ba**:

| Chỉ số | Ngưỡng |
|---|---|
| sự kiện TEAR / GTEAR | 0 |
| ‖a‖ std khi đứng yên | < 0.1 % |
| lật qua 6 mặt, ‖a‖ chênh nhau | < 0.1 % |

Cái thứ ba là bài kiểm tra thật sự, và nó là **bài độc lập** với A1: dữ liệu
soak không phải dữ liệu đã dùng để fit. Con số 0.063 % ở A1 là sai số *khớp*
(fit residual), tự nhiên phải nhỏ; 0.1 % ở đây là sai số *dự đoán* trên dữ liệu
mới.

---

## Giai đoạn B — IMU '1'

Lặp lại A1 → A4 với con IMU còn lại, ghi ra file riêng (`--unit 1`).

**Ràng buộc bắt buộc** (`methods.md` §3.4): calib hai IMU **cùng một buổi, cùng
khối hộp, cùng nhiệt độ**. Lý do: khi tính góc *tương đối* giữa hai chân, sai số
*chung* của hai IMU triệt tiêu nhau, sai số *riêng* thì không. Calib cách nhau
một tuần ở hai nhiệt độ khác nhau là tự tay tạo ra sai số riêng.

---

## Giai đoạn C — Lắp lên robot

**Đang bị chặn bởi cơ khí.** Cần **2 lỗ định vị + 2 vít** để board tháo ra lắp
lại đúng vị trí cũ. Chưa có thì calib lắp đặt vô nghĩa — tháo ra lắp vào là mất
sạch.

Thuật ngữ: *calib lắp đặt* (mounting calibration) là bước tìm phép xoay giữa hệ
trục của con chip và hệ trục của robot. Ellipsoid fit **không** cho được cái
này — ‖a‖ không đổi khi xoay, nên nó mù về hướng tuyệt đối (`methods.md` §3.2).
Đây chính là chỗ bù lại lỗ hổng đó.

- **C2 — zero-at-home** (làm trước): đưa robot về tư thế chuẩn đã biết, ghi lại
  quaternion tại đó, coi nó là gốc. Đơn giản, đủ dùng.
- **C1 — trục khớp bằng PCA trên gyro** (nâng cấp sau): quay từng khớp một,
  vector vận tốc góc gyro đo được sẽ nằm dọc trục khớp; lấy thành phần chính
  (PCA) của đám vector đó ra trục khớp thật, không cần biết tư thế chuẩn.

---

## Giai đoạn D — Fusion

> **Kế hoạch chi tiết: `fusion.md`.**

Quyết định: **chuyển sang `imufusion`** (thuật toán chương 7 luận án Madgwick)
thay vì vá Madgwick chương 3 đang dùng. Việc này xoá luôn 2 trong 3 lỗi đã biết
của đường Madgwick (`analysis.md` §6):

1. **`beta` lệch nhau** — `gyroacce.py` dùng 0.033, `leg_server_left.py` dùng
   0.1, cùng dữ liệu ra hai góc khác nhau. → **biến mất**, imufusion không có
   `beta`.
2. **Quaternion khởi tạo `[1,0,0,0]`** — giả định robot nằm ngang hoàn hảo lúc
   bật nguồn. → **được lo** bởi *startup ramp*.
3. **`updateIMU()` không được truyền `dt` thật** — đã xác nhận:
   `leg_server_left.py:321` không truyền `dt`, nên `ahrs` 0.4.0 dùng `1/50` cố
   định, trong khi `dt` đo thật có `p99 = 41.5 ms` và 8.92 % số mẫu lệch quá
   20 %. → **vẫn phải tự lo**, bằng `set_sample_period()`.

Đợt này **chỉ làm offline**, chưa đụng `leg_server_left.py`. Không cần thu dữ
liệu mới — log `angles_20260831_183200` (712 s, DLPF=low) đủ dùng.

Xong D mới tới fusion hai IMU trên laptop.

### Trôi bias gyro — hệ quả không đối xứng giữa các trục

A2 đo được bias gyro trôi 5.7 LSB (0.086 °/s) trong 40 phút. Hệ quả **khác nhau
hoàn toàn** tùy trục, và đây là thứ quyết định kiến trúc của bước D:

| Trục | Có tham chiếu độc lập không? | Hệ quả của trôi bias |
|---|---|---|
| roll, pitch | **Có** — trọng lực | accel liên tục kéo về, bias bị bộ lọc tự nuốt |
| yaw | **Không** (chỉ dùng accel + gyro) | trôi tự do, không gì chặn |

Trọng lực cho biết đâu là "xuống", nên roll và pitch luôn có mốc. Nhưng trọng lực
**không nói gì về hướng bắc** — xoay quanh trục thẳng đứng thì vector trọng lực
đo được không đổi. Nên yaw không có mốc nào cả.

Với bias gz còn dư 0.039 °/s: **2.3° trôi yaw mỗi phút, ~140°/giờ.** Roll/pitch
thì không sao.

Ba lối ra, theo thứ tự nên cân nhắc:

1. **Ước lượng bias online** (tầng 4, `methods.md`) — coi bias gyro là biến trạng
   thái trong EKF, cập nhật liên tục. Đây là câu trả lời công nghiệp thật sự cho
   robot. Chữa được trôi ở cả ba trục.
2. **Dùng magnetometer cho yaw** — ICM-20948 có sẵn (9-DOF). Nhưng từ kế đặt gần
   động cơ và dây công suất thì đọc ra rác; phải khảo sát nhiễu trước khi tin.
3. **Chấp nhận, nếu bài toán không cần yaw tuyệt đối.** Nếu chỉ cần góc *tương
   đối* giữa hai chân trong mặt phẳng dọc, yaw có thể không nằm trong đường đo.
   Cần chốt điều này trước khi làm D.

**Nghiệm thu cuối cùng** không phải góc tuyệt đối, mà là **góc tương đối giữa
hai chân so với encoder**. Encoder là thước độc lập, IMU không biết gì về nó.

---

## Lệnh nhanh

```bash
# A1  calib accel (ENTER = dừng, s+ENTER = bỏ qua hướng đang gợi ý)
python3 Calib/CALIB/calib_accel_ellipsoid.py

# A2  calib gyro
python3 Calib/CALIB/calib_gyro.py

# A3  gộp hệ số + nạp cho leg_server
python3 Calib/CALIB/make_calibfull.py --unit 2 --install

# A4  nghiệm thu
python3 Calib/READ/imu_soak.py --angles --dlpf low

# kiểm tra nhanh cảm biến còn sống
python3 Calib/READ/check_sensor.py
```

## Việc còn treo, chưa xếp vào giai đoạn nào

- Lấy lại `Calib/READ/check_sensor.py` bản gốc từ máy anh Nam (bản hiện tại đã
  bị ghi đè).
- Bù nhiệt (tầng 3 trong `methods.md`) — chỉ làm nếu A4 hoặc D lộ ra hiện tượng
  trôi theo nhiệt. Nghi phạm đã có sẵn: bias X 372 mg ở A1.
