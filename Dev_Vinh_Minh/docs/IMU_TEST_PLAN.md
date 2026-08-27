# Kế hoạch Test IMU (đo độ chính xác từng IMU + IMU fusion)

> Mục tiêu: kiểm tra xem **từng IMU** (trái/phải) và **IMU sau khi fuse** đo có
> chính xác không, có ổn định không, và fusion có thật sự giúp giảm nhiễu không.

---

## 0. Hai khái niệm phải phân biệt trước

| Khái niệm | Nghĩa (dễ hiểu) | Cần gì để đo |
|---|---|---|
| **Precision (độ ổn định)** | Đo nhiều lần có ra giống nhau không? Ít nhảy số = tốt | KHÔNG cần biết đáp án đúng |
| **Accuracy (độ chính xác)** | Số đo có **trùng sự thật** không? | CẦN **ground truth** (góc đúng đã biết) |

**Ground truth** = giá trị đúng thật sự để so sánh. Trong bộ test này ground truth lấy từ:
- Mặt bàn đã cân bằng (dùng nivô / app đo góc) → roll = pitch = 0°.
- Đồ gá / nêm góc đã biết chính xác (VD 10°, 20°, 30°, 45°).
- Trọng lực: khi đứng yên, accelerometer luôn chỉ thẳng xuống → tự suy ra roll/pitch đúng.

---

## Test 1 — Đứng yên (Static test) ⭐ ưu tiên làm trước

**Ý tưởng:** robot/IMU đứng im tuyệt đối → hướng đúng là **hằng số** → mọi dao động
đo được chính là **nhiễu**. Đây là bài test giá trị nhất, chứng minh fusion có ích.

### Chuẩn bị
- Đặt **2 IMU đứng yên** trên **mặt bàn đã cân** (roll = pitch = 0°).
- Ghi log **roll / pitch** của: IMU **trái**, IMU **phải**, và **fused**.
- Thời lượng: ~60 giây (đủ mẫu để tính thống kê).

### Các biến thể tư thế (làm lần lượt)
1. Mặt bàn phẳng đã cân → ground truth roll = pitch = 0°.
2. Đặt lên **các góc đã biết** bằng đồ gá (10/20/30/45°).
3. **Test 6 mặt**: lần lượt đặt IMU theo 6 mặt (mỗi trục hứng ±1g).
4. **Nghiêng thuần 1 trục** (VD chỉ nghiêng pitch) → kiểm tra roll có **bị ăn theo**
   không (cross-axis / nhiễu chéo trục). Nếu roll đổi theo = trục lắp lệch.

### Chỉ số cần tính
- **Standard deviation (độ lệch chuẩn, std)** của roll và pitch cho **từng cái**
  (trái, phải, fused). Std nhỏ = ổn định.
- **Bias / offset**: giá trị trung bình đo được − góc thật. (VD luôn lệch +2°.)
- **2 IMU lệch nhau bao nhiêu độ** khi đứng yên: `|roll_trái − roll_phải|`,
  `|pitch_trái − pitch_phải|`. Lệch nhiều = lỗi lắp đặt / bias transform.

### Tiêu chí PASS — kiểm tra fusion có đúng không
Nếu 2 IMU nhiễu độc lập và fusion (trung bình) đúng, thì:

```
std(fused) ≈ std(mỗi IMU) / √2      (√2 ≈ 1.41)
```

- Nếu đúng cỡ này → fusion **thật sự giảm nhiễu** ~29%. ✅
- Nếu `std(fused)` ≈ `std(mỗi IMU)` (không giảm) → nhiễu 2 IMU **không độc lập**,
  hoặc fusion đang có vấn đề. ⚠️
- Nếu `std(fused)` **lớn hơn** từng cái → 2 IMU đang **bất đồng** (lắp lệch / transform
  sai), trung bình đang trộn 2 hướng khác nhau → sai. 🔴

---

## Test 2 — Đo góc nghiêng (accuracy với ground truth)

**Ý tưởng:** nghiêng robot/IMU tới **1 góc biết trước chính xác** bằng đồ gá, rồi so
góc IMU đo được với góc thật. Làm ở **nhiều góc** để thấy sai số theo dải.

### Cách làm
- Dùng đồ gá / nêm góc đã biết: ví dụ 0°, 10°, 20°, 30°, 45°.
- Ở mỗi góc, để yên vài giây, ghi roll/pitch của trái / phải / fused.
- Tính **sai số = góc_đo − góc_thật** cho từng góc.

### Cái cần phát hiện
- **Bias offset**: sai số cố định ở mọi góc (VD luôn +2°).
- **Sai số scale**: đo 30° nhưng ra 28° → hệ số nhân sai.
- **Phi tuyến (non-linearity)**: sai số tăng dần / méo theo góc, không đều.

Vẽ đồ thị **góc_đo theo góc_thật**: lý tưởng là đường thẳng y = x. Độ lệch khỏi
đường này cho biết bias (dịch lên/xuống), scale (dốc khác 1), phi tuyến (cong).

---

## Test 3 — Đo độ trôi (drift test)

**Ý tưởng:** để IMU **đứng yên lâu** (vài phút), xem góc có **tự trôi** theo thời gian
không, dù chẳng ai động vào.

### Cách làm
- Đặt yên, ghi log **pitch / roll** liên tục vài phút.
- Vẽ pitch/roll theo thời gian.

### Kỳ vọng
- **Roll / pitch**: nhờ có trọng lực làm mốc → phải **ổn định lâu dài**, không trôi.
  Nếu roll/pitch trôi dần → bộ lọc / calib có vấn đề.
- **Yaw**: sẽ trôi (không có magnetometer làm mốc) — đây là điều **đã biết trước**,
  không phải lỗi. Cần chú ý nếu yaw-drift kéo lệch cả pitch/roll khi robot xoay nhiều.

---

## Ghi chú quan trọng về fusion

- Fusion hiện tại = **trung bình cộng (mean)** 2 quaternion, KHÔNG phải trung vị.
  Với **2 IMU**, trung vị = trung bình cộng (không khác gì). Trung vị chỉ có ý nghĩa
  từ **≥3 IMU** (bỏ phiếu loại cảm biến hỏng).
- Trung bình 2 IMU **chỉ đúng khi cả 2 đo cùng một hướng** (cùng gắn trên thân cứng).
  Nếu 2 IMU gắn trên 2 chân xoay khác nhau → trung bình 2 hướng chân KHÔNG ra hướng thân.
- Trung bình **không chống được cảm biến hỏng**: nếu 1 IMU trả rác, kết quả bị kéo lệch.

## Thứ tự đề xuất

1. **Test 1** (static, mặt cân 0°) — làm ngay, rẻ và giá trị nhất.
2. **Test 2** (nhiều góc bằng đồ gá) — đo accuracy thật.
3. **Test 3** (drift) — chạy nền lâu, kiểm tra ổn định dài hạn.
4. Test 6 mặt + nghiêng thuần (cross-axis) — làm khi Test 1/2 lộ nghi vấn.
