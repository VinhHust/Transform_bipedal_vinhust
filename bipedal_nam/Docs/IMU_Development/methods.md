# Hai phương pháp đo — chọn cái nào, khi nào

> Lý thuyết vì sao: `wonder.md`. Bản đồ code: `analysis.md`.
>
> Kết luận trước, giải thích sau: **hai phương pháp này không cạnh tranh nhau.**
> Ellipsoid fit để **CALIB**. Gá góc để **NGHIỆM THU**. Bỏ cái nào cũng hỏng.

---

## Bối cảnh: chuẩn công nghiệp

Ngành chia calib IMU làm bốn tầng:

| Tầng | Làm gì | Thiết bị | Bạn ở đâu |
|---|---|---|---|
| 1. Nhà máy chip | Trim offset/scale, đốt vào OTP | Máy ATE | TDK đã làm sẵn |
| 2a. Module, có thiết bị | Rate table biết chính xác hướng | Bàn quay $30k–$500k | **Không có** |
| 2b. Module, không thiết bị | Multi-position / ellipsoid fit | Không cần gì | **Ở đây** |
| 3. Bù nhiệt | Quét −40…+85°C, khớp đa thức | Buồng nhiệt | Chưa làm |
| 4. Online | Bias là biến trạng thái trong EKF | Không cần gì | Chưa làm |

Tầng 2b **không phải giải pháp chữa cháy** — đó là chuẩn sản xuất của iOS,
Android, PX4, ArduPilot và mọi hệ VIO, vì không hãng nào đủ tiền mua rate
table cho từng thiết bị.

Bạn đã thấy nó rồi: khi hiệu chuẩn la bàn drone, app bắt **múa hình số 8**.
Đó chính xác là ellipsoid fitting, chỉ dùng từ trường thay cho trọng lực.

**Sai lầm hiện tại của dự án:** đang dùng thuật toán của tầng **2a** (đòi rate
table) trong khi chỉ có điều kiện của tầng **2b**.

### Vì sao PX4 chỉ cần 6 vị trí mà ta cần 26?

Vì họ giải bài toán nhỏ hơn:

```
PX4/ArduPilot :  3 offset + 3 scale            = 6 an  -> 6 vi tri du
File nay      :  3 offset + 3 scale + 3 cross  = 9 an  -> can >=9, nen lay ~26
```

PX4 **cố ý bỏ cross-axis**, chấp nhận sai 1–2° vì EKF của drone nuốt được.
Ta muốn lấy lại 0.74° đó nên phải giải 9 ẩn.

Cả hai đều đúng chuẩn — chúng giải hai bài khác nhau.

---

# PHẦN 1 — Phương pháp mới: ellipsoid fit

## 1.1 Phần cứng của bạn và ràng buộc thật

Hiện trạng: IMU nằm trong một khối hộp, **dây jumper thoát ra mặt trên**.

Ràng buộc: không úp mặt có dây xuống bàn được, sẽ gãy dây.

**Ràng buộc này gần như vô hại.** Đo bằng mô phỏng (dữ liệu biết trước đáp án):

| Trường hợp | Số tư thế | Độ phủ | Sai `‖a‖` max | Sai cross-axis |
|---|---|---|---|---|
| Khối lý tưởng, đủ 26 | 26 | 0.327 | 0.0005 m/s² | 0.001° |
| Bỏ mặt +Z úp xuống | 25 | 0.310 | 0.0003 m/s² | 0.001° |
| **Bỏ mặt +Z + 4 cạnh + 4 góc kề** | **17** | **0.257** | **0.0004 m/s²** | **0.001°** |
| Chỉ nửa dưới | 9 | — | **THẤT BẠI** | — |

Mất cả một chỏm cầu vẫn không sao. Chỉ hỏng khi bạn co về đúng một nửa.

## 1.2 Ba mẹo gỡ ràng buộc dây

### Mẹo 1 — Thả dây ra mép bàn

Đặt hộp sát mép bàn, dây thò ra ngoài. Lấy lại được gần hết các tư thế cạnh
và góc kề mặt trên. Rẻ nhất, làm ngay được.

### Mẹo 2 — Kê cao

Đặt hộp lên hai quyển sách cách nhau, dây rơi vào khe giữa.

### Mẹo 3 — Dán lại IMU sang mặt khác (quan trọng nhất)

> **Giữa chừng buổi calib, bạn được phép tháo IMU ra dán sang mặt khác của hộp.**

Nghe như phạm luật, nhưng hoàn toàn hợp lệ. Lý do:

Ellipsoid fit chỉ dùng **một phương trình cho mỗi mẫu**: `‖SM·raw − bias‖ = g`.
Phương trình đó **không hề nhắc tới các mẫu khác**. Không có giả định nào về
quan hệ giữa tư thế 5 và tư thế 6. Mỗi mẫu là một sự thật độc lập.

Nên IMU nằm ở đâu trong hộp, hộp xoay thế nào, đều **không liên quan**. Cái duy
nhất quan trọng là **vector trọng lực trong hệ tọa độ CẢM BIẾN** phải rải đều.

So sánh: với `calib_accel.py` cũ thì tuyệt đối không được — vì nó giả định 6
mặt có quan hệ hình học cố định với nhau.

**Điều kiện duy nhất:** IMU không được xê dịch **trong lúc** đang lấy 150 mẫu
của một tư thế, và nhiệt độ phải ổn định suốt buổi.

## 1.3 Quy trình

```bash
cd Calib/CALIB
python3 calib_accel_ellipsoid.py
```

Chu trình lặp ~26 lần:

```
xoay hop -> DAT XUONG mat phang -> giu yen ~1s -> script tu chup -> xoay tiep
```

**Bắt buộc đặt xuống, không cầm trên tay.** Run tay khoảng 10–30 mg, vượt
ngưỡng nên script không bao giờ chụp.

Danh mục 26 tư thế của một khối hộp:

```
 6 MAT     nam sap tren tung mat
12 CANH    dat nghieng, ta len mot canh   <- BAT BUOC, xem 1.4
 8 GOC     dung tren mot goc (ke cho khoi do)
```

Không cần vuông. Không cần biết nghiêng bao nhiêu. Không cần bàn phẳng.
Kê bằng sách, cốc, cục tẩy đều được.

Đọc màn hình:

```
[ 7/30] std_a= 62.4(<40)  std_g= 15.2(<40)  cach_tu_the_gan_nhat=134.2deg | DANG DI CHUYEN
[ 7/30] std_a=  8.1(<40)  std_g=  3.4(<40)  cach_tu_the_gan_nhat=  6.8deg | TRUNG tu the cu
```

| Triệu chứng | Xử lý |
|---|---|
| `std_a` / `std_g` cao | Đang rung — đặt xuống, buông tay |
| `cach_tu_the_gan_nhat` < 15° | Tư thế trùng — xoay sang hướng khác |
| Bàn rung (quạt, xe chạy) | `--still-acc 80 --still-gyr 80` |

Bấm ENTER để dừng sớm. Dưới 9 tư thế thì script từ chối tính.

## 1.4 Vì sao 12 cạnh là bắt buộc

Không phải để "cho nhiều dữ liệu". Chúng là **loại dữ liệu duy nhất** chứa
thông tin cross-axis.

Cross-axis nghĩa là *"kéo trục X thì trục Y nhúc nhích bao nhiêu"*. Ở 6 mặt,
chỉ một trục cảm nhận trọng lực:

```
mat Z+ :  ax=0     ay=0     az=9.81     <- X,Y deu bang 0, khong bao gio "noi chuyen"
canh XY:  ax=6.94  ay=6.94  az=0        <- gio X va Y cung bi keo -> cross-axis lo ra
```

Đo thực nghiệm:

```
bo tu the                       n   do phu   sai ||a|| max
chi 6 MAT                       6     ---    THAT BAI (6 phuong trinh / 9 an)
6 mat + 4 canh (cung mot ho)   10     ---    THAT BAI (suy bien)
6 mat + 12 CANH                18   0.3264   0.0006 m/s2
6 mat + 12 canh + 8 GOC        26   0.3268   0.0006 m/s2
```

Chỉ 6 mặt là **vô nghiệm**, không phải "kém chính xác".

## 1.5 Ưu điểm

| | Chi tiết |
|---|---|
| **Miễn nhiễm gá đặt** | Không khai hướng → không có cửa cho sai số gá chui vào. Xoay cả bộ dữ liệu 5°: kết quả không đổi một chữ số |
| **Không cần thiết bị** | Không ke, không bàn máp, không rate table, không nivo |
| **Đo được cross-axis** | Lấy lại 0.74° mà PX4/ArduPilot bỏ |
| **Sai số tự kiểm chứng** | `‖a‖` phải đều ở mọi tư thế → có chỉ số nghiệm thu khách quan |
| **Nhanh** | ~26 tư thế × 4 s ≈ 3 phút. Gá 6 mặt cẩn thận mất 20 phút |
| **Chống người dùng vụng** | Đặt lệch 10° cũng không sao. Với `calib_accel.py`, lệch 1° là hỏng |
| **Chuẩn công nghiệp** | Đúng thứ iOS/Android/PX4 dùng |

## 1.6 Nhược điểm

| | Chi tiết | Nghiêm trọng? |
|---|---|---|
| **Không biết HƯỚNG** | Rotation ambiguity — nghiệm sai khác một phép xoay. Ta chốt bằng cách chọn `SM` đối xứng, nhưng hệ trục vẫn có thể lệch vài phần mười độ so với mũi tên in trên board | **Không**, vì phải đo phép xoay lắp đặt (bước C) dù sao đi nữa |
| **Cần nhiều tư thế** | ≥9, thực tế ~26. Cần cạnh và góc, không chỉ mặt | Nhỏ — dán lên khối hộp là xong |
| **Cần phủ đều mặt cầu** | Dữ liệu đồng phẳng → vô nghiệm | Nhỏ — chỉ số `do phu` bắt được, ngưỡng 0.10 |
| **Không đo được scale gyro** | Chỉ calib accel | Cần mẹo Tedaldi hoặc rate table |
| **Cần nhiệt ổn định** | Bias trôi theo nhiệt trong buổi calib sẽ méo kết quả | Nhỏ — để IMU chạy ấm 5 phút trước khi bắt đầu |
| **`‖a‖` không bắt được sai số xoay** | Phép xoay không đổi độ dài. `‖a‖` đều 100% vẫn có thể lệch 1° | **Có** — chính là lý do cần Phần 2 |

---

# PHẦN 2 — Phương pháp cũ: gá góc chuẩn (30°, 45°)

## 2.1 Cách làm

Chế gá cơ khí có góc biết trước. Đặt IMU lên, khai góc đó cho phần mềm, so
với góc đo được.

Hai kiểu dùng:

**2.1a — Dùng để CALIB** (`calib_accel.py` hiện tại)

Đặt 6 mặt, khai `a = [0,0,g]` cho mỗi mặt, giải least-squares 12 ẩn.

**2.1b — Dùng để NGHIỆM THU** (`imu_soak.py --angles`)

Đặt ở góc biết trước, đọc góc, so sánh:

```bash
python3 imu_soak.py --angles --dlpf low
# nhap "30,0" -> ENTER ghi -> ENTER ket thuc -> nhap goc tiep
```

## 2.2 Ưu điểm

| | Chi tiết |
|---|---|
| **Có mốc TUYỆT ĐỐI** | Cách duy nhất bắt được sai số **hướng** mà ellipsoid fit mù tịt |
| **Đo đúng thứ cần** | Sản phẩm cuối cần góc. Đây đo thẳng góc, không suy diễn |
| **Dễ hiểu** | Đặt 30° đọc ra 28.9° — ai cũng hiểu ngay |
| **Bắt được lỗi hệ thống** | Sai dấu, hoán trục, sai FSR đều lộ ra |
| **Là chuẩn công nghiệp** | Khi có rate table thì đây chính là tầng 2a |

## 2.3 Nhược điểm

| | Chi tiết | Số đo thật |
|---|---|---|
| **Chính xác không quá cái gá** | Sai số gá chui thẳng vào kết quả, không phát hiện được | **0.79°** nằm trong `SM` |
| **Không lặp lại được** | Tháo ra lắp lại board là ra số khác | **3.4°** chênh giữa 3 lần đo cùng tư thế phẳng |
| **Cần mặt phẳng chuẩn** | Bàn gỗ không đủ. Cần bàn máp + ke vuông | Chưa có |
| **Đo cross-axis rất kém** | 6 mặt là 6 điểm tệ nhất để thấy cross-axis (xem 1.4) | Vô nghiệm nếu dùng cho ellipsoid |
| **Chậm và mệt** | Mỗi mặt phải gá cẩn thận | ~20 phút, và vẫn sai |
| **Gimbal lock ở pitch ±90°** | Tại đó `ay`, `az` → 0, `roll = atan2(ay,az)` là nhiễu thuần | `90,0` đo được; `0,90` chỉ đọc cột pitch |
| **Bẫy dấu quy ước** | roll dương = **+Y chúc xuống**; pitch dương = **+X hếch lên** (bất đối xứng) | Đo `0,30` ra `−25.96°` |

## 2.4 Chẩn đoán: bộ đo kém chính xác hơn cảm biến

Đây là kết luận đau nhưng quan trọng.

```
Nhieu cua cam bien (dlpf low)      = 0.034 deg
Sai so khi dat lai board tren ga   = 3.400 deg     <- gap 100 lan
```

Khi cái thước tệ hơn vật cần đo **100 lần**, mọi con số nghiệm thu đều vô nghĩa.
Phải sửa gá trước, rồi mới nói chuyện được về độ chính xác.

Sửa gá là việc **cơ khí**:
- 2 lỗ định vị + 2 vít (1 vít thì board xoay quanh vít được)
- hoặc 2 chốt dowel pin + kẹp
- **không** băng dính, không keo hai mặt, không dây rút

---

# PHẦN 3 — So sánh

## 3.1 Bảng đối chiếu

| Tiêu chí | Ellipsoid fit | Gá góc chuẩn |
|---|---|---|
| Khai gì cho thuật toán | `‖a‖ = 9.80665` | `a = [0,0,g]` |
| Lời khai đó có đúng không | **Luôn đúng** | Chỉ đúng nếu gá hoàn hảo |
| Cần bàn phẳng | Không | **Có, rất nhạy** |
| Cần thiết bị | Không | Bàn máp + ke, hoặc rate table |
| Số tư thế | ≥9, nên ~26 | 6 |
| Tư thế phải chính xác | Không, bừa cũng được | **Có, tới từng độ** |
| Đo được scale | Có | Có |
| Đo được cross-axis | **Có** | Rất kém |
| Đo được bias | Có | Có |
| **Đo được HƯỚNG tuyệt đối** | **Không** | **Có** |
| Sai số gá chui vào kết quả | Không | **Có, 0.79°** |
| Thời gian | ~3 phút | ~20 phút |
| Tự kiểm chứng được | Có (`‖a‖` đều, `do phu`) | Không |
| Chuẩn công nghiệp tương ứng | Tầng 2b | Tầng 2a |

## 3.2 Điểm mấu chốt: chúng mù ở hai chỗ khác nhau

```
                    do duoc SCALE   do duoc CROSS   do duoc HUONG
                     + BIAS          -AXIS           TUYET DOI
ellipsoid fit          CO              CO              KHONG
ga goc chuan           CO              kem             CO
```

`‖a‖` không đổi khi xoay → **ellipsoid fit mù về hướng**.
6 mặt không cho hai trục cùng bị kéo → **gá góc mù về cross-axis**.

**Hai lỗ mù không trùng nhau.** Đó là lý do phải dùng cả hai.

## 3.3 Cách ghép đúng

```
1. ELLIPSOID FIT   -> SM, bias
   (calib_accel_ellipsoid.py, ~26 tu the bua bai)
   Ket qua: cam bien dung HINH DANG. ||a|| deu <0.1% moi tu the.
   Con lai: mot phep xoay chua biet.
        |
        v
2. NGHIEM THU BANG ||a||  -> bat loi hinh dang
   (imu_soak.py --angles --dlpf low, do vai tu the bat ky)
   Chi can nhin cot ||a||, KHONG can biet goc that.
   Ban nghieng bao nhieu cung khong anh huong phep kiem tra nay.
        |
        v
3. CALIB LAP DAT   -> phep xoay IMU <-> chan robot
   Day moi la cho giai bai toan HUONG. Va la buoc bat buoc dù calib kieu gi.

   C1 (tot nhat): dung chinh KHOP ROBOT lam thuoc.
        Cho khop quay qua lai +-30 deg. Gyro do vector van toc goc,
        vector do LUON nam doc truc khop. PCA -> huong truc khop trong he IMU.
        So voi ban ve -> ra phep xoay lap dat.
        Khong can ban, khong can ke. Robot tu lam thuoc cho chinh no.

   C2 (nhanh): zero o tu the dung.
        Dung robot o tu the home, doc 2 IMU, lay so do lam so 0.
        Chi triet tieu OFFSET, khong triet tieu phep XOAY.
        Du dung trong +-30 deg quanh tu the dung.
        |
        v
4. NGHIEM THU BANG GOC TUONG DOI  -> bat loi huong
   Dung robot o tu the biet truoc theo ENCODER.
   So goc tuong doi giua 2 chan voi goc encoder.
   Day moi la con so that su quan trong (xem 3.4).
```

## 3.4 Với 2 IMU: sai số tương đối mới là thứ đáng lo

Fusion tính **góc giữa hai chân**. Nên:

```
IMU trai +2.0 deg,  IMU phai +2.0 deg  ->  goc tuong doi DUNG   (triet tieu)
IMU trai +2.0 deg,  IMU phai -2.0 deg  ->  goc tuong doi sai 4 deg
```

Vì thế **đừng nghiệm thu từng IMU riêng lẻ**. Con số cần là góc tương đối so
với encoder.

Hệ quả thực tế: calib hai IMU **cùng một buổi, cùng một khối hộp, cùng nhiệt
độ**. Sai số chung sẽ triệt tiêu; sai số riêng thì không.

## 3.5 Chọn cái nào

| Tình huống | Dùng |
|---|---|
| Calib `SM`, `bias` | **Ellipsoid fit**, luôn luôn |
| Kiểm tra calib có tốt không | `‖a‖` đều ở mọi tư thế — không cần gá |
| Tìm phép xoay lắp đặt | Khớp robot (C1) hoặc zero tư thế đứng (C2) |
| Nghiệm thu cuối cùng | Góc tương đối vs encoder |
| Có rate table | Dùng nó cho mọi thứ, bỏ hết những cái trên |
| Debug sai dấu, hoán trục | `imu_soak.py --angles`, gá thô cũng đủ |

## 3.6 Ngân sách sai số

| Nguồn | Hiện tại | Sau khi sửa | Sửa bằng gì |
|---|---|---|---|
| Bàn nghiêng chui vào `SM` | 0.79° | ~0° | Ellipsoid fit |
| Cross-axis thật của chip | 0.74° | 0.74° (`SM` bù) | Ellipsoid fit |
| Nhiễu | 0.14° | 0.034° | `--dlpf low` (xong) |
| Torn read | 13.3° / 3 phút | 0 | `--dlpf low` (xong) |
| **Đặt lại board** | **3.4°** | **?** | **Cơ khí — 2 vít + lỗ định vị** |
| Phép xoay lắp đặt | chưa đo | ? | Bước C |

Sau khi ellipsoid fit xong, **kẻ giết bạn là cách gá board**, không phải cảm
biến. Cái đó sửa bằng cơ khí, không sửa bằng toán.

## 3.7 Câu hỏi gốc: có bao giờ chuẩn 100% không?

**Không.** Luôn còn dư. Nhưng câu hỏi đúng là *"sai số còn lại có nhỏ hơn mức
bộ điều khiển cần không"*.

Và về nỗi lo **mặt đất không phẳng** — tin vui: bạn không cần mặt đất phẳng.
Trọng lực chính là mốc thẳng đứng, chuẩn hơn mọi cái bàn. Khi kiểm tra góc,
đừng đo góc tuyệt đối — đo **góc giữa hai vector trọng lực** ở hai tư thế.
Bàn nghiêng bao nhiêu cũng bị triệt tiêu trong phép trừ.

Đó là cách tính ra được "s03 đặt 30° đo 28.87°" từ dữ liệu cũ, dù bàn cong.
