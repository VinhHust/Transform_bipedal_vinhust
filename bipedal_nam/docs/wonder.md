# Vì sao calib accel hiện tại phụ thuộc vào cách gá đặt

> Ghi lại buổi phân tích ngày 2026-08-31. Đọc file này để hiểu **tại sao**;
> đọc `analysis.md` để biết **file nào làm gì**.

---

## 1. Câu hỏi khởi nguồn

> *"Tôi đặt IMU trên bàn, hoàn toàn phẳng, nhưng không thể biết khi nào nó
> thẳng với mặt đất — thậm chí mặt đất cũng không phẳng. Vậy có bao giờ calib
> được chuẩn 100%?"*

Và nhận xét độc lập của anh Nam:

> *"Nó sẽ lấy tham chiếu theo cách mình đặt, nên mình đặt nghiêng thì nó cũng
> lấy bias theo cái nghiêng đó. Thuật toán calib phụ thuộc nhiều vào độ chính
> xác khi mình gá đặt."*

Cả hai đều đúng. Dưới đây là cơ chế, con số đo được, và lối ra.

---

## 2. Gốc rễ: script khai cứng **hướng** của trọng lực

`Calib/CALIB/calib_accel.py:78`

```python
R[row_start + face["axis"], 0] = face["sign"] * G_REFERENCE
```

Dòng này nói với thuật toán least-squares:

> *"Khi tôi để trục Z hướng lên, vector gia tốc thật là `[0, 0, 9.80665]`."*

Thuật toán **tin tuyệt đối**. Nó không có cách nào kiểm tra. Nếu bàn nghiêng
0.8°, sự thật là `[0.13, 0.02, 9.806]` — và toàn bộ chênh lệch đó bị nhét
vĩnh viễn vào tham số calib.

**Đây là một khai báo không kiểm chứng được.** Mọi sai số gá đặt đi thẳng vào
kết quả mà không bị phát hiện.

---

## 3. Nghiêng chui vào `SM`, **không phải** `bias`

Điểm cần chính xác. Anh Nam nói "lấy bias theo cái nghiêng đó" — cơ chế đúng,
nhưng địa chỉ sai.

| | Bản chất toán học | Bị nghiêng bàn ảnh hưởng? |
|---|---|---|
| `bias` | phép **cộng** (offset) | **Không** |
| `SM` | phép **nhân** (ma trận) | **Có** |

Nghiêng bàn là một **phép xoay** — nó nhân vào vector, không cộng vào vector.
Phép nhân thì chui vào ma trận.

Còn `bias` miễn nhiễm, với điều kiện đo đủ 6 mặt đối xứng:

```
(+X) + (-X) + (+Y) + (-Y) + (+Z) + (-Z) = 0
```

Xoay một tổng bằng 0 thì vẫn bằng 0. Tâm của tập điểm không đổi → `bias`
không đổi. (Nếu chỉ đo 3 mặt, hoặc mỗi mặt lệch một kiểu, thì tính chất này
mất và `bias` cũng bị ảnh hưởng.)

---

## 4. Một nguyên nhân, **hai bệnh khác nhau**

Đây là chỗ dễ nhầm nhất.

```
KHAI CUNG HUONG g   (calib_accel.py:78)
│
├─ ga lech DEU nhau (cung mot huong cho ca 6 mat)
│     -> SM dinh mot phep XOAY thuan tuy
│     -> goc doc ra SAI
│     -> ||a|| VAN DUNG 9.80665            <-- VO HINH
│
└─ ga lech KHAC nhau moi mat
      -> SM bi meo HINH DANG (scale + cross-axis)
      -> ||a|| SAI, thay doi theo tu the   <-- THAY DUOC
```

### Vì sao bệnh thứ nhất vô hình?

> **Phép xoay không làm đổi độ dài vector.**

Xoay một quả bóng thì nó vẫn là quả bóng, bán kính y nguyên. Bóp nó mới thành
quả trứng.

Nếu `SM` chỉ dính một phép xoay `R`, thì:

```
a_do = R · a_that     ->     ||a_do|| = ||a_that||
```

Bài test `‖a‖ = 9.80665` **hoàn toàn không phát hiện được**. Bạn có thể lệch
1° mà mọi chỉ số vẫn xanh.

### Vì sao bệnh thứ hai thấy được?

Gá lệch khác nhau ở mỗi mặt thì không còn là một phép xoay chung. Nó bóp méo
hình dạng ellipsoid → bán kính thay đổi theo hướng → `‖a‖` dao động.

---

## 5. Số đo thực tế trên IMU #1

Tách `SM` hiện tại bằng **polar decomposition** (phân tích cực — tách một ma
trận thành "phần xoay thuần túy" × "phần bóp méo đối xứng"):

```
SM = R · S      R = phep xoay,  S = bien dang doi xung
```

```
phan XOAY  = 0.791 deg      <- ban nghieng / ga khong vuong. LOI.
   quanh X: +0.537 deg
   quanh Y: +0.544 deg
   quanh Z: -0.194 deg

phan MEO   = 0.738 deg      <- truc cam bien that su khong vuong goc.
                               BINH THUONG, moi chip deu co, SM sua duoc.
scale lech = 1.220 %        <- cung la dac tinh that.

||a|| lech 2.7 % giua cac tu the   <- bang chung ga dat KHONG NHAT QUAN
bias  = [3.574, -1.269, -0.452] m/s2 = [365, -130, -46] mg
        (datasheet ICM-20948: zero-g offset dien hinh +-25 mg
         -> cao gap hon 10 lan, dang nghi)
```

**0.79°** chính là con số mà anh Nam dự đoán bằng trực giác.

Và **2.7%** là bằng chứng sắc hơn: nếu bàn chỉ nghiêng đều một hướng thì
`‖a‖` phải đúng. Nó không đúng → mỗi mặt đã bị đặt lệch một kiểu khác nhau.

---

## 6. Lời giải: đổi ràng buộc

Có hai cách khai sự thật cho thuật toán:

| | Khai gì | Có đúng không? | Cần bàn phẳng? |
|---|---|---|---|
| Hiện tại | trọng lực **hướng** này | chỉ đúng nếu gá hoàn hảo | **Có**, rất nhạy |
| Ellipsoid fit | trọng lực **lớn** 9.80665 | **luôn đúng** | **Không** |

Độ lớn trọng lực không quan tâm bạn cầm board thế nào. Nghiêng 0.8°, nghiêng
37°, dốc ngược — `‖a‖` vẫn là 9.80665. Đó là sự thật bạn biết chắc 100%, miễn
phí, không cần dụng cụ.

### Ellipsoid fit là gì

**Trực giác trước:** nếu cảm biến hoàn hảo, xoay nó khắp mọi hướng thì tập hợp
điểm `(ax, ay, az)` phải nằm trên một **mặt cầu** bán kính 9.80665. Cảm biến
thật cho ra một **ellipsoid** (quả trứng) bị lệch tâm — lệch tâm do `bias`,
méo do scale và cross-axis.

Calib chính là tìm phép biến đổi **bóp quả trứng về lại mặt cầu**.

**Công thức:** tìm `SM`, `bias` sao cho

```
|| SM @ raw - bias || = 9.80665     voi MOI mau
```

Khai triển ra thành phương trình quadric tổng quát:

```
r' Q r - 2 u' r + c = 0        (r = raw, Q doi xung 3x3)
```

Giải bằng SVD → được `Q, u, c` → suy ra tâm `r0 = Q⁻¹u` và bán kính →
`SM` = căn bậc hai **đối xứng** của `Q` đã chuẩn hoá, `bias = SM·r0`.

**Hệ quả dễ chịu:**
- Bàn nghiêng: không sao
- Gá không vuông: không sao
- Không cần 6 mặt cẩn thận — **lăn board bừa** qua ~30 tư thế rải đều

---

## 7. Ellipsoid fit **không** cho bạn cái gì

Trung thực về giới hạn.

Vì `‖a‖` không đổi khi xoay cả hệ trục, nên bài toán có **vô số nghiệm**, sai
khác nhau một phép xoay. Nghề gọi là **rotation ambiguity** (mơ hồ về phép
xoay). Ta chọn `SM` **đối xứng** để chốt một nghiệm duy nhất — nhưng hệ trục
kết quả vẫn có thể lệch vài phần mười độ so với mũi tên in trên board.

| Bệnh | Ellipsoid fit |
|---|---|
| Méo hình dạng (`‖a‖` sai) | **Chữa dứt điểm** |
| Xoay (góc sai) | **Không tạo ra, nhưng cũng không biết** |

Nghe thì tệ, nhưng **không ảnh hưởng gì tới dự án**, vì IMU dán lên chân robot
thì chân robot cũng chẳng thẳng hàng với board. Bạn **buộc phải** đo phép xoay
IMU↔chân dù calib kiểu gì. Nên đừng cố giải bài đó bên trong calib.

---

## 8. Ba lớp calib — đừng trộn vào nhau

Script cũ bí vì cố làm cả ba trong một.

| | Calib gì | Cần mốc chuẩn gì | Làm ở đâu |
|---|---|---|---|
| **A** | Gyro bias | chỉ cần **đứng yên** | bàn |
| **B** | Accel nội tại (`SM`, `bias`) | chỉ cần **‖g‖ = 9.80665** | bàn, lăn bừa |
| **C** | Lắp đặt: IMU ↔ chân robot | **chính con robot** | trên robot |

Sai số 3.4° khi tháo ra lắp lại board thuộc về **C**. Không lượng toán nào ở
A hay B sửa được nó.

### A. Gyro bias

Đặt yên, đọc 60 s, lấy trung bình. Phải làm lại với `dlpf low`.

Gyro bias **trôi theo nhiệt độ**. Trên robot nhiệt sẽ khác (gần motor). Tốt
nhất: mỗi lần khởi động, robot đứng yên 3 giây rồi tự đo lại bias.

### B. Accel nội tại

Chạy `calib_accel_ellipsoid.py`. Lăn board qua ~30 tư thế: 6 mặt + 12 cạnh +
8 góc.

**Nghiệm thu:** `‖a‖` phải giống nhau ở mọi tư thế trong **0.1%**. Hiện tại
lệch 2.7%.

*Nhớ cái bẫy ở mục 4: test này không bắt được sai số xoay.*

### C. Lắp đặt

**C0 — gá phải LẶP LẠI ĐƯỢC.** Việc cơ khí, không phải phần mềm. Board phải
gắn sao cho tháo ra lắp lại vẫn về đúng chỗ cũ:
- 2 lỗ định vị + 2 vít (1 vít thì board xoay quanh vít được)
- hoặc 2 chốt dowel pin + kẹp
- **không** băng dính, không keo hai mặt, không dây rút

Chưa có C0 thì làm C là vô nghĩa — tháo board một lần là mất hết.

**C1 — dùng chính khớp robot làm thước (tốt nhất).** Trục quay của khớp là một
sự thật cơ khí, chính xác hơn bất kỳ cái bàn nào.

Cho khớp quay qua lại ±30°, chậm, 5–10 lần. Gyro đo được vector vận tốc góc,
và vector đó **luôn nằm dọc trục khớp**. Lấy PCA các mẫu gyro → ra hướng trục
khớp **trong hệ tọa độ IMU**. So với hướng đáng lẽ phải có theo bản vẽ →
chênh lệch chính là **phép xoay lắp đặt**.

Không cần bàn, không cần ke, không cần mặt đất phẳng. Robot tự làm thước cho
chính nó.

**C2 — zero ở tư thế đứng (nhanh, làm ngay được).** Dựng robot ở tư thế "home".
Đọc cả 2 IMU. Số đó **chính là số 0**. Trừ đi.

Hút hết sai số lắp đặt trong một phát. Nhược điểm: chỉ triệt tiêu **offset**,
không triệt tiêu **phép xoay** — càng xa tư thế home thì sai số càng lớn lại.
Với biped chạy trong ±30° quanh tư thế đứng thì đủ dùng để bắt đầu.

Làm C2 trước để chạy được, nâng lên C1 khi cần chính xác hơn.

**C3 — với 2 IMU, cái quan trọng là sai số TƯƠNG ĐỐI.** Fusion tính góc giữa
hai chân. Nếu cả 2 IMU cùng lệch +2° theo cùng hướng thì góc tương đối **vẫn
đúng** — sai số triệt tiêu. Nguy hiểm là khi IMU trái lệch +2° còn IMU phải
lệch −2°.

Nên khi nghiệm thu, đừng kiểm tra từng IMU riêng lẻ. Dựng robot ở một tư thế
bất kỳ nhưng **biết trước theo encoder**, so góc tương đối đo được với góc
encoder. Đó mới là con số cần.

---

## 9. Kết quả phụ: DLPF khử được torn read

Song song với chuyện calib, `imu_soak.py` phát hiện một lỗi phần cứng khác.

**Torn read (đọc rách thanh ghi)** là gì: `begin()` tắt DLPF phần cứng → ODR
(Output Data Rate — tốc độ chip cập nhật thanh ghi) vọt lên ~4.5 kHz. Trong
lúc Pi đọc khối 23 byte qua I2C (mất 2.3 ms), chip cập nhật thanh ghi ~10 lần.
Byte cao và byte thấp đến từ hai mẫu khác nhau → giá trị rác.

Dấu hiệu nhận biết: chênh lệch xấp xỉ **bội số của 256 LSB**.

**Cách chữa:** bật DLPF phần cứng để hạ ODR xuống ~1.1 kHz.

Kết quả đo (cửa sổ yên tĩnh 180–600 s, đã cắt đoạn chỉnh tư thế và đoạn bị đạp):

| | DLPF=off | DLPF=low |
|---|---|---|
| TEAR | 1 / 8663 mẫu | **0 / 20996 mẫu** |
| Nhiễu góc (std) | 0.14° | **0.034°** |
| Trôi 7 phút | — | 0.02° |
| `‖a‖` std | — | 0.0056 m/s² (0.06%) |

Torn read biến mất, bonus là nhiễu giảm 4 lần.

> **`dlpf low` phải bật ở MỌI script** — calib lẫn đọc lẫn `leg_server`.
> Calib ở `dlpf off` mà chạy ở `dlpf low` thì hệ số không khớp.

---

## 10. Có bao giờ chuẩn 100% không?

**Không.** Luôn còn dư. Nhưng câu hỏi đúng không phải *"có hoàn hảo không"* mà
là **"sai số còn lại có nhỏ hơn mức bộ điều khiển cần không"**.

| Nguồn sai số | Hiện tại | Sau khi sửa |
|---|---|---|
| Bàn nghiêng chui vào SM | 0.79° | ~0° (ellipsoid fit) |
| Cross-axis thật của chip | 0.74° | 0.74° (SM sửa được) |
| Nhiễu | 0.14° | 0.034° (dlpf low) |
| **Đặt lại board** | **3.4°** | **cần sửa cơ khí** |

Nhìn bảng là rõ: sau khi sửa calib, **kẻ giết bạn là cách gá board**, không
phải cảm biến. Cái đó sửa bằng cơ khí, không sửa bằng toán.

Còn chuyện **mặt đất không phẳng** — tin vui: bạn **không cần** mặt đất phẳng.
Trọng lực chính là mốc thẳng đứng, và nó chuẩn hơn mọi cái bàn. Khi kiểm tra
góc, đừng đo góc tuyệt đối — đo **góc giữa hai vector trọng lực** ở hai tư
thế. Bàn nghiêng bao nhiêu cũng bị triệt tiêu trong phép trừ.

---

## 11. Thứ tự thi công

```
1. [XONG]  Khoa dlpf=low o MOI script
2.         Calib gyro lai voi dlpf=low, 60s dung yen
3.         Calib accel ellipsoid, lan bua ~30 tu the
4.         Nghiem thu: ||a|| deu <0.1% o moi tu the
5.         Ga co khi lap lai duoc (2 vit + lo dinh vi)
6.         Lap len chan, calib C2 (zero o tu the dung)
7.         Lam lai buoc 2-6 cho IMU #2
8.         Fusion + nghiem thu goc tuong doi vs encoder
9.  [Sau]  Nang C2 -> C1 neu can chinh xac hon
```

---

## 12. Thuật ngữ

| Thuật ngữ | Nghĩa | Vai trò ở đây |
|---|---|---|
| **FSR** (Full Scale Range) | dải đo tối đa của cảm biến | `gpm4` = ±4g. Calib và đọc **phải trùng**, lệch là sai hệ số |
| **LSB** (Least Significant Bit) | đơn vị số nguyên thô chip trả về | ±4g → 8192 LSB/g |
| **ODR** (Output Data Rate) | tốc độ chip cập nhật thanh ghi | Cao quá → torn read |
| **DLPF** (Digital Low-Pass Filter) | bộ lọc thông thấp **trong chip** | Bật lên để hạ ODR, khử torn read |
| **EMA** (Exponential Moving Average) | lọc mượt trong phần mềm: `f = α·x + (1−α)·f` | α=0.2, chạy trên Pi sau khi calib |
| **SM** | ma trận 3×3 scale + cross-axis | `a = SM @ raw − bias` |
| **bias** | offset cộng, đơn vị m/s² | phần cảm biến đọc sai khi không có gia tốc |
| **cross-axis** | trục cảm biến không vuông góc nhau | ICM-20948 thật: ~0.74° |
| **ellipsoid fit** | khớp dữ liệu về mặt cầu bán kính g | phương pháp calib không cần biết hướng |
| **rotation ambiguity** | nghiệm sai khác nhau một phép xoay | giới hạn của ellipsoid fit → giải ở bước C |
| **polar decomposition** | tách ma trận = xoay × biến dạng | dùng để đo 0.79° nghiêng bàn |
| **torn read** | byte cao/thấp từ hai mẫu khác nhau | dấu hiệu: lệch bội số 256 LSB |
| **gimbal lock** | mất một bậc tự do ở pitch = ±90° | tại đó `roll = atan2(ay,az)` là nhiễu thuần |
