# Hướng Dẫn Căn Chỉnh Vị Trí Đứng Thẳng (Home Pose) Bằng Software

Tài liệu này giải thích sự khác biệt giữa Hardware Offset (bằng App FD) và Software Offset (bằng code Python), cũng như hướng dẫn cách ghi đè vị trí "Home" chuẩn vào code khi robot chịu tải trọng trên mặt đất.

## 1. Vấn đề với Hardware Calibration (App FD)
Hiện tại, việc set "điểm 0" vật lý cho servo được thực hiện qua App FD của Feetech. Khi ở trên không, robot đứng thẳng, bạn ấn "offset" thì servo sẽ ghi nhớ vị trí vật lý đó và gán cho nó giá trị encoder là `2048` (điểm giữa của dải 0-4095).

**Nhược điểm:**
- Khi đặt xuống đất, do trọng lượng robot (tải trọng) và khe hở cơ khí, robot bị xệ, nghiêng và không còn đứng thẳng nữa.
- Robot đang cắm cáp điều khiển vào Raspberry Pi, việc rút ra cắm lại vào máy tính Windows để set lại offset cứng trên App FD rất bất tiện.

**Bản chất của App FD:**
App FD của Feetech không hề biết "vị trí home của robot" là gì. Tính năng "offset" đơn thuần dịch điểm 0 của encoder bên trong servo về vị trí vật lý hiện tại. Từ đó trở đi, vị trí đó luôn trả về `2048`.

## 2. Giải pháp: Software Calibration

**Software Calibration** là phương pháp dùng bộ giá trị thực tế đo được để làm mốc Home mới trực tiếp trong phần mềm, thay vì ép phần cứng phải nhận là `2048`.

Bạn có thể dùng một Slider UI trên Pi để kéo thả các khớp bù trừ lại độ nghiêng cho đến khi robot đứng hoàn toàn thẳng trên mặt đất. Khi đó, giá trị của các khớp thu được sẽ là một bộ số khác `2048` (ví dụ: `2055`, `2030`, `2060`...).

Code chuyển đổi động học (kinematics) trong file `transformer.py` (hàm `degree_to_ticks` và `ticks_to_degree`) đã được thiết kế rất tốt: nó lấy biến `home_ticks` làm gốc `0` độ. 
Do đó, nếu bạn đổi `home_ticks` thành giá trị mới (ví dụ `2055`), thì hệ thống sẽ tự động hiểu `2055` là `0 độ`. Mọi tính toán dáng đi (gait) sẽ tự động lấy dáng đứng thẳng hoàn hảo dưới đất làm hệ quy chiếu mà không bị sai lệch.

## 3. Hướng dẫn sửa code

Sau khi dùng Slider UI tìm ra được bộ giá trị đứng thẳng chuẩn dưới đất, giả sử bạn có bộ giá trị mới như sau:
- **Trái:** `[2048, 2060, 2048, 2030, 2055, 2048]` *(Các chỉ số 1, 3, 4 trong mảng tương ứng khớp Hip, Knee, Foot)*
- **Phải:** `[2048, 2035, 2048, 2050, 2040, 2048]`

Bạn cần ghi đè vào **3 file** sau trong source code:

### 3.1. File `bipedal_nam/src/bipedal_robot/transformer.py`
Cập nhật mảng `home_pos` và giá trị `home_ticks` trong dictionary config.

```python
# Thay 2048 bằng các giá trị mới đo được
self.home_pos_left = [2048, 2060, 2048, 2030, 2055, 2048]
self.home_pos_right = [2048, 2035, 2048, 2050, 2040, 2048]

# Cập nhật tương ứng cho cấu hình từng khớp
self.servo_config_left = {
    5: {"name": "hip", "home_ticks": 2060, "min_ticks": 1676, "max_ticks": 2396}, # Cập nhật 2060
    7: {"name": "knee", "home_ticks": 2030, "min_ticks": 996, "max_ticks": 3051}, # Cập nhật 2030
    8: {"name": "foot", "home_ticks": 2055, "min_ticks": 1416, "max_ticks": 2711}, # Cập nhật 2055
}

self.servo_config_right = {
    5: {"name": "hip", "home_ticks": 2035, "min_ticks": 1700, "max_ticks": 2420}, # Cập nhật 2035
    7: {"name": "knee", "home_ticks": 2050, "min_ticks": 1045, "max_ticks": 3100}, # Cập nhật 2050
    8: {"name": "foot", "home_ticks": 2040, "min_ticks": 1385, "max_ticks": 2680}, # Cập nhật 2040
}
```

### 3.2. File `bipedal_nam/src/leg_server/leg_server_left.py`
Tìm phần xử lý lệnh `home` (khoảng dòng 499) và thay đổi mảng vị trí:

```python
elif cmd_type == "home":
    # Thay thế mảng [2048]*6 bằng mảng thật
    home_pos = [2048, 2060, 2048, 2030, 2055, 2048] 
    success = self.apply_new_positions(home_pos)
```

### 3.3. File `bipedal_nam/src/leg_server/leg_server_right.py`
Tìm phần xử lý lệnh `home` (khoảng dòng 534) và thay đổi mảng vị trí:

```python
elif cmd_type == "home":
    # Thay thế mảng [2048]*6 bằng mảng thật
    home_pos = [2048, 2035, 2048, 2050, 2040, 2048]
    success = self.apply_new_positions(home_pos)
```

## 4. Hướng phát triển nâng cao (Nên làm)
Việc chỉnh sửa cứng (hardcode) các con số vào mã nguồn như trên sẽ khá bất tiện nếu sau này các ốc vít bị lỏng và bạn cần calib lại.

**Giải pháp đề xuất:** 
Cải tiến script Slider UI để sau khi bạn ấn "Save", nó sẽ tự động ghi bộ số này ra một file JSON, ví dụ `home_offset.json`. 
Sau đó, trong hàm `__init__` của `transformer.py` và các file `leg_server`, bạn sử dụng `json.load('home_offset.json')` để nạp tự động các giá trị này lúc khởi động (tương tự như cách bạn đang load file calib IMU `vinh_accel_calib_ellipsoid.json`). Như vậy, mỗi lần căn chỉnh lại bằng UI, phần code cốt lõi sẽ không bao giờ cần phải chỉnh sửa thủ công nữa.
