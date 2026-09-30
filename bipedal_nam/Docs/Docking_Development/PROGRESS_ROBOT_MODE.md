# Tiến độ hai mode robot: BIPEDAL và CAR

**Cập nhật:** 30/09/2026.  
**Nhánh:** `feat/robot-mode`.  
**Kế hoạch gốc:** [PLAN_DIFF_DRIVE_CONTROLLER.md](PLAN_DIFF_DRIVE_CONTROLLER.md).  
**Trạng thái:** phần code server và API đã xong và có test. Chưa thử trên phần cứng. Giới hạn khớp theo mode mới làm cho chân phải. Phần test trên UI chưa viết.

## 1. Ý tưởng trong một đoạn

Mỗi module (chân trái, chân phải) có 2 mode:

| Mode | Bánh (ID 1, 3) | Khớp 4–9 | Lệnh được nhận |
|---|---|---|---|
| `BIPEDAL` | Đứng yên, khoá lệnh lái | Làm chân | `move`, `home` |
| `CAR` | Chạy xe vi sai theo `v`, `ω` | Làm tay gắp | `drive`, `arm_move` |

Mỗi lần gọi `set_mode`, Pi trả về một **vé** (`session`). Vé cũ hết hiệu lực ngay lúc đó. Lệnh `move`, `arm_move`, `drive` phải mang vé đúng thì mới được chạy.

Lệnh `drive` còn có **watchdog** (bộ đếm giờ an toàn): nếu 0.25 s không có lệnh mới thì bánh tự dừng và mất quyền lái. Muốn lái lại phải gọi `set_mode("CAR")` để lấy vé mới.

## 2. Đã commit trên nhánh

| Commit | Nội dung |
|---|---|
| `e915e4e` | `car_mode/diff_drive.py`: toán xe vi sai (v, ω ↔ tốc độ bánh ↔ raw), có kẹp, ramp và `load_config`. Thêm 2 file `car_mode/config/diff_drive_{left,right}.json`. |
| `6e29ef8` | `car_mode/wheel_bus.py`: đọc/ghi 2 bánh. `bipedal.py`, `bipedal_left.py`: dừng bánh trước khi bật torque; xoá khối omni cũ. |
| `8122fd2` | `car_mode/mode_controller.py`: bộ não mode (`set_mode`, `drive`, `stop_drive`, `tick`, `check_command`, `feedback`). Nối vào cả 2 `leg_server`: trạm gác lệnh ở đầu `process_command`, gọi `tick()` trong vòng chính, dừng bánh khi shutdown. |
| `343fbdb` | `transformer.py`: `set_mode`, tự gắn vé, `set_base_velocity` (PUSH), `stop_base`, `get_base_state`, `arm_move` ở CAR, phanh khi shutdown. |

**Test:** `cd bipedal_nam && PYTHONPATH=src python3 -m pytest tests/test_diff_drive.py tests/test_wheel_bus.py tests/test_mode_controller.py` → 45 passed (30/9).

### Thông số xe đã chốt

Nằm trong `car_mode/config/diff_drive_*.json`:

| Tham số | Giá trị |
|---|---|
| Bán kính bánh `r` | 0.05 m |
| Khoảng cách 2 bánh `b` | 0.24145 m |
| Tỉ số truyền | 1 (bánh gắn thẳng trục servo) |
| Bước / vòng | 4096 |
| `max_raw` | 1500 |
| `max_v` / `max_omega` | 0.1 m/s / 0.5 rad/s |
| Gia tốc bánh tối đa | 4 rad/s² |
| Watchdog `command_timeout_s` | 0.25 s |
| Chiều bánh `left/right_direction` | +1 / +1 (**tạm**, chờ thử thật) |

## 3. Chưa commit: giới hạn khớp theo mode (chân phải)

File: `src/leg_server/leg_server_right.py`.

### 3.1. Số đo (30/9)

Hai mode chỉ khác nhau ở khớp **bub (ID 4)**. Bub xoay lên là tay (CAR), bub xoay xuống là chân (BIPEDAL). Khớp 5–9 dùng chung một dải.

| Khớp | BIPEDAL (`servo_limits`) | CAR (`servo_limits_car`) |
|---|---|---|
| 4 bub | 2000 – 3200 | 3095 – 4050 |
| 5 | 1700 – 2400 | 1700 – 2400 |
| 6 | 1040 – 3060 | 1040 – 3060 |
| 7 | 900 – 3200 | 900 – 3200 |
| 8 | 1400 – 2600 | 1400 – 2600 |
| 9 | 1870 – 2800 | 1870 – 2800 |

- **Giới hạn cứng trên FD** = hợp của 2 bảng. Bub: 2000 – 4050.
- **Tư thế chuyển mode:** bub nằm trong vùng giao **3095 – 3200**, các khớp khác ở home.
- **Home:** mọi khớp = 2048, tức `home_pos = [2048] * 6`.

### 3.2. Thay đổi code

1. Thêm bảng `self.servo_limits_car` ngay dưới `self.servo_limits`. Bảng được viết đầy đủ từng dòng, không dùng `{**self.servo_limits, ...}`, để `sliderUI` đọc được bằng `ast.literal_eval`.
2. Thêm hàm `active_servo_limits()`. Robot đang ở mode nào thì hàm trả bảng của mode đó.
3. `apply_new_positions` kẹp vị trí theo bảng mà `active_servo_limits()` trả về.
4. `home_pos` đổi từ `[1989, 2070, 2052, 2036, 2133, 2048]` thành `[2048] * 6`.
5. Import thêm `RobotMode` từ `mode_controller`.

### 3.3. Vì sao chọn bảng theo mode, không theo loại lệnh

Kế hoạch ban đầu là thêm tham số `apply_new_positions(positions, limits=None)`, và chỉ nhánh `arm_move` truyền bảng CAR. Cách đó có lỗi với lệnh `stop`, vì `stop` chạy được ở cả 2 mode:

- Ở CAR, bub đang ở 3800. Người dùng bấm Stop.
- `stop` gửi lại vị trí hiện tại, nhưng kẹp theo bảng BIPEDAL (max 3200).
- Kết quả: bub bị kéo giật xuống 3200. **Bấm Stop mà robot lại cử động.**

Chọn bảng theo mode hiện tại thì mọi lệnh ghi vị trí đều dùng đúng bảng.

## 4. Việc còn lại

### 4.1. Phần cứng (chưa thử)

- [ ] Kiểm tra `FeetechMotorsBus` trên Pi có `sync_read` (bản lerobot trên Pi là bản copy riêng).
- [ ] Kê bánh lên, `v = 0.05, ω = 0`: mỗi bánh ≈ 652 raw, cùng chiều. Kiểm đơn vị.
- [ ] Kê bánh lên, `v = 0, ω = 0.5`: ≈ ±787 raw, bánh phải quay tiến. Kiểm `left/right_direction`.
- [ ] Đặt xuống sàn, `v = 0.05` trong 4 s: xe đi 20 cm. Kiểm công thức.
- [ ] Đặt xuống sàn, `ω = 0.5` trong 12.6 s: xe quay đúng 1 vòng. Kiểm `b`.

Kê bánh lên chỉ kiểm được đơn vị và dấu. Nó không kiểm được công thức, vì UI và server dùng chung `diff_drive.py` nên luôn cho cùng một đáp án. Muốn kiểm công thức thì phải đo bằng thước.

### 4.2. Code

- [ ] **Chân trái:** đo lại và làm giống mục 3. Bub trái xoay ngược chiều (bảng cũ 996 – 2146), nên số đo sẽ khác chân phải.
- [ ] **Chuyển mode an toàn (đang bàn):** home (bub 2048) nằm ngoài vùng giao 3095 – 3200. Câu hỏi còn mở:
  - Ai đưa bub vào vùng giao: kéo tay, hay có nút "Về tư thế chuyển"?
  - Chỉ UI kiểm tra, hay server cũng từ chối `set_mode` khi bub ở ngoài vùng giao?
- [ ] **Test CAR trong `UI/sliderUI.py`** (thay cho script `drive_car.py` riêng):
  1. Hàng chọn mode BIPEDAL / CAR. Mới mở UI thì phải chọn mode để lấy vé. Chưa chọn thì khoá gửi lệnh.
  2. Gắn `session` vào mọi lệnh. Ở BIPEDAL gửi `move`, ở CAR gửi `arm_move`. Khoá nút Home ở CAR.
  3. `parse_servo_limits` nhận tên bảng (`servo_limits` / `servo_limits_car`). Khi đổi mode thì đổi dải slider, rồi gọi lại `sync_from_robot`.
  4. Khung Xe: slider `v`, `ω` (giới hạn đọc từ config JSON), nút 0, nút DỪNG XE (`stop_drive`).
  5. Gửi `drive` ở **mọi tick** 20 Hz, kể cả khi số không đổi, để làm nhịp tim cho watchdog. `seq` tăng sau mỗi gói.
  6. Hỏi `base_feedback` khoảng 5 Hz. Hiện `drive_armed = False` bằng chữ đỏ ("watchdog đã ngắt"). `cmd_age_s` cho biết độ trễ mạng.
  7. Đóng cửa sổ khi đang ở CAR thì gửi `stop_drive` trước.
  8. Ô "chạy N giây": UI gửi lệnh trong N giây rồi tự đưa về 0.
- [ ] Không chạy UI cùng lúc với script `TransformerAPI`, vì mỗi bên lấy vé mới sẽ làm vé của bên kia mất hiệu lực.
- [ ] Nới `command_timeout_s` sau khi cải thiện mạng (dây LAN riêng). Người phụ trách tự chỉnh.
