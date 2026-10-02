# Tiến độ hai mode robot: BIPEDAL và CAR

**Cập nhật:** 30/09/2026.  
**Nhánh:** `feat/robot-mode`.  
**Kế hoạch gốc:** [PLAN_DIFF_DRIVE_CONTROLLER.md](PLAN_DIFF_DRIVE_CONTROLLER.md).  
**Trạng thái:** module **phải** (`mobile1`) đã chạy trên robot thật: lái bánh, điều khiển tay ở CAR, sequence về transition pose, đổi mode CAR → BIPEDAL. Module **trái** là xe mang tag, tạm thời không làm mode CAR. Còn phải đo xe trên sàn.

## 1. Ý tưởng trong một đoạn

Mỗi module có 2 mode:

| Mode | Bánh (ID 1, 3) | Khớp 4–9 | Lệnh được nhận |
|---|---|---|---|
| `BIPEDAL` | Đứng yên, khoá lệnh lái | Làm chân | `move`, `home` |
| `CAR` | Chạy xe vi sai theo `v`, `ω` | Làm tay gắp | `drive`, `arm_move` |

- **Vé (`session`).** Mỗi lần gọi `set_mode`, Pi trả về một vé mới, và vé cũ hết hiệu lực. `move`, `arm_move`, `drive` phải mang vé đúng.
- **Watchdog.** Đang lái mà quá `command_timeout_s` không có lệnh `drive` mới thì bánh tự dừng và mất quyền lái. Muốn lái lại, gọi `set_mode("CAR")` lần nữa. Gọi cùng mode thì không bị kiểm tra bub.
- **Vùng chuyển mode.** Chỉ được đổi mode khi bub nằm trong vùng giao của 2 bảng giới hạn (3095–3200). Server tự kiểm tra, UI hay script đều không lách được.

## 2. Đã commit trên nhánh

| Commit | Nội dung |
|---|---|
| `e915e4e` | `car_mode/diff_drive.py`: toán xe vi sai (v, ω ↔ tốc độ bánh ↔ raw), kẹp, ramp, `load_config`. Config `car_mode/config/diff_drive_{left,right}.json`. |
| `6e29ef8` | `car_mode/wheel_bus.py`: đọc/ghi 2 bánh. `bipedal.py`, `bipedal_left.py`: dừng bánh trước khi bật torque. |
| `8122fd2` | `car_mode/mode_controller.py`: bộ não mode. Nối vào cả 2 `leg_server`. |
| `343fbdb` | `transformer.py`: `set_mode`, gắn vé, `set_base_velocity`, `stop_base`, `get_base_state`, `arm_move`. |
| `832378a` | `leg_server_right.py`: 2 bảng giới hạn theo mode, chặn đổi mode ngoài vùng chuyển, chọn mode lúc khởi động theo bub. `car_mode/transition.py` + `tests/test_transition.py`. |
| `c3f925f` | `UI/sliderUI.py`: chọn mode + vé, đổi bảng giới hạn theo mode, khung Xe v/ω. |
| `28eb98e` | `UI/sliderUI.py`: sequence về transition pose. `car_mode/config/transition_right.json`. Watchdog bánh phải lên 1 s. |

**Test:**
```bash
cd bipedal_nam
PYTHONPATH=src python3 -m pytest tests/test_transition.py tests/test_mode_controller.py tests/test_diff_drive.py tests/test_wheel_bus.py -q
```
→ 58 passed. Gọi đích danh từng file vì `test_gait.py` và `test_kinematics.py` cần `lerobot`, laptop không có.

## 3. Thông số module phải

### 3.1. Xe (`car_mode/config/diff_drive_right.json`)

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| Bán kính bánh `r` | 0.05 m | |
| Khoảng cách 2 bánh `b` | 0.24145 m | |
| Tỉ số truyền | 1 | bánh gắn thẳng trục servo |
| Bước / vòng | 4096 | |
| `max_raw` | 1500 | |
| `max_v` / `max_omega` | 0.1 m/s / 0.5 rad/s | |
| Gia tốc bánh tối đa | 4 rad/s² | |
| `left_direction` / `right_direction` | **−1 / +1** | đo bằng app FD (mục 5.1) |
| Watchdog `command_timeout_s` | **1.0 s** | nới từ 0.25 s cho lúc test qua wifi |

### 3.2. Giới hạn khớp (`leg_server_right.py`)

Hai mode chỉ khác nhau ở **bub (ID 4)**. Bub xoay lên là tay (CAR), xoay xuống là chân (BIPEDAL).

| Khớp | BIPEDAL (`servo_limits`) | CAR (`servo_limits_car`) |
|---|---|---|
| 4 bub | 2000 – 3200 | 3095 – 4050 |
| 5 hip | 1700 – 2400 | 1700 – 2400 |
| 6 twist | 1040 – 3060 | 1040 – 3060 |
| 7 knee | 900 – 3200 | 900 – 3200 |
| 8 foot | 1400 – 2600 | 1400 – 2600 |
| 9 gripper | 1870 – 2800 | 1870 – 2800 |

- Giới hạn cứng ghi trên FD = hợp 2 bảng. Bub: 2000–4050.
- Vùng chuyển mode = giao 2 bảng ở bub = **3095–3200**. Vùng này được **tính từ 2 bảng** (`transition_range`), không ghi tay.
- Home: mọi khớp = 2048.
- Server kẹp vị trí theo **mode hiện tại**, không theo loại lệnh. Nếu kẹp theo loại lệnh thì lệnh `stop` ở CAR sẽ kẹp bub về 3200, tức là bấm dừng mà tay lại giật.

### 3.3. Sequence CAR → transition pose (`car_mode/config/transition_right.json`)

Đi **từng bước**: bước này xong (mọi khớp trong bước lệch ≤ `tolerance`) mới sang bước sau, để tay không va chạm.

| Bước | Khớp → đích |
|---|---|
| 1 | foot → 1400 |
| 2 | twist → 2048, hip → 2048, gripper → 2048 (cùng lúc) |
| 3 | bub → 3200 |
| 4 | knee → 2048 |
| 5 | foot → 2048 |
| 6 | bub → 3150 |

`speed` 500, `accel` 70, `tolerance` 30, `margin_s` 2.

- Mỗi bước chờ tối đa = quãng đường dài nhất ÷ `speed` + `margin_s`. Quá giờ thì UI dừng sequence và ghim tay tại chỗ.
- `tolerance` lúc đầu là 20. Lần chạy thật đầu tiên, twist dừng lệch 24 tick rồi đứng im (vùng chết + tải), nên nới lên 30.
- File này chỉ UI trên laptop đọc, không cần chép lên Pi.

## 4. Server và UI

### 4.1. Server (`leg_server_right.py` + `car_mode/transition.py`)

- `set_mode` đổi mode thật (khác mode hiện tại) mà bub đo được nằm ngoài 3095–3200 → từ chối, kèm lý do có số.
- `set_mode` cùng mode (lấy lại quyền lái, UI mới mở xin vé) → không kiểm tra bub.
- Lúc khởi động, chọn mode theo bub đo được: bub < 3095 → BIPEDAL, còn lại → CAR. Log in dòng `✓ Mode lúc khởi động (bub=…)`.

### 4.2. UI (`UI/sliderUI.py`)

| Phần | Làm gì |
|---|---|
| Hàng mode | Mở UI thì tự hỏi `base_feedback` xem server đang ở mode nào, rồi xin vé đúng mode đó. Nút "Đổi sang …" chỉ sáng khi bub thật nằm trong vùng chuyển và không có sequence đang chạy. |
| Slider khớp | Dải slider đổi theo bảng của mode. BIPEDAL gửi `move`, CAR gửi `arm_move`, luôn kèm vé. Nút Home khoá ở CAR. |
| Nút "Về transition" | Chạy sequence ở mục 3.3. Khớp ngoài bước giữ nguyên đích đã ra lệnh. Bấm DỪNG thì huỷ sequence. |
| Khung Xe | Slider + ô nhập cho `v`, `ω`. Dòng "bánh dự kiến" (UI tự tính) để so với dòng "đo" (server gửi về). |
| BẬT lái | Bật = xin vé CAR mới (seq đếm lại từ 1), rồi gửi `drive` **mỗi tick 20 Hz, kể cả khi số không đổi** (nhịp tim cho watchdog). Tắt = `stop_drive`. |
| Đồng hồ bánh | Hỏi `base_feedback` ~5 Hz trong luồng poller: đích / đang ra lệnh / đo thật, `cmd_age_s`, chữ đỏ khi watchdog đã ngắt. |
| Chạy N giây | Chạy v, ω đang đặt trong N giây rồi về 0 bằng ramp (không phanh gấp), nên quãng đường ≈ v·N. |
| Đóng cửa sổ | Đang ở CAR thì gửi `stop_drive` trước. |

**Lưu ý khi dùng:**
- Không chạy UI cùng lúc với script `TransformerAPI`. Bên nào xin vé cũng làm vé của bên kia mất hiệu lực.
- Pi nào đang tắt thì comment dòng của Pi đó trong `LEGS`. Nếu không, UI đứng hình 3 s mỗi khi thử kết nối lại.

## 5. Kết quả thử trên robot thật (module phải, 30/9)

### 5.1. Dấu bánh (app FD, nhìn từ sau xe, mặt hướng về mũi)

| Bánh | Ghi vel dương thì xe… | `direction` |
|---|---|---|
| Phải (ID 1) | tiến | +1 |
| Trái (ID 3) | lùi | −1 |

Cả 2 bánh đã đặt Mode = 1 (velocity).

### 5.2. Checklist

| Việc | Kết quả |
|---|---|
| `FeetechMotorsBus.sync_read` có trên Pi | ✅ `True` |
| Đọc tốc độ bánh qua `base_feedback` | ✅ có số đo |
| Server chọn mode lúc khởi động theo bub | ✅ |
| UI tự lấy vé, điều khiển tay bằng `arm_move` ở CAR | ✅ |
| Lái bánh, watchdog | ✅ sau khi tắt wifi power save (mục 6) |
| Sequence về transition pose | ✅ sau khi nới `tolerance` lên 30 |
| Đổi mode CAR → BIPEDAL | ✅ |
| Kê bánh: v = 0.05 → raw ≈ −652 / +652, v đo ≈ +0.050 | ⬜ chưa ghi số |
| Kê bánh: ω = 0.5 → raw ≈ +787 / +787, ω đo ≈ +0.5 | ⬜ chưa ghi số |
| Trên sàn: v = 0.05, chạy 4 s → đi 20 cm | ⬜ |
| Trên sàn: ω = 0.5, chạy 12.6 s → quay 1 vòng | ⬜ |

Kê bánh chỉ kiểm được đơn vị và dấu. Công thức phải kiểm bằng thước, vì UI và server dùng chung `diff_drive.py` nên luôn ra cùng một đáp án.

## 6. Bài học khi đưa lên Pi

| Lỗi gặp | Nguyên nhân | Cách sửa |
|---|---|---|
| `No module named 'bipedal_robot.car_mode'` | Thư mục `car_mode` chép vào `src/` thay vì `src/bipedal_robot/` | `mv src/car_mode src/bipedal_robot/` |
| Không tìm thấy file calib IMU | Pi đặt tên thư mục `Calib`, code tìm `Calib_IMU` | Đổi tên thư mục trên Pi thành `Calib_IMU` |
| bub đọc ra 22 | Tay quay lố qua 4095 nên encoder quay vòng về 0 | Đưa tay về trong 2000–4050 rồi khởi động lại server |
| Watchdog ngắt liên tục | Wifi của Pi bật tiết kiệm pin, lệnh tới trễ 150–380 ms | `sudo iw dev wlan0 set power_save off`, và cấu hình cho giữ sau khi khởi động lại |
| UI đứng hình từng lúc | Panel trái cứ thử kết nối Pi trái đang tắt, mỗi lần chặn 3 s | Comment dòng `LEFT` trong `LEGS` |

## 7. Việc còn lại

- [ ] Đo xe trên sàn (mục 5.2), ghi số kê bánh.
- [ ] Sequence chiều ngược lại: BIPEDAL → transition pose.
- [ ] Module trái: hiện là **xe mang AprilTag**, đứng yên khi docking, nên chưa cần mode CAR. Khi cần thì làm lại toàn bộ: bảng giới hạn (bub trái xoay ngược chiều), dấu bánh, wifi power save, `sync_read`, sequence riêng.
- [ ] Sửa lỗi có sẵn trong UI: `on_feedback` ghi đè dòng `status` khoảng 67 lần/giây, nên phần lớn thông báo ở dòng đó biến mất ngay.
- [ ] Nối dây LAN Pi ↔ laptop, rồi đưa `command_timeout_s` về lại khoảng 0.25 s.

## 8. Docking trên xe thật — kế hoạch đã bàn (1/10)

Robot mode xe phải đã lái được, nên docking nối thẳng vào đó. Phương pháp vẫn theo [DOCKING_ASTOLFI.md](DOCKING_ASTOLFI.md). Xe A (chủ động, camera) = module phải. Xe B (đứng yên, mang tag) = module trái.

### 8.1. Hệ trục camera và dấu

- Hệ OpenCV, **nhìn khi đứng sau camera**: `+x` sang phải, `+y` xuống đất, `+z` ra trước (hướng ống kính nhìn). Đây là hệ tay phải.
- Bẫy: đứng **trước** webcam thì `+z` đâm vào mặt mình, còn "phải trong ảnh" lại là **bên trái** của mình. Ghép "x phải màn hình + z ra ngoài màn hình" thành hệ tay trái, không tồn tại.
- Hai xe luôn cùng độ cao nên bỏ **giá trị** `y`, bài toán là 2D: `Fwd = z`, `Left = −x`. Nhưng vẫn cần **hướng** của `y`, vì yaw là góc quay quanh nó.
- `yaw = atan2(R[0,2], R[2,2])` là góc quay quanh `+y` (chỉ xuống), theo quy tắc tay phải. Nhìn từ trên xuống, yaw dương = **cùng chiều kim đồng hồ**.
- Xe quy ước ngược chiều kim đồng hồ là dương (trục Up = `−y`), nên dự đoán: `θ = −yaw`, `ψ = wrap(180° − yaw)`, `θ_d = −yaw`.

**Bài kiểm dấu (§9.3): ✅ đã làm 2/10, tag 40 mm.** Đứng sau camera, chạy `tag_control.py`:

| Làm | Dự đoán | Thật |
|---|---|---|
| Đẩy tag ra xa | `Z` tăng | ✅ |
| Dịch tag thẳng sang phải, không xoay | `X` ra **+** | ✅ |
| Hạ tag xuống | `Y` tăng | ✅ |
| Xoay tag cùng chiều kim đồng hồ (nhìn từ trên), tag vẫn đứng thẳng | `Yaw` ra **+** | ✅ |

Khớp hết với hệ OpenCV. **Chốt:** `Fwd = z`, `Left = −x`, `ψ = wrap(180° − yaw)`, `θ_d = −yaw`.
Thế số §8.7: `P_tag = (z + L, −x + d)` với `L = 0.13765`, `d = −0.07535`.

Còn: ghi kết quả vào comment đầu `tag_control.py`.

### 8.2. Lộ trình và ước lượng thời gian

Thứ tự: **mắt** (camera đọc đúng) → **não** (Astolfi) → **chân** (bánh).

| # | Bước | Thời gian | Chạy motor? |
|---|---|---|---|
| 0 | Kiểm dấu + đo 6 hằng số `L, d, a, t_n, t_l, D` + calib camera 640×480 | ½ ngày | ❌ |
| 1 | Mắt: trạm 1–3, in `G` ra và đo đối chiếu bằng thước | 1–1,5 ngày | ❌ |
| 2 | Não: cho log cũ trong `Docking_bipedal/logs/` chạy qua `controller_step` (`sim2d/dock_math.py`), kiểm dấu `ω` | ½ ngày | ❌ |
| 3 | Nối vào xe phải: `(v, ω)` mỗi frame, mất tag → `0, 0`, nút dừng khẩn | ½–1 ngày | ✅ |
| 4 | Lên gain 3 nấc (§9.2): `k_α` → `k_ρ` → `k_β < 0` | 1–2 ngày | ✅ |
| 5 | Giám sát 4 chế độ SEARCH / RECOVER / REGULATE / ENGAGE | 1–2 ngày | ✅ |
| 6 | ENGAGE + vặn ren bằng động cơ 2 | 1,5–2 ngày | ✅ |

- Tổng **≈ 1,5–2 tuần**. Mốc đầu: **xe tự bò tới điểm chờ, đúng hướng, sau khoảng 1 tuần**.
- Dễ trễ nhất: yaw nhiễu (có thể phải in tag to hơn), lỗi khi đưa code lên Pi, bước 6.
- Sim Mốc 4 (bản đồ miền hội tụ) bỏ qua, thử thật học được nhiều hơn.

### 8.3. Vặn ren (§11 đã có hướng)

- Vặn ren bằng **động cơ 2 (servo ID 2), gắn theo ren male**. Đúng phương án "servo riêng ở mũi" của §11.
- ENGAGE: bánh đẩy thẳng `v` nhỏ, `ω = 0`; động cơ 2 vặn cùng lúc. Thân không xoay nên không sinh lực cắt ngang lên ren.
- Quy tắc tốc độ: **`v_engage ≤ pitch × số vòng/giây`**. Ren dẫn đường, bánh chỉ đi theo. Đẩy nhanh hơn ren tiến → dễ vào sai ren (cross-thread).
- Biết lúc siết xong: **đếm vòng làm chính**, **`Present_Load` vọt lên làm lưới an toàn** (vượt ngưỡng thì dừng ngay).
- Hiện bus module phải ([bipedal.py](../../src/bipedal_robot/bipedal.py)) chưa có ID 2, sẽ phải thêm.

Còn phải đo / trả lời:
- [ ] Ren male (động cơ 2) nằm trên xe A (phải) đúng không?
- [ ] Bước ren (pitch), mm/vòng.
- [ ] Số vòng để khoá chặt.
- [ ] Ren thuận hay ren nghịch.
- [ ] Ren male nhô khỏi mũi bao nhiêu mm (ảnh hưởng `t_n`, `D`).

### 8.4. Kiến trúc: `dock_node` tiến trình riêng trên Pi xe phải

```
                ┌──────────────────────────── Pi xe phải ───────────────────────────┐
laptop          │                                                                    │
sliderUI ──wifi─┼──► leg_server (luôn chạy, giữ cổng serial) ──► servo, bánh, đ/c 2 │
    │           │        ▲                                                           │
    └──wifi─────┼──► dock_node (luôn chạy, camera) ──localhost──┘                    │
                └────────────────────────────────────────────────────────────────────┘
```

- **`leg_server` chạy suốt, không bao giờ phải tắt/bật lại.** Nó là tiến trình duy nhất mở cổng serial (một cổng serial chỉ một tiến trình mở được).
- **`dock_node`** là thêm một client giống `sliderUI`, nhưng chạy trên Pi và có camera. Nó gửi lệnh lái + động cơ 2 vào `leg_server` qua `localhost` (~1 ms, không qua wifi). Lúc không dock thì ngồi im.
- Không nhét docking vào `leg_server`: camera 30 FPS + OpenCV sẽ tranh khoá GIL với vòng servo/IMU (vốn đã chậm, nền 5.6 Hz). Camera lỗi cũng không kéo sập server; watchdog tự dừng bánh.
- `dock_node` mở **cổng riêng** để UI gửi lệnh và hỏi trạng thái.

**UI = điều khiển từ xa**, không tính toán:

| Việc | Cách làm |
|---|---|
| Bắt đầu dock | `dock_start` (có thể kèm gain, `D`) |
| Xem tình hình ~5 Hz | `dock_feedback`: chế độ, thấy tag chưa, `ρ, α, β`, `v, ω` đang gửi |
| Dừng khẩn | `stop_drive` tới `leg_server`, **không cần vé**, chạy được ngay bây giờ |
| Thử vặn ren | Nút riêng cho động cơ 2, thử tay trước khi cho ENGAGE tự chạy |

**Bẫy vé lái:** server chỉ phát **một** vé. Lúc dock thì `dock_node` giữ vé. Bấm "Dock" là UI phải **tắt heartbeat lái và slider v/ω**, chỉ còn nút Dừng. Dock xong, UI lấy vé lại và làm việc khác tiếp.

**Hiệu năng:** tách tiến trình thì không dính GIL, Pi nhiều nhân nên hai bên chạy song song. Vẫn dùng chung tổng CPU và nhiệt.
- Giảm ảnh hưởng: `cv2.setNumThreads(1)` ở đầu `dock_node`, chạy bằng `nice -n 5`.
- Kiểm: đo tần số vòng `leg_server` khi chạy một mình và khi có `dock_node`, xem `htop`. Gần như không đổi là xong.

### 8.5. Sequence trọn vẹn vẫn ghép được

Có 2 kiểu đồng bộ, đừng gộp:

| Kiểu | Ví dụ | Cần nhanh | Đặt ở đâu |
|---|---|---|---|
| Theo bước: xong A mới làm B | tay về tư thế → dock → vặn ren | vài trăm ms | nhạc trưởng ở đâu cũng được, kể cả UI qua wifi |
| Cùng lúc: A và B song song | ENGAGE: bánh đẩy trong khi động cơ 2 vặn | vài chục ms | chung một bộ não: trong `dock_node` |

Bộ chạy sequence "Về transition" trong `sliderUI.py` chính là nhạc trưởng kiểu 1. Sequence docking dự kiến:

```
1. set_mode CAR                 → chờ: mode == CAR
2. tay về tư thế dock           → chờ: các khớp tới đích (±30)
3. dock_start                   → chờ: dock_feedback == "ARRIVED"   (hoặc "FAILED"/hết giờ → dừng)
4. engage (đẩy + vặn)           → chờ: "LOCKED"                     (hoặc tải vọt / hết giờ → dừng)
```

Điều kiện duy nhất: mỗi phần trả lời được "đang làm gì, xong chưa, có lỗi không". `leg_server` đã có `mode`, `servo_pos`. `dock_node` cần trạng thái `SEARCH / REGULATE / ARRIVED / LOCKED / FAILED`.

### 8.6. Việc tiếp theo

- [x] Bài kiểm dấu §9.3 (8.1): khớp OpenCV, `ψ = wrap(180° − yaw)`, `Left = −x`.
- [x] Đo 6 hằng số `L, d, a, t_n, t_l, D` (bảng §3.3 DOCKING_ASTOLFI), xem 8.7.
- [x] Chốt dấu: `d = t_l = −0.07535 m` (8.7). `t_n = 0` đã xác nhận.
- [x] Sửa `TAG_SIZE = 0.040` trong `tag_control.py` (tag in lại 4 cm).
- [x] Calib camera 640×480: đã có `Docking_bipedal/calibdatanew.npz` (26/9), `fx ≈ 349` khớp FOV 85.1°. Còn kiểm Pi có đúng file này chưa.
- [ ] Trả lời 5 câu về ren (8.3).
- [ ] Chốt nhạc trưởng nằm ở UI laptop hay script trên Pi.
- [ ] Chốt nhánh git: PR `feat/robot-mode` vào `main` rồi tách `feat/docking`, hay tách thẳng từ `feat/robot-mode`.

### 8.7. Hằng số cơ khí đã đo (2/10)

Quy ước dấu theo §3.1 DOCKING_ASTOLFI: `Fwd` dương = về phía mũi, `Left` dương = bên trái (nhìn từ sau xe về mũi).

| Ký hiệu | Đo được | Đổi ra m | Phía thật | Dấu |
|---|---|---|---|---|
| `L` | 137.65 mm | 0.13765 | camera **trước** trục bánh A | + |
| `d` | −75.35 mm | **−0.07535** | camera bên **phải** trục giữa A (nhìn từ sau A về mũi) | − |
| `a` | 177.2 mm | 0.1772 | tâm trục bánh A → **đầu male** (mặt xa nhất), trên trục giữa A | + |
| `t_n` | 0 mm | 0 | mặt tag trùng mặt trước B (female lõm vào trong B) | — |
| `t_l` | −75.35 mm | **−0.07535** | lỗ dock nằm bên **phải** tag (nhìn từ sau B về mũi B) | − |
| `D` | ≈ 200 mm | 0.20 | ở điểm chờ: **đầu male → miệng lỗ female** | + |
| `b` | 241.45 mm | 0.24145 | trùng `diff_drive_right.json` | + |
| `TAG_SIZE` | 40 mm | 0.040 | tag in lại; trong code vẫn là 0.034 | — |

**Chuỗi mốc đã chốt:** `[tâm trục A] ─a─> [đầu male] ─D─> [miệng lỗ female = mặt trước B = mặt tag]`. Tag dán trên mặt trước B, ngang miệng lỗ, nên `t_n = 0`. Ba số đo tiếp nối nhau, không chồng, không hở → `t_n + D + a` trong công thức `G` đúng như đo. Điểm chờ = lúc đầu male vừa cách miệng lỗ đúng `D`.

Hệ quả rút ra từ số đo:
- Camera nằm **sau** đầu male một đoạn `a − L = 39.55 mm`.
- Ở điểm chờ, camera cách mặt tag theo phương pháp tuyến ≈ `D + 39.55` ≈ **240 mm**.
- Bố trí **đối xứng**: camera lệch phải A, tag lệch trái B. Hai xe quay mặt vào nhau thì "phải của A" trùng "trái của B". Nên cắm xong camera nhìn thẳng vào tag, ở điểm chờ tag nằm gần giữa ảnh (lệch ngang ≈ 0).
- Vì đối xứng nên `D` **không bị FOV ép** (nếu lắp cùng phía, lệch 150.7 mm, `D` dưới khoảng 12 cm là tag văng khỏi khung). Muốn giảm `D` để rút ngắn đoạn chạy mù thì được.

**Vì sao cả hai đều âm:**
- `d`: `Left` dương = bên trái A. Camera bên phải → `d < 0`.
- `t_l`: trong công thức, `t_l` nhân với `(−sin ψ, cos ψ)` = hướng "bên trái của B" (B quay mặt về A). Lỗ dock nằm bên **phải** tag theo B → `t_l < 0`.

**Kiểm bằng điểm chờ** (xe A đứng đúng đích, thẳng hướng → `G` phải ra `(0, 0)`):
- Tag trong hệ thân A: `Fwd = a + D + t_n`, `Left = d` (thẳng trước camera), `ψ = π` → `cos ψ = −1`, `sin ψ = 0`.
- `G_fwd = (a + D + t_n) − (t_n + D + a) = 0` ✓
- `G_left = d + t_l·cos π = d − t_l = −0.07535 − (−0.07535) = 0` ✓
- Quy tắc rút ra: bố trí đối xứng ⇔ `t_l = d`.

**ENGAGE:** bánh đẩy `D` để đầu male tới miệng lỗ, rồi đẩy tiếp thêm đúng độ nhô `p` của male trong lúc vặn ren. Tổng quãng chạy mù = `D + p` (`p` chưa đo, câu hỏi ở 8.3).

**Bẫy ENGAGE chạy mù 20 cm.** Lúc vào ENGAGE còn sai tối đa `θ_done = 3°` và `ρ_done = 2 cm`. Đi thẳng 20 cm với hướng lệch 3° → trôi ngang thêm `200 × tan 3°` ≈ 10.5 mm. Cộng lại có thể lệch tới ~3 cm ở mặt dock. Cần phễu (mép vát) rộng hơn mức đó, hoặc siết `θ_done`, `ρ_done`, hoặc giảm `D`.

**Tag 40 mm:** chỉ to hơn tag cũ khoảng 18%, nên nhiễu yaw chỉ giảm cỡ đó (từ ±8–9° xuống khoảng ±7°). Điểm chờ gần hơn log cũ (0.24 m so với 0.3–0.6 m) nên có thể đỡ hơn. Đo lại nhiễu yaw ở 0.24 m trước khi chỉnh `θ_done`.

Còn thiếu để chạy được trạm 1–3:
- [x] Sửa `TAG_SIZE = 0.040` trong `tag_control.py`. Số này là cạnh **ô vuông đen ngoài cùng**, không tính viền trắng.
- [x] File calib 640×480: `calibdatanew.npz`.
- [ ] Độ rộng phễu / dung sai cơ khí của dock, cùng 5 câu về ren ở 8.3.
