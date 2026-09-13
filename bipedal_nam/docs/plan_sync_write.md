# Kế hoạch: chuyển ghi động cơ tuần tự → sync_write

> Mục tiêu: thay vì ghi từng động cơ một (`for` loop gọi `write_pos_ex`),
> gộp lệnh cho cả 6 khớp chân vào **1 packet** bằng `sync_write` của lerobot.
> Kết quả kỳ vọng: **18 packet → 3 packet** mỗi chu kỳ, khớp chuyển động đồng bộ.

---

## Sơ đồ kiến trúc phân tầng

```
                        ┌─────────────────────────────────────┐
                        │         CLIENT (gửi lệnh ZMQ)         │
                        │      ~25-33 lệnh move / giây          │
                        └──────────────────┬──────────────────┘
                                           │ 6 vị trí đích
                                           ▼
╔══════════════════════════════════════════════════════════════════════╗
║ TẦNG 3 — SERVER / ĐIỀU KHIỂN                                          ║
║ leg_server_debug/left.py   +   right.py                              ║
║ ┌──────────────────────────────────────────────────────────────┐    ║
║ │ CŨ:  for servo_id in 4..9:  robot.write_pos_ex(1 con)         │    ║
║ │      → 6 vòng lặp                                             │    ║
║ │ MỚI: robot.write_leg_positions_sync({6 con})  → 1 lời gọi     │    ║
║ └──────────────────────────────────────────────────────────────┘    ║
╚═══════════════════════════════════╤══════════════════════════════════╝
                                    │ gọi
                                    ▼
╔══════════════════════════════════════════════════════════════════════╗
║ TẦNG 2 — DRIVER ROBOT (wrapper)  ← CON VIẾT Ở ĐÂY                    ║
║ bipedal_robot/bipedal_left.py   +   bipedal_right.py                 ║
║ ┌──────────────────────────────────────────────────────────────┐    ║
║ │ write_pos_ex(1 con):  3× bus.write()      ← giữ lại để debug  │    ║
║ │ write_leg_positions_sync(6 con):  ← HÀM MỚI                                       │    ║
║ │     bus.sync_write("Acceleration",  {...})│    ║
║ │     bus.sync_write("Goal_Velocity", {...})                    │    ║
║ │     bus.sync_write("Goal_Position", {...}, normalize=False)   │    ║
║ └──────────────────────────────────────────────────────────────┘    ║
╚═══════════════════════════════════╤══════════════════════════════════╝
                                    │ gọi
                                    ▼
╔══════════════════════════════════════════════════════════════════════╗
║ TẦNG 1 — BUS (lerobot, vendor/submodule)  ← KHÔNG SỬA, chỉ dùng      ║
║ lerobot/motors/feetech → FeetechMotorsBus                           ║
║ ┌──────────────────────────────────────────────────────────────┐    ║
║ │ write(reg, motor, val)      : 1 register / 1 motor            │    ║
║ │ sync_write(reg, {id:val},   : GroupSyncWrite → txPacket()     │    ║
║ │            *, normalize=True, num_retry=0)   → 1 packet thật  │    ║
║ └──────────────────────────────────────────────────────────────┘    ║
╚═══════════════════════════════════╤══════════════════════════════════╝
                                    ▼
                            ┌───────────────┐
                            │  Serial bus   │  18 packet → 3 packet / chu kỳ
                            │  6× STS servo │  (khớp chuyển động đồng bộ)
                            └───────────────┘

  ┌─ Luồng song song (đã có sẵn, KHÔNG đổi) ────────────────────────┐
  │ read loop: bus.read("Present_Position") → so đích vs thực       │
  │ → đây là nơi bắt lỗi động (servo quá nhiệt/mất bước)            │
  └─────────────────────────────────────────────────────────────────┘
```

---

## Signature đã xác minh (từ source lerobot upstream)

```python
def sync_write(
    self,
    data_name: str,
    values: Value | dict[str, Value],
    *,
    normalize: bool = True,      # ⚠️ mặc định TRUE
    num_retry: int = 0,
) -> None:
```

- `normalize` và `num_retry` là **keyword-only** (sau dấu `*`) → phải gọi kèm tên.
- Gộp **1 packet thật** (`GroupSyncWrite` → `txPacket()` gọi 1 lần), không lặp trá hình.
- Normalize chỉ áp dụng khi `data_name` nằm trong `bus.normalized_data`.

### ⚠️ Cái bẫy `normalize`
- `write_pos_ex` gốc ghi `Goal_Position` bằng **raw ticks** với `normalize=False`.
- `sync_write` mặc định `normalize=True` → nếu quên, lerobot tưởng giá trị là thang
  chuẩn hoá `[-100, 100]` rồi tự `_unnormalize` → **vị trí bay loạn, chân đá lung tung**.
- ⇒ Lời gọi `Goal_Position` **bắt buộc** `normalize=False`.
- `Acceleration` / `Goal_Velocity` không nằm trong `normalized_data` nên để mặc định vô hại.

---

## Plan công việc

### Giai đoạn A — Viết tầng 2 (bên trái trước)
- [ ] A1. Trong `bipedal_robot/bipedal_left.py`: viết `write_leg_positions_sync(positions, speed, acceleration)`.
  - `positions: Dict[str, int]` (raw ticks).
  - 3 lời `sync_write` theo thứ tự **accel → velocity → position**.
  - `Goal_Position` **phải** `normalize=False`; hai cái kia mặc định được.
  - Cân nhắc `num_retry=5` cho `Goal_Position` (giống dòng 260) để chịu nhiễu bus.
- [ ] A2. **Giữ nguyên** `write_pos_ex` — không xoá, để làm chế độ debug từng con.

### Giai đoạn B — Nối tầng 3 (bên trái)
- [ ] B1. Trong `leg_server_debug/left.py` (~dòng 409-442): thay vòng `for ... write_pos_ex`:
  - Dựng dict `{motor_name: clamped_pos}` cho 6 khớp (giữ nguyên clamp theo `servo_limits`).
  - Gọi `write_leg_positions_sync(...)` **1 lần**, vẫn trong `with self.serial_lock`.
- [ ] B2. Xử lý `success_count`/`fail_count`: giờ là "cả cụm" — trả True/False theo việc
  `sync_write` có ném exception hay không.

### Giai đoạn C — Nhân đôi cho bên phải
- [ ] C1. Lặp lại A cho `bipedal_robot/bipedal_right.py`.
- [ ] C2. Lặp lại B cho `leg_server_debug/right.py`.
- [ ] C3. Kiểm tra range `servo_id` / `servo_map` bên phải có khác bên trái không.

### Giai đoạn D — Kiểm thử trên Pi
- [ ] D1. Xác nhận `"Goal_Position" in bus.normalized_data` (kiểm chứng cái bẫy normalize).
- [ ] D2. Test tĩnh: gửi 1 lệnh move → đọc `Present_Position` 6 khớp → khớp với đích.
- [ ] D3. Test động: chạy walking gait, quan sát độ mượt + đo lại tần suất packet.
- [ ] D4. Nếu nghi servo lỗi: bật lại `write_pos_ex` từng con để cô lập.

### Giai đoạn E — Dọn & commit
- [ ] E1. Commit khi test xong (chú ý branch hiện tại + thay đổi chưa commit của left/right).

---

## Ghi chú
- 2 file sửa mỗi bên: `bipedal_robot/bipedal_{left,right}.py` (tầng 2) và
  `leg_server_debug/{left,right}.py` (tầng 3). Tầng 1 (lerobot) KHÔNG đụng.
- Luồng đọc feedback (`Present_Position`) là nơi bắt lỗi động — không thay đổi.
```
