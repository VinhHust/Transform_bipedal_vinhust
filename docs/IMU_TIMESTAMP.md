# Đóng dấu thời gian IMU (Timestamp at Source)

Tài liệu này mô tả **vấn đề độ trễ/tuổi mẫu IMU** trong kiến trúc hiện tại và
**cách giải quyết bằng đóng dấu thời gian tại nguồn** (timestamp at source),
kèm hướng dẫn sửa code từng bước.

---

## 1. Bối cảnh kiến trúc

- Có **2 IMU vật lý** — mỗi Pi (chân trái/phải) 1 con ICM20948.
- Mỗi Pi **tự chạy Madgwick** cục bộ (`leg_server`), ra quaternion + gyro.
- Laptop (`IMUFusion` trong `sensors/imu.py`) **đọc cả 2 IMU** qua ZMQ REQ/REP,
  đưa về hệ `base_link`, rồi **fuse thành 1 hướng thân** (`fuse_quat`, `fuse_gyro`).
- Luồng nền trong `transformer.py` gọi `get_fused_imu()` ~50Hz (thực tế thấp hơn,
  giật), lưu vào `imu_cache`.
- Policy (`policy_run.py`) đọc cache mỗi **50ms (20Hz)**.

> Lưu ý: `imu_server.py` (WebSocket cổng 8765) là bộ xem 3D HTML **riêng biệt**,
> KHÔNG nằm trong đường policy.

---

## 2. Vấn đề

### 2.1. Timestamp hiện tại "nói dối"

Cache gắn thời gian lúc **laptop cất mẫu vào cache**
(`transformer.py:145`, `timestamp = time.time()`), KHÔNG phải lúc IMU **thật sự
lấy mẫu** trên Pi.

Giữa 2 mốc đó là cả quãng: fuse trên Pi → gửi qua WiFi → fuse trên laptop.
Nên con timestamp **trẻ hơn tuổi thật** của dữ liệu → độ trễ thật bị giấu đi.

**Ẩn dụ:** giống hộp sữa ghi ngày lúc *về tới siêu thị* thay vì ngày *vắt sữa*.
Bạn tưởng sữa mới, thực ra nó đã đi đường mấy ngày.

### 2.2. Tuổi mẫu dao động (latency jitter)

Ba nhịp chạy lệch tốc độ và không đồng bộ:

```
IMU Pi (~50Hz) → fuse laptop (~30Hz, giật do WiFi blocking) → policy (20Hz, 50ms)
```

Mỗi 50ms policy nhặt **mẫu mới nhất đang có trong cache**. Tùy hai nhịp rơi vào
đâu, mẫu đó có thể **mới ~0ms** hoặc **đã cũ ~40ms**. Kết quả: policy nhận dữ liệu
với **độ trễ nhảy 0–40ms mỗi bước**.

Trong môi trường train (IsaacLab) độ trễ thường bằng 0 hoặc cố định → đây là khác
biệt sim-to-real trực tiếp, gây **rung/kém mượt** và bào mòn biên độ ổn định.

### 2.3. Ngưỡng "cũ" 200ms quá lớn

`get_state_with_retry` chỉ báo stale khi mẫu > **200ms** (`transformer.py:635`).
Nhưng vòng policy là 50ms → 200ms = **4 bước điều khiển**. Nếu luồng IMU kẹt,
policy có thể hành động trên hướng thân **cũ 200ms** mà không hề biết → té.

### 2.4. Fuse 2 IMU lệch pha

Hai Pi là 2 máy riêng, 2 đồng hồ riêng. Mẫu trái lấy lúc `t`, mẫu phải lúc
`t+Δ`. Laptop fuse như thể **cùng một thời điểm**. Lúc đứng yên thì ổn; lúc đang
đi (thân lắc nhanh) → hướng thân fuse ra bị **nhòe/lệch**.

---

## 3. Cách giải quyết: đóng dấu thời gian tại nguồn

**Ý tưởng:** ghi kèm **thời điểm IMU thật sự lấy mẫu, ngay tại Pi**, rồi gửi con
số đó đi cùng dữ liệu.

**Timestamp KHÔNG xóa độ trễ — nó cho ta *biết* độ trễ là bao nhiêu.** Từ đó:

1. **Đo** được tuổi thật của mẫu.
2. **Loại** mẫu ôi (cũ quá ngưỡng → dừng an toàn).
3. **Căn thời gian 2 IMU** trước khi fuse (giảm lệch pha ở 2.4).
4. **Bù trễ** bằng ngoại suy gyro (nâng cao, tùy chọn).

**Điều kiện bắt buộc đi kèm:** 3 máy phải **đồng bộ đồng hồ** (chrony/NTP), nếu
không thì hiệu số thời gian vô nghĩa. Độ chính xác của timestamp **bị chặn trên
bởi chất lượng đồng bộ đồng hồ** (WiFi ~ vài ms).

> **Nguyên tắc vàng: đo trước, siết sau.** Đừng siết ngưỡng khi chưa có số thật.

---

## 4. Hướng dẫn code từng bước

### Bước 0 — Đồng bộ đồng hồ (chrony) — LÀM TRƯỚC

Lấy **laptop làm server**, 2 Pi làm client.

**Trên laptop** (`/etc/chrony/chrony.conf`, thêm cuối file) — phát giờ trong LAN
kể cả khi không có internet:

```
local stratum 10
allow 192.168.0.0/16        # sửa đúng subnet mạng của bạn
```

**Trên mỗi Pi** (`/etc/chrony/chrony.conf`) — trỏ về laptop, sync dày:

```
server <IP_LAPTOP> iburst prefer minpoll 2 maxpoll 4
makestep 1.0 -1
```

Khởi động lại + kiểm tra:

```bash
sudo systemctl restart chrony
chronyc tracking        # xem "System time ... offset"
chronyc sources -v      # phải thấy laptop, offset nhỏ
```

**Điều kiện đạt:** offset mỗi Pi so với laptop cỡ **dưới vài ms**. Đây là giới
hạn chính xác của toàn bộ phép đo timestamp.

---

### Bước 1 — Đóng dấu tại nguồn (trên Pi)

Sửa **cả 2 file** `leg_server_left.py` **và** `leg_server_right.py` giống hệt.

**(a)** Trong `update_imu_data()`, chụp thời điểm **ngay sau khi đọc phần cứng**
(quanh dòng 262–263 và 310–311):

```python
if IMU.dataReady():
    IMU.getAgmt()
    t_sample = time.time()          # ← ĐÓNG DẤU ngay sát lúc lấy mẫu vật lý
    ...
    with self.imu_lock:
        self.state_data["imu"] = list(q_new)
        self.state_data["imu_t_sample"] = t_sample   # ← lưu kèm
```

> Nhớ thêm `"imu_t_sample": 0.0` vào chỗ khởi tạo `state_data` để tránh KeyError.

**(b)** Trong `process_command()`, nhánh `elif cmd_type == "feedback"`
(dòng 420–434), gắn dấu vào gói trả về:

```python
"quat": imu_quat,
"gyro": [filtered_gx, filtered_gy, filtered_gz],
"t_sample": self.state_data.get("imu_t_sample", 0.0),   # ← THÊM DÒNG NÀY
```

---

### Bước 2 — Laptop đọc dấu về

Trong `sensors/imu.py`, hàm `read_imu()` (dòng 69–72):

```python
imu_data = {
    "quat": response.get("quat", [1, 0, 0, 0]),
    "gyro": response.get("gyro", [0, 0, 0]),
    "t_sample": response.get("t_sample", 0.0),   # ← THÊM
}
```

---

### Bước 3 — Lấy dấu "cũ nhất" khi fuse 2 IMU

Trong `get_fused_imu()` (kết thúc ~dòng 366). Kết quả fuse **chỉ tươi bằng cái
đầu vào cũ nhất**, nên lấy `min` của 2 dấu:

```python
# sau khi đã đọc left_data = self.left.read_imu(), right_data = self.right.read_imu()
t_src = min(left_data.get("t_sample", 0.0), right_data.get("t_sample", 0.0))

return {
    "fused_euler": ...,
    "fused_gyro": ...,
    "t_sample": t_src,          # ← dấu nguồn thật
    "timestamp": time.time(),   # giữ nguyên nếu chỗ khác đang dùng
}
```

---

### Bước 4 — Cache dùng dấu nguồn thay vì dấu lúc cất

Trong `transformer.py`, `_imu_background_loop()` (dòng 141–145):

```python
with self.imu_lock:
    roll, pitch, yaw = imu_data["fused_euler"]
    self.imu_cache["orient"] = (roll, pitch, yaw)
    self.imu_cache["gyro"] = imu_data["fused_gyro"]
    self.imu_cache["timestamp"] = imu_data.get("t_sample", time.time())  # ← dấu NGUỒN
```

Giờ `imu_cache["timestamp"]` mới **nói thật** về tuổi dữ liệu.

---

### Bước 5 — ĐO độ trễ thật (chưa đổi hành vi)

Trong `transformer.py`, `get_state_with_retry()` (dòng 640–641), đưa tuổi mẫu ra
ngoài:

```python
self.state_data["orient"] = orient
self.state_data["gyro"] = gyro
self.state_data["imu_age"] = time.time() - timestamp   # ← tuổi mẫu (giây)
```

Trong `policy_run.py`, chỗ log mỗi 2 bước (dòng 416–420), in thêm:

```python
age_ms = state.get("imu_age", 0.0) * 1000
print(f"... | IMU age: {age_ms:6.1f} ms")
```

Cho robot **treo, chạy dry-run vài phút**, ghi lại age **trung bình**, **lớn
nhất**, và **độ dao động**.

---

### Bước 6 — Siết ngưỡng dựa trên SỐ vừa đo

Sửa `get_state_with_retry()` (dòng 635–636). Đặt ngưỡng ≈ **max age quan sát được
+ biên an toàn** (thường **50–80ms**), và **coi vượt ngưỡng là lỗi phải dừng**:

```python
age = time.time() - timestamp
if age > 0.06 and timestamp > 0:      # 60ms thay cho 200ms — chỉnh theo số đo
    logger.error(f"IMU stale: {age*1000:.0f}ms → DỪNG AN TOÀN")
    return None                        # policy_run thấy None sẽ dừng (dòng 373)
```

---

## 5. Thứ tự thực thi an toàn

1. **Bước 0**, verify offset chrony **dưới vài ms** — không đạt thì sửa mạng trước.
2. **Bước 1–4** (đóng dấu + luồng dữ liệu). Dry-run, xác nhận không vỡ gì.
3. **Bước 5**, đo age thật khi robot treo.
4. **Bước 6**, siết ngưỡng theo số đo.

---

## 6. Giới hạn & bước tiếp theo

Phương án này làm độ trễ **minh bạch và an toàn hơn**, nhưng độ chính xác **bị
chặn bởi offset chrony ở Bước 0** (WiFi thường vài ms).

Nếu đo ra age **dao động lớn và loạn** dù đã làm hết → đó là **bằng chứng số** để
quyết định chuyển sang **MCU trung tâm** (1 đồng hồ nguồn, dây, sample 2 IMU cùng
lúc) — giải tận gốc bài toán đồng bộ liên chân mà timestamp chỉ *đo* chứ không
*xóa* được.

| Vấn đề | Timestamp (phương án này) | MCU trung tâm |
|---|---|---|
| Độ trễ WiFi giật | chỉ đo được | xóa (dây, xác định) |
| 3 miền đồng hồ | cần đồng bộ NTP, sai số vài ms | còn 1 đồng hồ |
| 2 IMU lệch pha | căn bằng nội suy | sample cùng lúc |
| Đồng bộ 2 chân | không giải được | giải triệt để |
