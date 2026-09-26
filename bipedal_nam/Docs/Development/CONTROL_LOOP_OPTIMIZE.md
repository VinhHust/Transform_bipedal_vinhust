# Control loop optimize

> Kiến trúc hiện tại: 2 Pi làm low-level (IMU + động cơ), laptop chạy policy, nói
> chuyện qua WiFi bằng ZeroMQ. Đã đổi sang sync write động cơ.
>
> Nhược điểm gói trong 1 câu: **bộ não ở xa tay chân, nói chuyện qua WiFi**. Mỗi
> vòng là một chuyến "gọi điện", có lúc chậm, có lúc rớt.
>
> Tối ưu = **gọi ít hơn, gọi ngắn hơn, và khi rớt thì tay chân tự biết phải làm gì**.

File liên quan:
- [policy_run.py](../examples/RunRL%20/policy_run.py) — vòng lặp policy trên laptop
- [transformer.py](../src/bipedal_robot/transformer.py) — API nói chuyện với 2 chân
- [imu.py](../src/bipedal_robot/sensors/imu.py) — gộp 2 IMU
- [leg_server_left.py](../src/leg_server/leg_server_left.py) / [leg_server_right.py](../src/leg_server/leg_server_right.py) — chạy trên Pi

Trạng thái: ⬜ chưa · 🔶 đang · ✅ xong

---

## Hiện trạng đếm từ code

Mỗi vòng policy (20 Hz) hiện có:

| Chuyến đi mạng | Kiểu | Ai gọi | Cần cho obs? |
|---|---|---|---|
| Hỏi IMU trái | REQ (chờ trả lời) | thread nền IMU, 50 Hz | Có |
| Hỏi IMU phải | REQ | thread nền IMU, 50 Hz | Có |
| Hỏi feedback servo trái | REQ | `get_state_with_retry`, 20 Hz | **Không** |
| Hỏi feedback servo phải | REQ | `get_state_with_retry`, 20 Hz | **Không** |
| Lệnh move trái | PUSH (không chờ) | `set_joint_angles`, 20 Hz | — |
| Lệnh move phải | PUSH | `set_joint_angles`, 20 Hz | — |

Mỗi giây mỗi Pi nhận **~70 REQ** (50 IMU + 20 feedback), policy chỉ cần **20 mẫu**.
Các REQ đều tuần tự trái → phải, nên thời gian chờ = **tổng** 2 chuyến.

---

## Bậc 0 — Đo trước, đừng đoán

Không có số thì không biết sửa được gì.

| # | Việc | TT |
|---|---|---|
| 0.1 | Ghi `dt` thực mỗi vòng policy ra CSV. Vẽ histogram: trung bình, p99, % vượt 50 ms | ⬜ |
| 0.2 | Đo riêng `get_state_with_retry`, `perform_inference`, `set_joint_angles`. Cái nào ăn nhiều thì sửa cái đó | ⬜ |
| 0.3 | Đo phía Pi bằng [measure_server_timing.py](../../examples_client/measure_server_timing.py) | ⬜ |
| 0.4 | Đếm tỷ lệ mẫu IMU `stale` (đã có nhãn trong `imu.py`) | ⬜ |

**Mục tiêu:** p99 < 50 ms, `stale` < 1 %.

Câu hỏi tự trả lời trước khi code: đặt `time.perf_counter()` ở **đâu** trong vòng
lặp để `sleep` cuối vòng không làm sai số?

### Số đo nền đã có (2026-09-19)

Đo từ `examples_client/fuse.csv` (cột `dt_ms`, 337 mẫu), **chỉ lấy IMU qua REQ/REP
WiFi** — chưa có lệnh động cơ, chưa có inference:

| Chỉ số | Đo được | Mục tiêu |
|---|---|---|
| Tần số trung bình | **5.6 Hz** | 20 Hz |
| dt median (p50) | **93 ms** | 50 ms |
| dt p99 | **886 ms** | < 50 ms |
| dt max | **916 ms** | — |
| % vòng vượt 50 ms | **57 %** | < vài % |

Chẩn đoán 2 tầng:
- **Median 93 ms (bệnh nền):** hỏi 2 IMU **tuần tự** qua WiFi (~40 ms/chuyến × 2).
  → thuốc: mục 1.3 (song song), Bậc 2 (PUB/SUB bỏ khứ hồi).
- **Spike 886 ms (đột biến):** median 93 mà p99 886, chênh ~10×. Dấu hiệu **WiFi
  power-save** / **mDNS `.local`** / **REQ retry timeout**, không phải REQ chậm đều.
  → thuốc **rẻ nhất, làm trước:** 4.2 (tắt power-save), 1.5 (IP tĩnh), 4.1 (dây Ethernet).

Quy trình: mỗi lần sửa **1 thứ** → chạy lại `collect_imu_fusion.py` → so `dt_ms` với
bảng này. Đừng sửa nhiều thứ rồi mới đo.

---

## Bậc 1 — Cắt việc thừa (1 buổi, không đổi kiến trúc)

| # | Việc | Vì sao | TT |
|---|---|---|---|
| 1.1 | Bỏ 2 lệnh feedback servo khỏi vòng policy | Obs 44D không dùng `servo_pos`. 2 chuyến WiFi khứ hồi mỗi vòng cho thứ không ăn. Giữ lại chỉ để log 1 lần/giây | ⬜ |
| 1.2 | Sửa bug speed = 500 | `_configure_servo_speed(leg, None)` `return True` mà không gửi gì ([transformer.py:479](../src/bipedal_robot/transformer.py#L479)). Sau `initialize()` servo kẹt speed 500 suốt policy, default server là 3400 | ⬜ |
| 1.3 | Hỏi 2 chân song song | Hiện `left.read_imu()` xong mới `right.read_imu()`. Gửi cả 2 trước rồi nhận cả 2 (`zmq.Poller` hoặc 2 thread) → chờ = chuyến **chậm nhất** thay vì **tổng** | ⬜ |
| 1.4 | Tắt log thừa trên Pi | Kiểm tra level thực khi chạy. Ghi journald 50 dòng/giây trên Pi tốn I/O | ⬜ |
| 1.5 | IP tĩnh thay `.local` | mDNS đôi khi mất vài trăm ms để phân giải. Tốn lúc connect và lúc reconnect sau timeout | ⬜ |

---

## Bậc 2 — Pi tự phát, laptop chỉ nghe (1–2 ngày) — đáng giá nhất

**Vì sao:** REQ/REP là "hỏi – đáp". Laptop gửi câu hỏi, chờ, Pi trả lời. Mỗi mẫu tốn
**2 lượt** đi trên mạng. Chuyển sang **PUB/SUB**: Pi cứ có mẫu IMU mới là **phát** luôn
(50 Hz), laptop **nghe**. Mỗi mẫu chỉ **1 lượt**, và laptop không bao giờ bị kẹt chờ.

| # | Việc | Chi tiết | TT |
|---|---|---|---|
| 2.1 | Pi: thêm socket PUB, phát trong `imu_loop` | Gói tin: `quat, gyro, t_sample, seq`. `seq` tăng dần để phát hiện mất gói | ⬜ |
| 2.2 | Laptop: socket SUB với `zmq.CONFLATE = 1` | Chỉ giữ **mẫu mới nhất**, cũ tự vứt. Không có hàng đợi tích tụ → không đọc số của 300 ms trước | ⬜ |
| 2.3 | `_imu_background_loop` thành "nghe và ghi cache" | Không còn gọi `read_imu()` | ⬜ |
| 2.4 | Giữ REQ/REP cho việc hiếm | Config speed, home, đọc feedback khi debug | ⬜ |
| 2.5 | Pi: rút hết hàng đợi PULL, lấy lệnh cuối | PUSH/PULL là hàng đợi. Laptop lag 200 ms rồi bắn dồn 4 lệnh → Pi chạy cả 4, cái đầu đã cũ. Sửa: `NOBLOCK` trong `while True` tới khi `zmq.Again`, chỉ áp dụng lệnh cuối. Hoặc thêm `seq`, bỏ lệnh có `seq` nhỏ hơn lệnh vừa chạy | ⬜ |

---

## Bậc 3 — Tay chân tự bảo vệ khi bộ não im lặng (nửa ngày)

Ngoài đời mạng **sẽ** rớt. Câu hỏi không phải "có rớt không" mà "rớt thì robot làm gì".

| # | Việc | Chi tiết | TT |
|---|---|---|---|
| 3.1 | Watchdog trên Pi | Không nhận lệnh `move` trong 200 ms → giữ nguyên vị trí, hoặc từ từ về initial pose. Hiện không có → robot đứng đơ ở tư thế cuối, có thể đang gác chân giữa không trung | ⬜ |
| 3.2 | Watchdog trên laptop | IMU cache `stale` quá 3 mẫu liên tiếp → dừng gửi lệnh, về pose an toàn. Hiện `get_state_with_retry` chỉ log warning rồi vẫn trả số cũ cho policy | ⬜ |
| 3.3 | Nội suy trên Pi (tùy chọn) | Laptop gửi đích 20 Hz, Pi chia 4 bước 80 Hz ghi servo. Mượt hơn, mất 1 gói vẫn còn đường đi tới đích cũ. STS đã có profile `speed/accel` làm một phần — đo Bậc 0 rồi mới quyết | ⬜ |

Câu hỏi: policy nhận số IMU của 200 ms trước thì sẽ làm gì?

---

## Bậc 4 — Phần cứng mạng (tiền + 1 buổi)

| # | Việc | Chi tiết | TT |
|---|---|---|---|
| 4.1 | Dây Ethernet thay WiFi | Cú nhảy lớn nhất về jitter: WiFi p99 vài chục ms, Ethernet p99 < 1 ms. Thử nghiệm treo giá thì cắm dây được. Nếu phải không dây: router riêng 5 GHz, không dùng WiFi trường | ⬜ |
| 4.2 | Tắt WiFi power-save trên Pi | `iw dev wlan0 set power_save off`. Pi ngủ WiFi để tiết kiệm điện → gói đầu sau khi ngủ trễ 100+ ms. Rất hay gặp, rất khó nhận ra | ⬜ |
| 4.3 | Ưu tiên tiến trình trên Pi | `chrt -f 50 python leg_server_left.py` hoặc `nice -n -10`. Không phải real-time thật, nhưng giảm bị thread khác chen | ⬜ |

### Router riêng: TP-Link Archer C54

**Chỉ cần cắm nguồn, không cần dây WAN.** Router làm 2 việc tách biệt: (1) tạo
mạng LAN để các máy nói chuyện với nhau, (2) nối LAN ra internet qua cổng WAN.
Robot chỉ cần việc (1). Cắm điện là xong.

Cái mất khi không có internet: Pi không `pip install`, không `git pull`, không
đồng bộ giờ NTP. Giờ lệch không sao — Pi dùng `time.monotonic()`, laptop so
`t_sample` với chính nó, không so đồng hồ 2 máy. Cần cài gì thì cắm WAN tạm rồi rút.

**Ba mức cắm dây** — không bắt buộc mức nào, để số đo Bậc 0 quyết:

| Mức | Dây gì | Đoạn WiFi trên đường lệnh | Khi nào |
|---|---|---|---|
| 1 | Chỉ nguồn router | 2 (Pi → router → laptop) | Robot đi tự do. Chạy được, jitter cao nhất |
| 2 | Nguồn + laptop cắm LAN | 1 (Pi → router) | **Khuyên dùng.** Laptop đứng yên cạnh router, robot vẫn tự do |
| 3 | Nguồn + laptop + 2 Pi cắm LAN | 0 | Treo giá, debug timing |

Cách quyết: chạy mức 1 với Bậc 0. p99 < 50 ms và `stale` < 1 % thì giữ mức 1.
Không đạt → cắm 1 dây cho laptop (mức 2), đo lại. Thứ làm hỏng mức 1 không phải
WiFi mà là **nhiễu** từ WiFi khác cùng kênh — phòng lab trường thường có.

**Cài đặt router** (trang quản trị `192.168.0.1`):

| # | Việc | Vì sao | TT |
|---|---|---|---|
| R1 | Tách tên WiFi 5 GHz khác 2.4 GHz, cho laptop + 2 Pi nối 5 GHz | 2.4 GHz xa nhưng chậm, nhiều nhiễu; 5 GHz gần nhưng nhanh, sạch. Để cùng tên thì thiết bị tự chọn, có thể rơi về 2.4 | ⬜ |
| R2 | *DHCP → Address Reservation*: gán IP cố định cho 2 Pi | Rồi sửa `mobile1.local` / `mobile2.local` thành IP đó (mục 1.5) | ⬜ |
| R3 | Kiểm tra **AP Isolation** đang **tắt** (mặc định tắt) | Bật lên thì các máy WiFi không nhìn thấy nhau | ⬜ |
| R4 | Đặt router gần chỗ robot đi, trong vài mét, không qua tường | Giảm mất gói | ⬜ |
| R5 | Tắt power-save WiFi trên Pi (mục 4.2) | — | ⬜ |

---

## Bậc 5 — Dời bộ não lại gần (1 tuần, chỉ khi các bậc trên chưa đủ)

| # | Việc | Chi tiết | TT |
|---|---|---|---|
| 5.1 | Chạy policy trên 1 Pi (master), Pi kia slave | MLP 44→6 trên Pi 5 CPU < 1 ms. Bỏ laptop khỏi đường điều khiển; laptop chỉ xem log. Vẫn còn 1 hop Pi↔Pi | ⬜ |
| 5.2 | Nối 2 Pi bằng Ethernet trực tiếp | Không qua router. Hop này ~0.3 ms | ⬜ |
| 5.3 | STM32 + xung đồng bộ phần cứng | Xem [bipedal-sync-architecture.md](../../bipedal-sync-architecture.md). Bài toán khác, chưa cần | ⬜ |

---

## KHÔNG cần làm

- **JSON → binary** (msgpack/struct): tiết kiệm < 0.2 ms mỗi gói. WiFi jitter đang ở chục ms.
- **Tăng tần số policy**: sim học 20 Hz thì real 20 Hz. Nhanh hơn không tốt hơn.
- **Tối ưu inference**: mạng nhỏ, đã < 1 ms.

---

## Thứ tự đề xuất

```
Bậc 0 (đo)
  → Bậc 1: 1.1, 1.2, 1.3
  → Bậc 3: watchdog 3.1, 3.2
  → Bậc 4: dây Ethernet 4.1 + tắt power-save 4.2
  → ĐO LẠI
  → quyết có làm Bậc 2 không
```

Watchdog và dây mạng xếp **trước** PUB/SUB vì rẻ hơn nhiều và giải quyết đúng thứ
làm robot ngã (mất gói, trễ đột biến). PUB/SUB tốt nhưng phải viết lại cả 2 phía.
