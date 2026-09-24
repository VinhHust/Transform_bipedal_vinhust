# Kế hoạch mô phỏng docking 2D bằng Python (bản gọn)

**Dự án:** Modular Bipedal Robot — docking giữa hai module ở chế độ xe hai bánh vi sai.  
**Phiên bản:** 2.0 — 24/09/2026. Thay bản 1.0 (quá rộng so với nhu cầu).  
**Mục tiêu duy nhất:** kiểm chứng **phần toán** (chuỗi đổi hệ, standby, luật Astolfi) và **nhìn thấy** nó chạy đúng.  
**Quan hệ với [DOCKING_ASTOLFI.md](DOCKING_ASTOLFI.md):** sim này kiểm trạm 2–4 (§9) của tài liệu đó, trong điều kiện lý tưởng. Lớp giám sát 4 chế độ (§6) không nằm trong sim.

> Kết quả cần đạt: từ quan sát tag giả lập, xe A tự tính standby của tâm bánh rồi tự chạy tới đó. Tại standby, male và female thẳng trục, khe hở đúng D. Có hình để nhìn, có test để chứng minh.

## 1. Phạm vi

### 1.1. Làm

- Hình học phẳng: compose, inverse, camera ảo lý tưởng, female, standby, sai số dock.
- Động học xe vi sai, tích phân Euler.
- Luật Astolfi chỉ tiến, clamp `v/ω`, logic dừng 3 nhánh.
- Vẽ: cảnh tĩnh, animation, đồ thị sau chạy, bản đồ nhiều điểm xuất phát.
- Test: số tính tay, bất biến, dấu.

### 1.2. Tạm hoãn

| Hoãn | Khi nào quay lại |
|---|---|
| Nhiễu yaw, lọc EMA | Trước khi chốt kích thước và vị trí tag (đòn bẩy nhiễu) |
| Trễ quan sát | Nếu vòng docking chạy qua WiFi/ZMQ |
| FOV, mất tag, SEARCH | Khi ráp camera thật |
| Sai số gá (`estimated_geometry ≠ true_geometry`) | Khi có số đo CAD |
| Giới hạn tốc độ từng bánh, `v_min` | Khi chạy motor thật |
| Lùi/RECOVER, pha cuối bám trục, ENGAGE | Sau khi Astolfi lý tưởng đạt |
| Va chạm footprint, CSV, chạy hàng loạt | Khi cần báo cáo |
| Adapter OpenCV → 2D | Test riêng, không qua sim (§10) |

**Giả định:** sàn phẳng, B đứng yên, mọi thứ gắn cứng, camera luôn thấy tag, không nhiễu, vận tốc thực bằng lệnh.  
→ Sim đạt chỉ chứng minh **toán đúng**. Chưa chứng minh xe thật dock được.

## 2. Cấu trúc mã

```text
Dev_Vinh_Minh/Docking_bipedal/sim2d/
├── dock_math.py        # thuần numpy, KHÔNG import matplotlib
├── dock_sim.py         # cấu hình cảnh, vòng lặp, vẽ
└── test_dock_math.py   # pytest hoặc unittest
```

Ba nguyên tắc:

1. `dock_math.py` không phụ thuộc matplotlib → sau này chép sang xe thật được.
2. Controller chỉ nhận **đích trong hệ A**. Không bao giờ nhận pose thật của A trong W.
3. Đơn vị nội bộ: mét, giây, radian. Chỉ đổi sang mm/độ khi in hoặc vẽ.

| Hàm (`dock_math.py`) | Vào | Ra |
|---|---|---|
| `wrap_angle` | góc | góc trong `[-π, π)` |
| `compose`, `inverse` | pose | pose |
| `virtual_camera` | `pose_A_in_W`, cảnh thật | `pose_T_in_C` |
| `estimate_female` | `pose_T_in_C`, hình học | `pose_F_in_A` |
| `compute_standby` | `pose_F_in_A`, hình học | `pose_G_in_A` |
| `controller_step` | `pose_G_in_A`, gain, giới hạn | `(v, ω)`, trạng thái, `(ρ, α, β)` |
| `integrate` | pose, `(v, ω)`, `dt` | pose mới |
| `dock_errors` | male, female, `D` (cùng hệ) | `(e_lateral, e_gap, e_angle)` |

"Hình học" = `pose_C_in_A`, `pose_F_in_T`, `pose_M_in_A`, `D`. Bản này dùng một bộ chung cho cả thế giới thật và controller.

## 3. Quy ước

### 3.1. Trục và góc

- Mọi hệ phẳng: x hướng trước, y hướng trái.
- Góc dương = ngược kim đồng hồ khi nhìn từ trên xuống.
- `wrap_angle(a)` đưa góc về `[-π, π)`.

### 3.2. Các hệ

| Hệ | Gốc | x dương |
|---|---|---|
| W — world | Điểm tuỳ chọn trên sàn | Sang phải màn hình |
| A — thân | Tâm trục hai bánh A | Phía trước A |
| C — camera | Camera chiếu xuống sàn | Hướng nhìn camera |
| T — tag | Tâm tag chiếu xuống sàn | Pháp tuyến tag, hướng ra phía A tới |
| F — female | Tâm miệng female | Từ miệng hướng ra ngoài |
| M — male | Mốc đầu male | Hướng male chĩa vào female |

- C và T là hệ phẳng tự định nghĩa, **không** phải hệ trục OpenCV.
- Mốc male là một điểm cố định trên trục male. Dùng đúng điểm đó mọi lúc để đo khe.

### 3.3. Tên biến

`pose_X_in_Y` = pose của hệ X nhìn trong hệ Y. Ví dụ `pose_T_in_C` = tag mà camera thấy.  
Không dùng tên trơn `offset`, `yaw` — dễ nhầm hệ.

## 4. Toán cần kiểm chứng

### 4.1. Ghép và nghịch đảo pose

```text
R(θ) = [[cos θ, −sin θ],
        [sin θ,  cos θ]]

compose(pose_X_in_Y = (p1, θ1), pose_Z_in_X = (p2, θ2))
    = (p1 + R(θ1) @ p2,  wrap(θ1 + θ2))            # = pose_Z_in_Y

inverse((p, θ)) = (−R(θ).T @ p,  wrap(−θ))
```

Hàm trả dữ liệu mới, không sửa mảng đầu vào.

### 4.2. Camera ảo — dùng hình học thật

```text
pose_C_in_W = compose(pose_A_in_W, pose_C_in_A)
pose_T_in_C = compose(inverse(pose_C_in_W), pose_T_in_W)
```

Không nhiễu, không FOV: luôn trả `pose_T_in_C`.

### 4.3. Female trong A — controller làm, chỉ dùng quan sát

```text
pose_T_in_A = compose(pose_C_in_A, pose_T_in_C)
pose_F_in_A = compose(pose_T_in_A, pose_F_in_T)
```

### 4.4. Standby

F = vị trí female, φ_F = hướng female ra ngoài (trong hệ A hiện tại).  
m, θ_M = vị trí và góc male trong A. D = khe hở.

```text
n   = (cos φ_F, sin φ_F)
θ_G = wrap(φ_F + π − θ_M)
G   = F + D·n − R(θ_G) @ m
```

Male thẳng trục (`m = (a, 0)`, `θ_M = 0`): `G = F + (a + D)·n`, `θ_G = φ_F + π`.  
Đây đúng là công thức G trong ASTOLFI §4 khi `pose_F_in_T = (t_n, t_l, 0)`. `θ_G` ở đây chính là `θ_d` bên ASTOLFI.

### 4.5. Sai số dock — thước đo thành công

M = đầu male, F = miệng female, cùng một hệ.

```text
n = (cos φ_F, sin φ_F)          # dọc trục female
s = (−sin φ_F, cos φ_F)         # ngang trục female

e_lateral = dot(M − F, s)
e_gap     = dot(M − F, n) − D
e_angle   = wrap(θ_male − φ_F − π)
```

Không dùng khoảng cách Euclid M–F thay cho khe dọc trục.

## 5. Động học và điều khiển

### 5.1. Động học (Euler)

```text
x += v·cos θ·dt
y += v·sin θ·dt
θ  = wrap(θ + ω·dt)
```

Dùng θ ở đầu bước cho cả x và y. `dt = 0.02 s`.

### 5.2. Astolfi

Quy ước dấu **giống DOCKING_ASTOLFI.md** (`k_β` âm):

```text
ρ = hypot(x_G, y_G)
α = atan2(y_G, x_G)
β = wrap(θ_G − α)

v = k_ρ·ρ
ω = k_α·α + k_β·β
```

Gain khởi điểm: `k_ρ = 0.5`, `k_α = 1.5`, `k_β = −0.6`. Thoả `k_ρ > 0`, `k_β < 0`, `k_α − k_ρ > 0`.

### 5.3. Clamp giữ nguyên độ cong

```text
s = min(1, v_max/|v|, ω_max/|ω|)     # bỏ tỷ số có mẫu = 0
v, ω = s·v, s·ω
```

Chia cả hai cùng hệ số → tỉ `v/ω` không đổi → đường cong không đổi, chỉ chậm hơn.  
Minh hoạ: `v_max = 0.15 m/s`, `ω_max = 1.0 rad/s`.

### 5.4. Logic dừng — 3 nhánh

```text
if   ρ > pos_tol and |α| > π/2:   OUT_OF_DOMAIN, dừng        # bản này chỉ tiến
elif ρ > pos_tol:                 Astolfi + clamp
elif |θ_G| > heading_tol:         v = 0, ω = k_h·θ_G, clamp   # xoay tại chỗ
else:                             DONE
```

- Dùng **cùng một** `pos_tol` cho ngưỡng chuyển nhánh và ngưỡng thành công. Tránh bẫy "dừng ở 3 cm mà đòi 2 cm".
- Nhánh xoay dùng θ_G (hướng đích so với thân), **không** dùng β. Khi ρ ≈ 0 thì α vô định, nên β vô nghĩa.
- Xoay tại chỗ quanh tâm bánh không đổi ρ → không nhảy qua lại giữa các nhánh.
- Ngưỡng minh hoạ: `pos_tol = 0.01 m`, `heading_tol = 2°`, `k_h = 1.0`.
- `max_time = 30 s` → `TIMEOUT`.

## 6. Vòng lặp

```python
while t < max_time:
    pose_T_in_C = virtual_camera(pose_A_in_W, scene)              # thế giới thật
    pose_F_in_A = estimate_female(pose_T_in_C, geom)              # từ đây: chỉ dữ liệu controller được thấy
    pose_G_in_A = compute_standby(pose_F_in_A, geom)
    cmd, status, polar = controller_step(pose_G_in_A, gains, limits)
    log.append((t, pose_A_in_W, pose_G_in_A, polar, cmd, status))
    if status != "RUNNING":
        break
    pose_A_in_W = integrate(pose_A_in_W, cmd, dt)
    t += dt

errors = dock_errors(male_in_W(pose_A_in_W), female_in_W, D)     # chấm điểm bằng hình học thật
```

Log là list trong RAM. Chưa cần CSV.

## 7. Hiển thị

### 7.1. Cảnh tĩnh

`ax.set_aspect("equal")`. Vẽ:

- A: hình chữ nhật, chấm tâm bánh, mũi tên hướng.
- Camera: chấm, mũi tên hướng nhìn, tia camera → tag.
- Tag: đoạn thẳng + mũi tên pháp tuyến. B: hình chữ nhật.
- Female: chấm + trục. Male: chấm + mũi tên.
- **"Bóng ma" A tại standby G**, nét đứt. Male của bóng ma phải cách female đúng D.

### 7.2. Animation

- A chạy, để lại vết tâm bánh và vết đầu male.
- Chữ ở góc hình: t, trạng thái, ρ, α, β, v, ω.
- Render 10–20 Hz, độc lập với `dt`. Có cờ tắt animation.

### 7.3. Đồ thị sau chạy (1 figure, 3 ô)

1. ρ(t) và sai số hướng θ_G(t).
2. α(t), β(t).
3. v(t), ω(t) — trước và sau clamp.

In ra `e_lateral`, `e_gap`, `e_angle` cuối cùng.

### 7.4. Bản đồ miền hoạt động

- Chạy không animation từ 10–20 pose đầu (lưới vị trí × vài góc).
- Vẽ chồng mọi quỹ đạo lên một hình.
- Tô màu theo kết quả: `DONE` / `OUT_OF_DOMAIN` / `TIMEOUT`.

Hình này cho biết Astolfi dùng được từ những chỗ nào.

## 8. Test

### 8.1. Hình học

| ID | Nội dung | Kỳ vọng |
|---|---|---|
| G01 | Fixture 1 (§8.3) | Khớp tính tay, sai < `1e-9` |
| G02 | Fixture 2 góc lẻ (§8.4) | Khớp số **bạn tự tính tay** |
| G03 | Dời camera/tag trên thân, giữ female cố định trong W | Standby trong W không đổi |
| G04 | Xoay + tịnh tiến toàn cảnh | `pose_G_in_A` và sai số dock không đổi |
| G05 | Góc sát ±π | Không nhảy 2π |

Vì sao cần G02: fixture 1 chỉ có góc 0 và π, nên sin = 0. Gõ nhầm sin/cos hoặc sai dấu sin vẫn pass.  
So sánh góc bằng hiệu đã wrap.

### 8.2. Động học và điều khiển

| ID | Nội dung | Kỳ vọng |
|---|---|---|
| K01 | `v > 0`, `ω = 0` | Đi thẳng theo hướng thân |
| K02 | `v = 0`, `ω > 0` | Quay trái tại chỗ |
| K03 | `v > 0`, `ω > 0` | Cung tròn sang trái, bán kính `v/ω` |
| C01 | Chính diện (fixture 1) | Tới standby, ω ≈ 0 suốt đường |
| C02 | Xuất phát `(x, y, θ)` và `(x, −y, −θ)`, trục female là trục gương | Hai quỹ đạo tâm bánh là ảnh gương |
| C03 | Dời camera/tag, giữ female (như G03) | Quỹ đạo trùng nhau |
| C04 | `dt = 0.01` so với `0.02` | Pose cuối gần như nhau |
| C05 | Đích nằm sau xe | `OUT_OF_DOMAIN` |

C02 là bài kiểm **dấu** rẻ nhất: sai dấu α hoặc β thì hai quỹ đạo không đối xứng.

### 8.3. Fixture 1 — đã tính tay

Chỉ là số để lập trình, không phải kích thước xe thật.

| Đầu vào | Giá trị |
|---|---|
| A trong W | `(−1.20, 0.00, 0)` |
| Tag trong W | `(0.00, −0.08, π)` |
| Camera trong A | `(0.10, −0.05, 0)` |
| Female trong tag | `(0.00, −0.08, 0)` |
| Male trong A | `(0.20, 0.00, 0)` |
| D | `0.10 m` |

| Kết quả | Giá trị |
|---|---|
| Camera trong W | `(−1.10, −0.05, 0)` |
| Tag trong camera | `(1.10, −0.03, −π)` |
| Female trong W | `(0.00, 0.00, π)` |
| Female trong A | `(1.20, 0.00, π)` |
| Standby trong W | `(−0.30, 0.00, 0)` |
| Standby trong A | `(0.90, 0.00, 0)` |
| Đầu male khi ở standby, trong W | `(−0.10, 0.00)` |
| `e_gap` | `0` (khe thật = 0.10 m) |

Ở standby camera vẫn lệch so với tag. Không cần "tag nằm giữa ảnh".

### 8.4. Fixture 2 — bạn tự tính tay trước khi code

| Đầu vào | Giá trị |
|---|---|
| A trong W | `(−1.00, 0.30, 15°)` |
| Camera trong A | `(0.10, −0.05, 10°)` |
| Tag trong W | `(0.00, −0.08, −150°)` |
| Female trong tag | `(0.00, −0.08, 0)` |
| Male trong A | `(0.20, 0.00, 0)` |
| D | `0.10 m` |

| Cần tính | Giá trị |
|---|---|
| Camera trong W | ? |
| Tag trong camera | ? |
| Female trong W | ? |
| Female trong A | ? |
| Standby trong W | ? |
| Standby trong A | ? |

Tính bằng tay hoặc máy tính bỏ túi, **không chạy code**. Làm tròn 4 chữ số, góc ghi bằng độ. Điền xong mới đưa vào G02.

## 9. Lộ trình

### Mốc 1 — Hình học + cảnh tĩnh

- [ ] `wrap_angle`, `compose`, `inverse`, `virtual_camera`, `estimate_female`, `compute_standby`, `dock_errors`.
- [ ] G01–G05 pass.
- [ ] Vẽ cảnh tĩnh §7.1.

**Đạt khi:** test pass; bóng ma standby đúng chỗ; trên hình, khe male–female = D.

### Mốc 2 — Xe chạy, chưa điều khiển

- [ ] `integrate`, K01–K03.
- [ ] Animation với lệnh hằng.

**Đạt khi:** 3 test pass, animation quay đúng chiều.

### Mốc 3 — Astolfi, bật gain từng bước (ASTOLFI §9.2)

1. `k_ρ = 0`, `k_β = 0`: xe chỉ xoay tại chỗ cho mũi chỉ vào standby → xác nhận dấu α. (Sẽ `TIMEOUT` vì v = 0 — đúng như mong đợi.)
2. Bật `k_ρ`: xe chạy tới standby, tới nơi mới xoay sửa hướng.
3. Bật `k_β`: quỹ đạo cong, tới nơi gần đúng hướng sẵn. So đồ thị bước 2 và 3.

- [ ] Clamp §5.3, logic dừng §5.4.
- [ ] C01, C04, C05.

**Đạt khi:** `DONE` với `|e_lateral| < 1 cm`, `|e_gap| < 1 cm`, `|e_angle| < 2°`.

### Mốc 4 — Bản đồ miền hoạt động

- [ ] C02, C03.
- [ ] Vẽ §7.4.
- [ ] Ghi 2–3 câu: xuất phát từ vùng nào thì Astolfi tới được standby.

**Đạt khi:** có hình và kết luận. **Dừng sim ở đây.**

## 10. Sau sim

**Giữ lại:** `dock_math.py` — đổi hệ, standby, Astolfi, `dock_errors`.

**Thay:**

| Trong sim | Trên xe thật |
|---|---|
| `virtual_camera` | AprilTag + `solvePnP` + adapter OpenCV → 2D (`Fwd = z`, `Left = −x`, ASTOLFI §3.1) |
| `integrate` | Gửi `(v_left, v_right)` xuống motor |
| Số fixture | Số đo CAD đã kiểm trên lắp ráp |

Adapter là nguồn bug số 1 và sim **không** kiểm được nó. Test riêng:

- Dựng `R` từ một yaw đã biết bằng `cv2.Rodrigues`, cho qua adapter, so với pose phẳng mong đợi. Không cần ảnh.
- Bài kiểm dấu trên xe thật (ASTOLFI §9.3).

**Tham khảo:** [PythonRobotics — Move to a Pose](https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/move_to_a_pose/move_to_a_pose.html). Các phiên bản viết dấu `k_β` khác nhau (có bản dùng `−k_β·β` với `k_β > 0`). Đối chiếu định nghĩa β và dấu trước khi chép code.
