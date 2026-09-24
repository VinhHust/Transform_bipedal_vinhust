# Docking bằng AprilTag — Luật điều khiển Astolfi (xe 2 bánh vi sai)

> Điều khiển docking cho 2 xe vi sai cắm vào nhau bằng ren vặn.
> File liên quan: `Docking_bipedal/tag_control.py`, `camera_calibration.py`,
> `calibdatanew.npz`, `logs/tag_log_*.csv`. Tham khảo cơ khí: paper MobiDock.

---

## 0. Cấu hình đã chốt

| Hạng mục | Giá trị |
|---|---|
| Xe A (chủ động) | 2 bánh vi sai, mang camera, chạy Astolfi |
| Xe B (bị động) | 2 bánh vi sai, mang tag, **ĐỨNG YÊN + khoá bánh** |
| Vị trí dock | **Ở MŨI** cả hai xe |
| Cơ cấu khoá | Ren vặn (threaded screw-lock) |
| Tag | `tag36h11`, ID 0 |
| Camera | 640×480, MJPG, ~30 FPS |

**Quy tắc 2 vai:** B nhận lệnh HOLD qua ZMQ rồi khoá cứng; A chạy Astolfi vào dock.
❌ Không cho cả 2 xe cùng chạy "gặp nhau ở giữa" — 2 vòng phản hồi đánh nhau → dao động.

---

## 1. Vì sao dock ở mũi, bỏ mọi lệnh Slide

Xe vi sai **không đi ngang được** (ràng buộc non-holonomic), chỉ đi theo hướng mũi.
→ Dock ở sườn (kiểu omni) bất khả thi về hình học → **dời dock về mũi**.
→ Mọi lệnh `Slide LEFT/RIGHT` trong code cũ (viết cho omni) **vô nghĩa, bỏ hẳn**.

Quy về tốc độ 2 bánh (`b` = wheel base):

```
v_left  = v - ω·b/2
v_right = v + ω·b/2
```

---

## 2. Astolfi là gì

**Là phản xạ, không phải bộ giải.** Mỗi frame nó chỉ hỏi đúng một câu:
*"Đang lệch thế này, chạy bao nhanh và bẻ lái bao nhiêu cho 33 ms tới?"* → ra 2 số `v, ω`.
Không nhớ gì, không lập quỹ đạo. Đường cong mượt ta thấy **tự hiện ra** từ việc lặp phản xạ
đó 30 lần/giây (như đi bộ về phía cửa — không ai vẽ quỹ đạo trong đầu).

**Là điều khiển tới TƯ THẾ (pose), không phải tới điểm** → tới đúng chỗ **và** đúng hướng.
Đó là công dụng của số hạng `β`: nó thay thế waypoint. **Không có điểm trung gian** — Astolfi
đi thẳng tới tư thế đích. Nếu còn phải chèn waypoint thì `β` đã vô nghĩa.

Đích = tư thế lúc 2 xe đã cắm vào nhau, lùi lại khe hở `D`. `D` là **hằng số cơ khí đo bằng
thước**, không phải waypoint điều hướng.

---

## 3. Hệ quy chiếu và hằng số cơ khí

### 3.1. Đổi hệ toạ độ ngay sau `solvePnP`

Camera cho hệ khó chịu (`x` phải, `y` xuống, `z` trước). Toán chuẩn giả định `x` trước,
`y` trái, góc dương = ngược kim đồng hồ. Trộn 2 cái = 90% bug docking. Quy đổi một lần:

```
Fwd  =  z        # trước, mét
Left = -x        # trái dương, mét
θ    :  dương = ngược kim đồng hồ (nhìn từ trên)
```

### 3.2. Camera KHÔNG nằm ở tâm quay — vì sao phải dời gốc

Xe vi sai quay quanh **điểm giữa trục 2 bánh**. Động học chuẩn
(`X += v·cosΘ·dt`, `Θ += ω·dt`) mô tả chuyển động của **đúng điểm đó**. Astolfi được **suy ra
từ** động học này → mọi `(ρ, α, β)` phải tính cho **tâm trục bánh**, không phải cho camera.

Camera lệch khỏi tâm quay một đoạn `(L, d)`. Khi xe xoay tại chỗ, camera có thêm vận tốc
`ω × offset` mà công thức không biết → camera **vẽ một cung**, thấy tag *dịch ngang* dù xe chỉ
xoay. Bỏ qua → xoay để chỉnh hướng lại kéo lệch tín hiệu ngang → **rung quanh đích mãi**.

> **ELI5:** camera dán ở **vai phải**, còn bạn muốn ấn **rốn** (giữa) vào đích. Xoay người thì
> vai phải quét cung → hình đích nhảy ngang dù rốn gần như đứng yên. Não tưởng "tôi lệch" →
> chỉnh lại → nhảy tiếp → rung.

**Sửa: một phép biến đổi cứng, đo thước, làm một lần.**

```
P_body = R(φ_mount)·P_cam + (L, d)
```

**Camera lắp thẳng (không nghiêng) → `φ_mount = 0` → co lại còn một phép cộng vector:**

```
điểm-ở-tâm-bánh = điểm-camera-đo + (L tiến, d ngang)
```

Chỉ 1 dòng, không ma trận xoay. Không lý do gì bỏ.

**`(L, d)` là vector 2D trong mặt sàn — phải dịch cả 2 chiều:** `L` (trước/sau) và `d` (ngang).
`L` hầu như **≠ 0 dù không cố ý** (camera bắt vào khung, hiếm khi đúng đường trục bánh) → **đo
cả hai bằng thước**, đừng mặc định `L=0`. Độ cao camera (dọc) **không dịch** — bài toán phẳng.

⚠️ **Bẫy 3 chữ "z":** `z` của camera nằm **ngang** (ra trước, = `Fwd`); "z lên trời" khi vẽ từ
trên xuống là trục **dọc** (robot xoay quanh nó, = `θ`). Hai cái vuông góc. → gọi 2 trục sàn là
`Fwd/Left`, đừng gọi `x,y` (đã bị camera chiếm).

Sau transform, **tag lẫn standby G đều nằm trong hệ gốc-tại-tâm-xe**, mà tâm xe = gốc = chính
robot `(0,0,θ=0)`. Nên `ρ,α,β` = "từ chỗ tôi bây giờ, standby còn cách bao xa/lệch hướng nào".
Hệ này **sinh mới mỗi frame** (ảnh chụp, không phải bản đồ phòng) → đó là lý do không cần odometry.

⚠️ **Mẹo đặt tag lệch đối xứng KHÔNG thay thế được phép dời này.** Đặt camera lệch phải trên A
+ tag lệch tương ứng trên B để lúc cắm xong camera thấy tag ở `(0,0)` — mẹo đó chỉ cứu **điểm
ĐÍCH** (bức ảnh cuối cùng cân đẹp). Nó **không** cứu **đường đi**: cái rung sinh ra từ camera
quét cung *lúc đang xoay*, xảy ra bất kể tag đặt ở đâu. Endpoint đẹp ≠ dynamics đúng.

> **Quy tắc vàng:** mọi công thức Astolfi làm việc với **tâm trục bánh của A** và **mặt dock
> của B**. Camera và tag chỉ là *dụng cụ đo*, phải quy về 2 điểm đó trước.

### 3.3. Bảy hằng số phải đo bằng thước

Sai một số → không gain nào cứu được.

| Ký hiệu | Là gì | Đơn vị |
|---|---|---|
| `L`, `d` | camera lệch so với tâm trục bánh A (trước, ngang) | m |
| `a` | tâm trục bánh A → mặt dock ở mũi A (**dock nằm trên trục giữa A** → thuần tiến) | m |
| `t_n` | tâm tag → mặt dock của B, phần **dọc pháp tuyến tag** | m |
| `t_l` | tâm tag → mặt dock của B, phần **ngang** (tag lệch bên so với dock) | m |
| `D` | khe hở khi kết thúc pha REGULATE | m (10–15 cm) |
| `b` | wheel base xe A | m |

> Cấu hình đã chốt: dock male trên **trục giữa A** (`a` thuần tiến); mặt tag **cùng hướng** dock
> female (hướng lệch = 0 → `θ_d=ψ+π` không đổi); tag **lệch bên** so với dock → sinh `t_l`.

**Vì sao cần:** camera thấy **tag**, nhưng tag ≠ đích; Astolfi lái **tâm bánh**, nhưng chỗ chạm
là **mặt dock ở mũi**. Các số này bắc cầu qua các chỗ lệch đó. Chuỗi cột mốc (dọc trục dock):

```
[tâm tag] ─t_n─> [dock B] ═ chạm ═ [dock A] ─a─> [tâm bánh A]
              (+ lệch ngang t_l)      ▲  lùi thêm D = điểm STANDBY (đích Astolfi)
```

- `t_n, t_l`: tag chỉ là "biển số nhà", mặt dock B mới là "cửa" → từ tag suy ra lỗ dock (cả tiến `t_n` lẫn ngang `t_l`). Đặt tag ở chỗ **dễ nhìn nhất**, không cần trùng trục dock — đo đủ 2 số là được.
- `a`: Astolfi biết vị trí tâm bánh, nhưng mũi nhô trước `a` → quên là **mũi đâm quá vào B**.
- `D`: dừng sớm cho an toàn (tới nơi lực=0, gần quá `α` loạn) → ENGAGE mù đẩy nốt.
- `b`: không liên quan đích, dùng ở khâu xuất chia `(v,ω)` ra 2 tốc độ bánh.

→ Vì thế `(t_n+D+a)` trong công thức §4 = **3 bước nhảy dọc trục cộng dồn**; `t_l` = số hạng
ngang tách riêng. Tag lệch bên **không sai** — chỉ tốn thêm 1 số đo + 1 số hạng.

**Lệch cảm biến vs lệch đích — đừng gộp:** camera lệch (`L,d`) là **phía đo** → transform xoá sạch,
vô hại. Dock/tag lệch (`a, t_l`) là **phía đích** → vào thẳng công thức `G`, không xoá được. Vì
thế **đừng** đặt tag lệch để "khớp" camera — vô ích cho điều khiển; đặt tag cho dễ nhìn, đo offset thật.

---

## 4. Công thức đầy đủ

Luật điều khiển chạy lại **từ đầu mỗi frame**, không tích luỹ, **không cần odometry/EKF**.
Thứ duy nhất có nhớ là bộ lọc EMA làm mượt số đo tag (trạm 2c, §9) — nó chỉ làm mượt, không phải bản đồ.

```
1. solvePnP    → tư thế tag trong hệ CAMERA
2. đổi hệ      → tư thế tag trong hệ THÂN A (dùng L, d, φ_mount)
                 P_tag = (Fwd_t, Left_t) ;  ψ = hướng pháp tuyến tag trong hệ thân A

3. tư thế đích:
       θ_d = ψ + π                                       ← hướng A phải quay VÀO tag
       G   = P_tag + (t_n + D + a)·(cos ψ, sin ψ)        ← chuỗi dọc trục dock
                   + t_l·(−sin ψ, cos ψ)                 ← lệch ngang tag (bỏ nếu t_l=0)

4. Astolfi:
       ρ = sqrt(G_fwd² + G_left²)     ← còn cách bao xa
       α = atan2(G_left, G_fwd)       ← ngắm tới đích lệch mũi bao nhiêu
       β = θ_d − α                    ← tới nơi rồi thì lệch hướng bao nhiêu

       v = k_ρ·ρ
       ω = k_α·α + k_β·β
```

| Số hạng | Làm gì |
|---|---|
| `k_ρ·ρ` | càng xa càng nhanh, gần chậm dần → tự phanh mềm, không cần state machine |
| `k_α·α` | xoay mũi về phía đích ("nhìn vào chỗ muốn đến") |
| `k_β·β` | đánh lái ngược để tới nơi đúng hướng (như đỗ xe song song) |

`β` = **sai số hướng dự báo trước**: nếu đi thẳng từ đây, tới nơi mũi sẽ chỉ theo `α`, nhưng
hướng cần là `θ_d`; chênh lệch đó là `β`.

### 4.1. Điều kiện ổn định

```
k_ρ > 0        k_β < 0        k_α − k_ρ > 0
```

⚠️ `k_β` **ÂM**. Gõ nhầm dương → robot xoáy trôn ốc ra xa (lỗi hay gặp nhất).
Khởi điểm: `k_ρ = 0.5`, `k_α = 1.5`, `k_β = −0.6`.

### 4.2. Vì sao KHÔNG cần odometry

Tag là **gốc toạ độ**, mỗi frame là bài toán độc lập tính lại từ đầu:
- Đá robot lệch 20cm → frame sau tự sửa. Một bánh yếu / sàn trơn / trượt bánh → tự bù.
- Mất tag → `v=ω=0`, đứng chờ, thấy lại chạy tiếp.

**Chỉ cần đúng DẤU, không cần đúng LƯỢNG.** Vị trí là tích phân của vận tốc → "nhà máy" đã có
sẵn khâu tích phân → điều khiển P đủ, sai số xác lập = 0, không cần khâu I hay calib động cơ.
Cường độ sai → chỉ chậm/rung. **Dấu sai → hỏng hoàn toàn.**

---

## 5. Bốn thứ Astolfi mù → phải bọc lớp giám sát

Astolfi có chứng minh ổn định toàn cục (Lyapunov) nhưng với 4 giả định **đều sai trong lab**:
robot là 1 điểm, không vật cản, động cơ vô hạn, luôn thấy đích.

| Nó không biết | Hậu quả |
|---|---|
| Robot có thân, có kích thước | húc vào xe B |
| Camera chỉ thấy ~85° | xoay làm tag văng khỏi khung → tự chọc mù mình |
| Động cơ có giới hạn | `ω` bão hoà → tỉ lệ `v/ω` sai → đường đi khác tính toán |
| Có vùng không nên vào | không có khái niệm vùng cấm |

**Ở GẦN mà lệch nhiều thì Astolfi hỏng — 2 lý do:**

- **A) Bán kính cua co theo `ρ`:** `R = v/ω`; `ρ` nhỏ → `v` nhỏ nhưng `ω` vẫn lớn → `R` bé →
  robot quay tít tại chỗ, thân xe quét trúng xe B.
- **B) `α` phát điên khi `ρ` nhỏ:** cùng lệch 3cm, ở xa `α≈1.7°`, ở gần `α≈31°`. Nhiễu bị
  khuếch đại; tới `ρ=0` thì `α` vô định (**điểm kỳ dị toạ độ cực**) → rung mãi không ổn định.

---

## 6. Kiến trúc 4 chế độ

Bọc Astolfi bằng vài dòng `if` — một luật phản hồi ở lõi, một lớp logic mỏng xử lý những gì
luật đó mù.

```
GIÁM SÁT ── kiểm điều kiện, chọn chế độ
├─ SEARCH   ── chưa thấy tag → xoay tại chỗ về phía thấy lần cuối
├─ RECOVER  ── đích ở nửa sau xe (|α| > 90°) → LÙI THẲNG (v<0, ω=0)
├─ REGULATE ── Astolfi → standby; tới nơi thì xoay tại chỗ sửa hướng
└─ ENGAGE   ── MÙ: v=const + mô-men vặn ren, chạy T giây
```

**Thứ tự kiểm mỗi frame** (cái nào nguy hiểm nếu bỏ sót thì kiểm trước):

```
1. Không thấy tag                  → SEARCH
2. ρ < ρ_done                      → REGULATE, nhánh xoay tại chỗ: v = 0, ω = k_h·θ_d
      và |θ_d| < θ_done, giữ N frame → sang ENGAGE
3. |α| > 90°                       → RECOVER: lùi thẳng
4. Còn lại                         → REGULATE, nhánh Astolfi
```

- **Bước 2 đứng trước bước 3:** khi `ρ ≈ 0` thì `α` vô định (§5B), nên không được dùng `α` nữa.
- **Điều kiện xong dùng `θ_d`, KHÔNG dùng `β`:** `β = θ_d − α` cũng vô định khi `ρ ≈ 0`. Còn `θ_d`
  = "thân còn phải quay thêm bao nhiêu" luôn xác định. Camera song song thân + tag song song female
  → `θ_d = ±yaw` của tag (dấu chốt bằng bài kiểm §9.3).
- **Một ngưỡng `ρ_done` cho cả hai việc:** chuyển sang xoay tại chỗ và báo xong. Hai ngưỡng khác
  nhau (dừng ở 3 cm, đòi 2 cm) → robot đứng im mãi ngoài vạch đích.
- **Vì sao vẫn cần `|α| > 90°`:** Astolfi chỉ-tiến chỉ đúng khi đích ở nửa trước xe. Đây là giới
  hạn của toán, không phải của camera. Lùi thẳng làm đích trượt dần ra phía trước → chắc chắn kéo
  `|α|` về dưới 90°; `ω=0` nên tag không văng khỏi khung. 2 dòng code.

**Tạm hoãn (edge case camera):** góc nhìn nghiêng tag ±60°, tag sát mép ảnh. Lưới an toàn tạm thời:
mất tag → `v=ω=0` (§4.2).

**Pha ENGAGE tồn tại vì 2 lý do:**
1. Astolfi về đích với **lực = 0** (`v = k_ρ·ρ → 0`), mà ren cần lực ép → phải có pha ép riêng.
2. Phải bàn giao **trước khi `α` phát điên** (§5B). Dưới `D`, thị giác đang nói dối → ngừng
   nghe, đi thẳng, ép vào. `D = 10–15 cm` = ranh giới thoát vùng kỳ dị; càng ngắn càng tốt vì
   pha này chạy mù, sai số tích luỹ theo quãng đường.

---

## 7. Số đo thật của hệ thống

**Camera (`calibdatanew.npz`):** `fx=348.7 fy=347.8 cx=349.2 cy=255.4`; FOV ngang 85.1°,
dọc 69.2° (góc rộng → gần như không có đoạn mù).
⚠️ **`cx=349.2`, KHÔNG phải 320.** Code cũ hardcode `CENTER_X=320` → lệch 29 px. → bỏ `offset_x`
tính bằng pixel, dùng `atan2` trên toạ độ mét (đã có `cx` đúng qua `solvePnP`).

**Nhiễu yaw (log 117k frame @30 FPS):** 0.3–0.6 m → **±8.4°**; 0.0–0.3 m → **±9.7°**.
Camera chạy đều 30 FPS — **tốc độ vòng lặp không phải vấn đề**.
⚠️ Nhiễu ±9° > `TOLERANCE_YAW=5°` của code cũ → robot lắc tại chỗ vĩnh viễn.
Gốc: tag 34 mm quá nhỏ, `yaw` suy từ độ méo hình thang, tag nhỏ thì độ méo chỉ vài pixel.
Nhiễu tỉ lệ nghịch kích thước tag trên ảnh.

✅ **Việc rẻ nhất, hiệu quả nhất: in tag to hơn.** 34 mm → 100 mm giảm nhiễu ~3 lần (±9° → ±3°).
Dán lên **tấm phẳng cứng** (mica/nhôm) — mặt cong làm `solvePnP` sai hệ thống.

**Cơ khí (MobiDock):** dung sai ±5 mm, nhưng **mép vát hình phễu làm nốt** — thị giác chỉ cần
đưa vào phễu bắt. Giới hạn nhận diện ±60° lệch góc (tạm chưa dùng — edge case camera hoãn, §6).
> 💡 Mẹo quan trọng nhất **nằm ở cơ khí, không phải thuật toán**: vát mép dock thành phễu.
> Phễu 4 cm biến "chính xác 2 mm" thành "chính xác 2 cm" — dễ hơn 10 lần, miễn phí.

---

## 8. Tham số khởi điểm

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| `k_ρ` | 0.5 | |
| `k_α` | 1.5 | phải thoả `k_α − k_ρ > 0` |
| `k_β` | **−0.6** | ⚠️ phải ÂM |
| `D` | 0.10–0.15 m | khe hở cuối REGULATE |
| `α_max` | 90° | miền Astolfi chỉ-tiến; vượt → RECOVER |
| `ρ_done` | 0.02 m | dùng CHUNG: chuyển sang xoay tại chỗ **và** báo xong |
| `θ_done` | 3° | ngưỡng `\|θ_d\|` để xong |
| `k_h` | 1.0 | gain xoay tại chỗ `ω = k_h·θ_d` |
| EMA yaw | `a = 0.9` | τ ≈ 0.33 s @30 FPS; lọc `yaw` trước khi dựng `ψ` (trạm 2c) |
| Deadband `α` | 2° | tránh rung quanh cân bằng |
| Dung sai xong | `ρ<ρ_done`, `\|θ_d\|<θ_done`, giữ N frame | cơ khí quyết định |

**Chặn bắt buộc:** clamp `v, ω` về giới hạn phần cứng (chia cùng một hệ số để giữ tỉ `v/ω`);
`v_min` (thắng ma sát tĩnh). Xử lý gần đích và `|α|>90°` nằm ở giám sát §6, không lặp lại ở đây.

---

## 9. Triển khai — dữ liệu chảy qua 6 trạm

Mỗi frame đi đúng chuỗi này, **mỗi trạm một hàm riêng** (để test từng khúc không cần motor):

```
ảnh → [1]solvePnP → [2]đổi hệ cam→thân (+2c lọc EMA) → [3]tính đích G,θ_d
    → [4]Astolfi (v,ω) → [5]giám sát chọn chế độ → [6]clamp → motor
```

**Trạm 1 — tư thế tag.** Từ `solvePnP` lấy `tvec → x,y,z`; `rvec → yaw` (qua `Rodrigues`, lấy
đúng thành phần góc quanh trục đứng của tag). *Chỗ hay sai: xác định đúng trục đứng là trục nào.*

**Trạm 2 — đổi hệ (2 phép, đừng gộp).**
- 2a: xoay trục `Fwd=z, Left=−x` (đổi tên + dấu, rất ngắn).
- 2b: dời gốc camera → tâm trục bánh, cộng `(L, d)` (§3.2). Camera thẳng → `φ_mount=0` → chỉ
  một phép cộng.
- Ra: `Fwd_t, Left_t, ψ` — tất cả trong hệ thân A.

**Trạm 2c — lọc EMA (thứ duy nhất có nhớ).** Làm mượt **trước** khi tính đích, vì nhiễu yaw bị
đòn bẩy (khoảng tag→female, `D`, `a`) phóng to thành G nhảy lung tung.

```
est = est + (1 − a)·wrap(meas − est)     # góc: PHẢI wrap hiệu số
```

- **Lọc `yaw` (quanh 0° khi chính diện), rồi mới dựng `ψ`.** Đừng lọc `ψ` (quanh ±180°): trung bình
  thô của 179° và −179° ra 0° → sai hẳn 180°.
- **`Fwd_t, Left_t`:** `tvec` ít nhiễu hơn yaw nhiều (§9.3). Chỉ lọc nếu log cho thấy cần — lọc
  thêm là thêm trễ.
- **Đổi chác:** `a` lớn → mượt hơn nhưng trễ hơn, `τ ≈ dt/(1−a)`. Xe đang chạy/quay thì số đã lọc
  luôn chậm hơn thực tế ≈ `v·τ` (khoảng cách) và `ω·τ` (góc).
- **Nếu trễ gây rung:** trước khi trộn, xoay `est` ngược theo góc xe vừa quay trong `dt` (`ω·dt` từ
  lệnh hoặc gyro IMU) — "dự đoán rồi trộn". Chỉ bù 1 frame, không tích luỹ → vẫn không phải odometry.
- **Mất tag lâu** (vd > 0.5 s) → reset `est` bằng số đo đầu tiên khi thấy lại (số cũ đã sai hệ vì xe
  đã di chuyển).

**Trạm 3 — tư thế đích (§4 bước 3).** Chỉ hình học: `θ_d=ψ+π`,
`G=P_tag+(t_n+D+a)(cosψ,sinψ)+t_l·(−sinψ,cosψ)`. Gõ `t_n,t_l,a,D,b` thành hằng số có comment + đơn vị đầu file.

**Trạm 4 — Astolfi (lõi, ngắn nhất).** Hàm thuần toán `compute_control(G_fwd,G_left,θ_d)→(v,ω)`,
không camera/motor bên trong (→ test offline được). 3 điều bắt buộc:
1. `k_β` âm.
2. `α, β` là góc → **wrap về `[−π,π]`** (viết hàm `wrap_angle`), nếu không sẽ nhảy vọt quanh ±180°.
3. Hàm này không nhớ gì giữa các frame — việc nhớ nằm riêng ở trạm 2c.

**Trạm 5 — giám sát 4 chế độ (§6).** Máy trạng thái nhỏ SEARCH/RECOVER/REGULATE/ENGAGE.
*Tự thiết kế thứ tự kiểm điều kiện: cái nào nguy hiểm nếu bỏ sót thì kiểm trước.*

**Trạm 6 — chặn an toàn trước motor (§8).** Clamp (giữ tỉ `v/ω`); `v_min`;
cuối cùng đổi `(v,ω)→(v_left,v_right)`.

### 9.1. Thứ tự NÊN code (đừng làm một lượt)

1. **Trạm 4 trước** — hàm thuần toán, test ngay bằng CSV log có sẵn, không cần robot.
2. **Replay log qua nó, kiểm DẤU** — bài kiểm quan trọng nhất. Sai dấu là hỏng, sai lượng chỉ chậm.
3. Xong dấu mới ráp trạm 1–2–3 (camera) vào.
4. Trạm 5–6 sau cùng.

### 9.2. Lên gain theo giai đoạn (đừng bật cả 3 cùng lúc)

1. `k_α=1.5, k_ρ=0, k_β=0` → chỉ xoay tại chỗ về đích. **Xác nhận dấu `α`.**
2. Bật `k_ρ=0.5` → chạy tới đích, kệ hướng cuối. **Xác nhận tới nơi và dừng.**
3. Bật `k_β=−0.6` → quỹ đạo cong lại, tới nơi đúng hướng. **So đồ thị với bước 2.**

### 9.3. Bài kiểm dấu (bước 2) — 15 phút lời nhất

**Quy ước camera cố định trong công thức OpenCV** (`x` phải, `y` xuống, `z` trước) **nhưng KHÔNG
đảm bảo khớp xe thật** — còn tuỳ camera lắp xoay/lộn thế nào + thứ tự 4 góc nạp vào `solvePnP`.
→ coi quy ước là *giả thuyết*, **kiểm bằng tay**. Vị trí (`tvec`) khá bền; **yaw dễ lật dấu** (do
thứ tự góc) — đây là cái hay hỏng.

**Về 2 dòng lật `R[:,0]*=-1; R[:,2]*=-1`** (= quay khung tag 180° quanh trục đứng, bù thứ tự 4 góc
bị lộn): **✅ ĐÃ XÁC NHẬN ĐÚNG — GIỮ LẠI.** Test: chạy code nguyên bản (còn lật), đặt tag chính
diện → `yaw = 0`. Bỏ lật thì head-on ra 180° (2 version luôn lệch đúng 180° vì `atan2(−a,−b)=atan2(a,b)±180°`).

**Hệ quả:** `yaw` giờ là góc nghiêng sạch (0 ở chính diện) → dựng `ψ` thẳng từ nó, khỏi cần `R` gốc:
`ψ = wrap(180° ± yaw)` (head-on `yaw=0 → ψ=180°`). Log cũ (cột `yaw_deg`) **dùng lại được** cho `ψ`.

⚠️ Head-on = 0 chỉ chốt vụ **180°**, CHƯA chốt **DẤU** (chính diện là điểm đối xứng). Còn 1 test:
xoay tag chiều đã biết → dấu `yaw` + dấu `±` trong `ψ` chốt cùng lúc. `X=tvec[0]` không dính lật (`tvec` tách khỏi `R`).

Dòng `[DEBUG]` giờ in `X` (=`tvec[0]`), `Y`, `Dist(Z)`, `Yaw` — có sẵn dấu `+/−`.
Đặt tag cố định, **dịch tag sang PHẢI** (đứng *sau* camera cho khỏi loạn trái/phải; dịch thuần
ngang, đừng vừa dịch vừa xoay):
- `X` chạy về `+` hay `−`? Xoay tag rõ một góc → `Yaw` về `+` hay `−`? So `X` với `offset_x` (px)
  có cùng dấu không (ngược dấu → đúng như doc, bỏ `offset_x`).
- Theo `Left=−x`, CCW dương → `α` dương hay âm? → `ω` phải dấu gì để sửa? Khớp theo `ω=k_α·α`
  (`k_α>0`) → xong phần khó nhất. **Ghi kết quả vào comment đầu file.**

### 9.4. Sim: cần, nhưng chỉ mức tối thiểu

Bỏ Gazebo/Webots — camera ảo sạch hơn thật, tune ngon trong đó ra thực tế hỏng.
Chỉ cần **3 dòng numpy** (đúng gần tuyệt đối ở tốc độ docking vài cm/s):

```python
X += v*cos(Θ)*dt ;  Y += v*sin(Θ)*dt ;  Θ += ω*dt
```

Đặt tag cố định trong thế giới ảo, tính ngược "camera thấy gì", đẩy vào bộ điều khiển, vẽ
matplotlib. Bắt được đúng lỗi đắt nhất: sai dấu `α/β`, gain quá lớn, `k_β` nhầm dương.
Giá trị nhất: **thêm 1 dòng `+ np.random.normal(0,9)`** (nhiễu thật §7) và **1 dòng `deque`**
(trễ 100 ms) → biết trước bộ điều khiển sống hay chết. Đừng tune quá lâu — nó là lưới an toàn,
không phải phòng lab.

---

## 10. Bẫy thường gặp — checklist

| ❌ Bẫy | ✅ Cách đúng |
|---|---|
| `offset_x` tính bằng pixel | `atan2` trên toạ độ mét (pixel không nhất quán; `cx≠320`) |
| Hardcode `CENTER_X=320` | dùng `cx=349.2` qua `solvePnP` |
| Giữ lệnh `Slide LEFT/RIGHT` | bỏ hẳn — xe vi sai không đi ngang |
| Bỏ qua offset camera↔tâm quay | áp `P_body=R(φ)·P_cam+(L,d)` (mẹo tag lệch KHÔNG thay được) |
| `k_β` để dương | phải âm, nếu không xoáy ra xa |
| Bơm `yaw` thô vào `ω` | lọc EMA `a=0.9` + tag to hơn |
| `TOLERANCE_YAW=5°` khi nhiễu ±9° | tăng tag, lọc, rồi mới đặt ngưỡng |
| Recovery kiểu `"Slide LEFT"` | LÙI THẲNG (`v<0, ω=0`) |
| Magic number `offset_x >= −128.7` | điều kiện tổ hợp `ρ<2cm và \|θ_d\|<3°`, giữ N frame |
| Dừng xong bằng `\|β\|` | dùng `\|θ_d\|` — `β` vô định khi `ρ≈0` |
| Ngưỡng dừng ≠ ngưỡng xong (3 cm vs 2 cm) | một `ρ_done` chung cho cả hai |
| Cấm Astolfi khi `ρ` nhỏ (`ρ_min`) | `ρ` là khoảng tới **đích**; gần đích → xoay tại chỗ theo `θ_d` |
| Cho cả 2 xe cùng chạy | 1 xe PASSIVE khoá cứng, 1 xe ACTIVE |
| Để Astolfi chạy tới `ρ=0` | bàn giao ENGAGE ở `D=10–15 cm` |

---

## 11. Câu hỏi còn mở — mô-men vặn ren từ đâu?

Xe vi sai dock ở mũi, **không có bánh nào ở đó** (khác paper có bánh omni tại chỗ dock).
- **Servo riêng ở mũi** → ENGAGE chỉ cần `v=const, ω=0`. Đơn giản, **khuyến nghị**.
- **Xoay cả thân bằng 2 bánh** (`ω≠0` lúc ép) → ⚠️ thân quay quanh tâm trục bánh nhưng ren cần
  quay quanh trục dock ở mũi, cách nhau đoạn `a`. Khi 2 mặt dock đã chạm, `ω≠0` làm mũi vẽ cung
  bán kính `a` → sinh lực cắt ngang lên ren thay vì mô-men vặn. Cần kiểm chứng trước khi chọn.

---

## 12. Tóm tắt một trang

1. **Bỏ mọi lệnh Slide** — dock ở mũi.
2. **Đổi hệ toạ độ một lần** (`Fwd=z, Left=−x`, CCW dương) rồi quên camera.
3. **Quy đổi camera → tâm trục bánh** (`+L,d`), tag → mặt dock. Bảy hằng số đo thước.
   Mẹo tag lệch đối xứng chỉ cứu đích, KHÔNG cứu đường đi — vẫn phải dời gốc.
4. **Astolfi là phản xạ, tới thẳng TƯ THẾ đích.** `β` thay waypoint.
5. `v=k_ρρ`; `ω=k_αα+k_ββ` với `k_ρ>0`, `k_β<0`, `k_α−k_ρ>0`.
6. **Không cần odometry.** Tính lại mỗi frame. Chỉ cần đúng **dấu**.
7. **Bọc 4 chế độ** SEARCH/RECOVER/REGULATE/ENGAGE. Gần đích (`ρ<ρ_done`) → xoay tại chỗ theo
   `θ_d`; đích ở nửa sau xe (`|α|>90°`) → lùi thẳng. Xong khi `ρ<ρ_done` và `|θ_d|<θ_done`.
8. **ENGAGE tồn tại vì**: Astolfi về đích lực=0, và phải thoát vùng kỳ dị.
9. **Triển khai theo 6 trạm** (§9); code trạm 4 trước, replay log kiểm dấu, rồi ráp camera.
10. **In tag to hơn** + **vát mép dock thành phễu** — rẻ nhất, lợi nhất.
