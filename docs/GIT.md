# GIT.md — Quy trình làm việc chung Minh ↔ Vinh

Tài liệu này dành cho **Minh (Cat2I)** và **Vinh (VinhHust)** khi cùng dev trên fork của Vinh,
rồi gộp thành quả vào repo chính của **Nam (khacnambn)**.

> **Đọc gì:** Phần B ngay bên dưới là thứ dùng hằng ngày — mở file là thấy luôn.
> Các phần còn lại chỉ đọc khi cần: **C** lúc sắp làm gì nguy hiểm, **0** lúc quên
> repo/remote nào là cái nào, **A** chỉ làm một lần lúc cài máy (đã xong).
>
> Thứ tự chữ cái A/B/C giữ nguyên như cũ để các tham chiếu trong bài (mục B4, mục C0…)
> không bị lệch — nên trên file, B nằm trước A. Không phải lỗi.

---

## PHẦN B — Daily workflow

### B1. Vòng lặp hằng ngày (làm mỗi ngày, cả hai người)

```bash
# ===== ĐẦU BUỔI — luôn luôn, không ngoại lệ =====
git switch dev_team
git pull --rebase

# ===== ... code ... =====

# ===== SAU MỖI VIỆC NHỎ ĐÃ XONG =====
git add -A
git commit -m "feat(imu): mô tả ngắn việc vừa làm"
git push
```

**`--rebase` nghĩa là gì?** Nó lấy commit mới của người kia về, rồi *đặt commit của bạn lên trên cùng*,
thay vì tạo một commit merge rác. Kết quả: lịch sử là một đường thẳng, dễ đọc, dễ tìm lỗi.

**Tần suất:** commit nhỏ, push thường xuyên. Ôm code 3 ngày rồi push một cục 40 file
là công thức chuẩn để tạo conflict địa ngục.

---

### B2. Khi `git push` bị từ chối

Thông báo trông như thế này:

```
! [rejected]  dev_team -> dev_team (fetch first)
error: failed to push some refs ... behind its remote counterpart
```

Nghĩa là: người kia vừa push trước bạn. Hoàn toàn bình thường. **Đừng dùng `--force`.** Chỉ cần:

```bash
git pull --rebase
git push
```

---

### B3. Khi gặp conflict lúc `git pull --rebase`

Git sẽ dừng lại và báo file nào bị đụng. Trong file đó sẽ có các dấu:

```
<<<<<<< HEAD
   code của người kia (đã có trên server)
=======
   code của bạn
>>>>>>> abc1234 (commit message của bạn)
```

Cách xử lý:

```bash
# 1. Mở từng file bị báo conflict, sửa tay:
#    xoá 3 dòng dấu <<<<<<< ======= >>>>>>>
#    giữ lại đoạn code ĐÚNG (có thể là của bạn, của người kia, hoặc trộn cả hai)

# 2. Đánh dấu đã xử lý xong
git add <file-vừa-sửa>

# 3. Tiếp tục rebase
git rebase --continue

# 4. Push
git push
```

**Nếu rối quá, muốn quay về lúc chưa pull:**

```bash
git rebase --abort
```

Lệnh này an toàn tuyệt đối, đưa mọi thứ về nguyên trạng. Rồi hỏi người kia xem đoạn đó nên giữ cái gì.

---

### B4. Định kỳ kéo `main` của Nam về (2–3 ngày/lần, hoặc khi Nam vừa merge gì đó)

Để tránh nhánh `dev_team` trôi quá xa khỏi repo chính — càng trôi xa, lúc gộp cuối càng đau.

Hai người gõ hai lệnh khác nhau vì **tên remote trên hai máy đặt khác nhau** — máy Minh gọi repo
Nam là `origin`, máy Vinh gọi là `upstream`. Quên cái nào là cái nào thì xem mục 0.

**Minh:**
```bash
git switch dev_team
git fetch origin
git merge origin/main
git push
```

**Vinh:**
```bash
git switch dev_team
git fetch upstream
git merge upstream/main
git push
```

> Ở bước này dùng **`merge`**, KHÔNG dùng `rebase`.
> Lý do: `dev_team` là nhánh chung, người kia cũng đang dùng.
> `rebase` viết lại lịch sử commit **đã push**, làm hỏng repo của người kia.
> `merge` chỉ *thêm* commit mới → an toàn tuyệt đối. Xem thêm mục C1.

Chỉ **một người** làm bước này rồi báo người kia `git pull --rebase` là đủ — không cần cả hai cùng làm.

---

### B5. Khi cả hai thống nhất là xong → gộp lên repo Nam

Chạy B4 một lần cuối trước đã, để PR không bị báo conflict.

**Tầng 1 — fork → nhánh tích hợp trên repo Nam:**

```bash
gh pr create --repo khacnambn/TransformBipedal_Firmware \
  --base dev_team --head VinhHust:dev_team \
  --title "Dev: <tóm tắt thành quả>" \
  --body "Tổng hợp công việc của Minh và Vinh: ..."
```

**Tầng 2 — nhánh tích hợp → `main`, Nam review.** Ở tầng này **tách làm hai PR**:

| PR | Nội dung | Nam review |
|---|---|---|
| A | `Dev_Vinh_Minh/`, `docs/`, file test riêng | Duyệt nhanh, gần như không rủi ro |
| B | `bipedal_nam/**`, `examples_client/**` | Đọc kỹ — đây là code của Nam |

PR #2 hỏng chính vì gộp cả hai làm một cục. Tách ra thì Nam duyệt A trong 2 phút,
còn B thì soi kỹ. Đừng bao giờ để một PR vừa thêm code mới vừa sửa code người khác.

---

## 0. Bối cảnh — ai là ai, repo nào là repo nào

Có **hai repo khác nhau** trên GitHub, dù nội dung gần giống nhau:

| Tên gọi trong tài liệu | URL | Chủ sở hữu | Vai trò |
|---|---|---|---|
| **repo chính** | `github.com/khacnambn/TransformBipedal_Firmware` | Nam | Nguồn chân lý. Code cuối cùng nằm ở đây. |
| **fork** | `github.com/VinhHust/Transform_bipedal_vinhust` | Vinh | Sân chơi của Minh + Vinh. Code nháp, thử nghiệm nằm ở đây. |

> Hai cái tên dễ nhầm:
> - Repo chính từng tên là `Transform_bipedal`, Nam đã đổi thành `TransformBipedal_Firmware`.
>   GitHub giữ tên cũ làm redirect nên **cả hai URL đều chạy** — cùng một repo, không phải hai.
> - Fork của Vinh **không cùng tên** với repo gốc. GitHub cho phép đổi tên fork, nên đừng đoán
>   URL fork từ tên repo gốc. Muốn tìm: `gh repo list <tên-người> --fork`

**Fork không phải là một nhánh.** Nó là một repo độc lập hoàn toàn, chỉ tình cờ chung lịch sử commit
với repo chính. Đây là điểm hay gây nhầm lẫn nhất.

Trên máy mỗi người, **`remote` chỉ là một cái tên rút gọn cho một URL**. Bạn có thể có nhiều remote cùng lúc
trong một thư mục code. Quy ước tên remote trong tài liệu này:

**Máy của Minh:**
- `origin` → repo chính của Nam (chỉ để *kéo về*, không push nhánh chung lên đây)
- `vinh` → fork của Vinh (nơi *push* code hằng ngày)

**Máy của Vinh:**
- `origin` → fork của chính Vinh (nơi *push* code hằng ngày)
- `upstream` → repo chính của Nam (chỉ để *kéo về*)

**Nhánh chung:** `dev_team` — nằm trên **fork**, cả hai cùng push/pull vào đây.

### Luồng 3 tầng

```
VinhHust/...:dev_team     ← Minh + Vinh push hằng ngày.
        │                   Thoải mái, hỏng cũng không ảnh hưởng ai.
        │  PR khi xong một mảng việc
        ▼
khacnambn:dev_team        ← nhánh tích hợp trên repo Nam.
        │                   Nam thấy được tiến độ, nhưng main vẫn an toàn.
        │  PR khi hoàn thiện — Nam review
        ▼
khacnambn:main            ← chỉ code đã duyệt
```

Tầng giữa là thứ đáng giá nhất: Nam nhìn được hai bạn đang làm gì mà `main` không chịu
rủi ro nào. Về kỹ thuật thì tầng này không bắt buộc, nhưng nó rẻ và tránh được đúng
kiểu tai nạn đã xảy ra ở PR #2 (merge thẳng vào `main`, phải revert).

### Tại sao `dev_team` mới, không dùng lại `dev_vinh`

Không phải vì code trong `dev_vinh` hỏng — code nó sạch và chạy được. Lý do nằm ở
**điểm rẽ nhánh**:

```
                 điểm rẽ khỏi main    main đã đi thêm    merge vào main
vinh/dev_vinh    1b8e55c          →   3 commit       →   20 CONFLICT
dev_team         74de25e          →   0 commit       →   0 conflict (fast-forward)
```

`dev_vinh` rẽ ra từ lâu, và sau đó `main` đi thêm 3 bước — trong đó có đúng cái revert
xoá sạch thư mục của Vinh. Hai bên cùng đụng một chỗ → Git phải hỏi từng file.

`dev_team` rẽ ra ngay tại đỉnh `main` hiện tại, nên `main` đứng yên hoàn toàn kể từ đó.
Không có gì để mâu thuẫn. Xem thêm mục C0.

---

## PHẦN A — Setup một lần duy nhất

> **Trạng thái tính đến 26/08/2026:** Minh đã xong toàn bộ Phần A — nhánh `dev_team`
> đã tạo và đẩy lên fork. Vinh chỉ còn phải làm **A2**.

### A0. Vinh cấp quyền cho Minh — *Vinh làm, Minh không tự làm được* ✅ xong

Không có bước này thì Minh chỉ **đọc** được fork, **không push** được.

Vinh vào `github.com/VinhHust/Transform_bipedal_vinhust` → tab **Settings** → mục
**Collaborators** → **Add people** → nhập username GitHub của Minh (`cat2I`).

Minh nhận lời mời qua email, hoặc vào thẳng
`github.com/VinhHust/Transform_bipedal_vinhust/invitations` bấm **Accept**.

Kiểm tra đã có quyền chưa:

```bash
gh api repos/VinhHust/Transform_bipedal_vinhust --jq .permissions.push
# true = push được
```

---

### A1. Minh — thiết lập máy ✅ xong

```bash
cd ~/Documents/projects/Transformer/Transform_bipedal

# 0. Đăng nhập GitHub cho git — KHÔNG CÓ BƯỚC NÀY THÌ KHÔNG PUSH ĐƯỢC
gh auth login
#   GitHub.com -> HTTPS -> Y (authenticate Git) -> Login with a web browser
#   Bước "authenticate Git with your GitHub credentials" mới là bước quan trọng:
#   nó cài credential helper cho git. Bỏ qua thì pull được nhưng push luôn hỏng.

# 1. Cập nhật main mới nhất từ repo Nam
git switch main
git pull origin main
#   LƯU Ý: git pull đổ code vào nhánh ĐANG ĐỨNG, không phải nhánh ghi trong lệnh.
#   Chữ "main" ở đây chỉ có nghĩa "lấy nhánh main TỪ SERVER".
#   Đứng ở dev_team mà chạy lệnh này là merge main vào dev_team — sai ý.

# 2. Thêm remote trỏ tới fork của Vinh
git remote add vinh https://github.com/VinhHust/Transform_bipedal_vinhust.git
git fetch vinh

# 3. Kiểm tra — phải thấy CẢ HAI remote
git remote -v
#   origin  .../khacnambn/Transform_bipedal.git           (fetch/push)
#   vinh    .../VinhHust/Transform_bipedal_vinhust.git    (fetch/push)

# 4. Tạo nhánh chung, xuất phát từ main sạch của Nam
git switch -c dev_team origin/main

# 5. Đẩy nhánh chung lên fork của Vinh, và ghi nhớ "nhánh này thuộc về remote vinh"
git push -u vinh dev_team
```

Cờ `-u` (viết tắt của `--set-upstream`) là thứ giúp sau này chỉ cần gõ `git push` / `git pull`
trống không, Git tự hiểu là đẩy lên `vinh/dev_team`.

**Xong bước 5, báo Vinh làm A2.**

---

### A2. Vinh — thiết lập máy ⬜ **CHƯA LÀM**

```bash
cd <thư-mục-code-của-Vinh>

# 1. Thêm remote trỏ tới repo chính của Nam (nếu chưa có)
git remote add upstream https://github.com/khacnambn/TransformBipedal_Firmware.git

# 2. Kiểm tra
git remote -v
#   origin    .../VinhHust/Transform_bipedal_vinhust.git      <- fork của mình
#   upstream  .../khacnambn/TransformBipedal_Firmware.git     <- repo Nam

# 3. Lấy nhánh chung mà Minh đã tạo
git fetch origin
git switch -c dev_team --track origin/dev_team
```

Sau đó **đóng PR #5** (bấm Close, KHÔNG merge). Hai commit `fixUI` và `update walking gait`
của nó đã nằm sẵn trong `dev_team` rồi. Để PR mở chỉ gây nhầm lẫn.

Từ đây trở đi cả hai chỉ dùng `dev_team`. Bỏ hẳn `dev_vinh`.

---

### A3. Cả hai — thống nhất phân chia công việc *trước khi gõ dòng code đầu tiên*

Git merge tự động **rất tốt** khi hai người sửa hai file khác nhau,
và **rất tệ** khi hai người sửa cùng một hàm.

Ngồi lại 5 phút, chốt: Minh ôm module nào, Vinh ôm module nào. Ghi ra đây:

```
Minh:  ...
Vinh:  ...
Chung (phải báo nhau trước khi sửa):  ...
```

5 phút này tiết kiệm được vài giờ gỡ conflict.

---

## PHẦN C — CẤM KỊ (làm là hỏng, có cái không cứu được)

### C0. ❌ KHÔNG gộp một nhánh cũ thẳng vào `main`

Đây là cái bẫy đã cắn nhóm một lần rồi (PR #2 → phải revert ở PR #4), và là cái khó
nhìn ra nhất, nên để đầu danh sách.

**Git khi merge KHÔNG so sánh nội dung hai nhánh.** Nó tìm **điểm rẽ nhánh chung**
(merge base), rồi hỏi: *"từ điểm đó tới giờ, mỗi bên đã làm gì?"*
Conflict xảy ra khi **cả hai bên cùng đụng vào một chỗ**.

Nên một nhánh có code hoàn hảo vẫn có thể merge ra thảm hoạ, nếu nó rẽ ra quá lâu rồi:

```
                 điểm rẽ khỏi main    main đã đi thêm    kết quả merge
vinh/dev_vinh    1b8e55c          →   3 commit       →   20 CONFLICT
dev_team         74de25e          →   0 commit       →   0 conflict
```

Cách tự kiểm tra **trước khi** mở PR — chạy thử mà không đụng gì tới code:

```bash
git fetch origin
git merge-tree --write-tree --name-only origin/main HEAD >/dev/null && \
  echo "SẠCH, mở PR được" || echo "SẼ CÓ CONFLICT, chạy B4 trước đã"
```

**Cách phòng:** chạy B4 đều đặn (2–3 ngày/lần). Nguyên tắc chung —
**conflict tỉ lệ thuận với thời gian hai nhánh không nói chuyện với nhau.**

> Trường hợp đặc biệt nguy hiểm: nếu công việc của bạn **từng bị revert** trên `main`,
> Git coi các commit đó là "đã merge rồi" và sẽ không mang nội dung về nữa.
> Phải tạo một commit đảo ngược cái revert: `git revert --no-edit <hash-cua-commit-revert>`
> — làm trên `dev_team`, KHÔNG làm trên `main`.

---

### C1. ❌ KHÔNG BAO GIỜ `git push --force` lên `dev_team`

```bash
git push --force        # ← CẤM
git push -f             # ← CẤM (viết tắt của cái trên)
```

**Hậu quả:** xoá sạch commit của người kia khỏi server. Người kia mất code, và thường không biết
là mình vừa mất cho tới vài giờ sau.

**Nếu bắt buộc phải force** (hiếm, ví dụ lỡ commit file secret): dùng `--force-with-lease`
(an toàn hơn, nó từ chối nếu người kia vừa push) và **báo người kia trước qua chat**.

---

### C2. ❌ KHÔNG `git rebase` commit đã push lên `dev_team`

```bash
git rebase -i HEAD~5    # ← CẤM nếu 5 commit đó đã push rồi
git rebase origin/main  # ← CẤM trên nhánh chung, dùng git merge (mục B4)
```

**Tại sao:** rebase *tạo commit mới thay thế commit cũ* (đổi hash). Commit cũ mà người kia đã pull về
sẽ thành mồ côi. Lần pull tiếp theo của họ sẽ ra một mớ commit trùng lặp không gỡ nổi.

**Ngoại lệ được phép:** `git pull --rebase` ở mục B1 là an toàn — nó chỉ rebase các commit
**local, chưa push** của bạn. Đó chính là mục đích thiết kế của nó.

---

### C3. ❌ KHÔNG push nhánh `dev_team` lên `origin` (repo của Nam)

```bash
git push origin dev_team    # ← CẤM (với máy của Minh)
```

Điểm hẹn là **fork**. Push lên repo chính sẽ tạo ra hai nhánh cùng tên ở hai nơi,
và không ai biết nhánh nào mới hơn. Với máy Minh, `dev_team` chỉ push lên `vinh`.

---

### C4. ❌ KHÔNG `git reset --hard` khi chưa hiểu mình đang làm gì

```bash
git reset --hard HEAD~3   # ← xoá vĩnh viễn 3 commit + toàn bộ thay đổi chưa commit
```

Không hỏi lại, không có thùng rác. Nếu chỉ muốn bỏ thay đổi ở **một file**:

```bash
git restore <tên-file>
```

Nếu muốn cất tạm mọi thứ để làm việc khác, rồi lấy lại sau:

```bash
git stash          # cất đi
git stash pop      # lấy lại
```

---

### C5. ❌ KHÔNG commit thẳng lên `main`

`main` chỉ được cập nhật thông qua PR mà Nam merge. Trước mỗi lần commit, kiểm tra:

```bash
git branch --show-current    # phải in ra: dev_team
```

Nếu lỡ commit nhầm lên `main` và **chưa push**:

```bash
git switch -c dev_team_tam   # chuyển commit đó sang nhánh mới
git switch main
git reset --hard origin/main # trả main về đúng trạng thái server
```

---

### C6. ❌ KHÔNG ôm code nhiều ngày không push

Không phải lệnh cấm kỹ thuật, nhưng là nguyên nhân số 1 gây conflict lớn.
Commit nhỏ (một việc = một commit), push trong ngày.

---

### C7. ❌ KHÔNG commit file rác

Kiểm tra `.gitignore` trước. File log, `__pycache__/`, dataset nặng, file `.env` chứa key —
không đưa lên repo. Trước khi commit, luôn nhìn qua:

```bash
git status
```

---

## Bảng tra cứu nhanh

| Muốn làm gì | Lệnh |
|---|---|
| Đang ở nhánh nào? | `git branch --show-current` |
| Có gì thay đổi? | `git status` |
| Xem mình sửa gì | `git diff` |
| Lấy code mới của người kia | `git pull --rebase` |
| Đẩy code lên | `git add -A && git commit -m "..." && git push` |
| Bỏ thay đổi 1 file | `git restore <file>` |
| Cất tạm để làm việc khác | `git stash` → `git stash pop` |
| Đang rebase mà rối, muốn thoát | `git rebase --abort` |
| Xem lịch sử gọn | `git log --oneline --graph -20` |
| Đồng bộ với main của Nam | xem mục B4 |
| Kiểm tra trước khi mở PR | xem mục C0 |
| Xem nhánh mình rẽ ra từ đâu | `git merge-base origin/main HEAD` |

---

## Nếu vẫn hỏng

Gần như mọi thứ trong Git đều cứu được, miễn là **chưa `push --force`**.
Trước khi thử lệnh lạ, chụp lại màn hình `git status` và `git log --oneline -10`,
rồi hỏi Nam hoặc hỏi lại — đừng gõ lệnh ngẫu nhiên trên StackOverflow khi đang hoảng.

Lệnh cứu hộ hay dùng nhất — xem lại **mọi** trạng thái HEAD từng đi qua (kể cả commit tưởng đã mất):

```bash
git reflog
```
