#!/usr/bin/env python3
"""
Script CHUYÊN ĐO độ trễ IMU (RTT / age / skew giữa 2 chân).

Kế thừa từ test_imu_fusion_simple.py:
  - IMUController : socket REQ + gửi {"type": "feedback"}
  - Tee           : ghi song song terminal + file log

Đo 3 đại lượng (tự xuống cấp nếu chưa đủ điều kiện):
  1) RTT  (round-trip, ms) : laptop hỏi -> chờ Pi trả lời. CHỈ dùng đồng hồ laptop
     (time.perf_counter) => KHÔNG cần chrony, KHÔNG cần sửa Pi. Chạy được NGAY.
  2) age  (tuổi mẫu, ms)   : now(laptop) - t_sample(Pi). CẦN Pi đã đóng dấu
     't_sample' + chrony đã đồng bộ 3 máy.
  3) skew (lệch 2 IMU, ms) : t_sample_LEFT - t_sample_RIGHT. Cùng điều kiện như age.

Cách chạy (trên laptop, khi 2 leg server đang chạy):
    python measure_imu_latency.py
    python measure_imu_latency.py --count 1000 --hz 50
    python measure_imu_latency.py --left mobile2.local:5556 --right mobile1.local:5555

LƯU Ý: age/skew chỉ CÓ NGHĨA khi chrony đã đồng bộ. Kiểm tra trước bằng:
    chronyc tracking        # trên mỗi Pi, offset phải dưới vài ms
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import zmq

# ── Kế thừa từ test_imu_fusion_simple.py (cùng thư mục) ───────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_imu_fusion_simple import IMUController, Tee  # noqa: E402

LOG_DIR = Path(__file__).resolve().parent.parent.parent / "logs"


def setup_logging():
    """Mở .log (đọc) + .jsonl (phân tích) trong logs/. Trả về file handle jsonl."""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"imu_latency_{stamp}.log"
    jsonl_path = LOG_DIR / f"imu_latency_{stamp}.jsonl"

    log_file = open(log_path, "w", encoding="utf-8")
    sys.stdout = Tee(sys.stdout, log_file)
    jsonl_file = open(jsonl_path, "w", encoding="utf-8")

    print(f" Log (text):  {log_path}")
    print(f" Log (jsonl): {jsonl_path}")
    return jsonl_file


class MeasureController(IMUController):
    """Mở rộng IMUController: đọc NGUYÊN gói response + bấm giờ RTT."""

    def read_full(self):
        """
        Trả về (response_dict, rtt_ms).
        response_dict = None nếu timeout/lỗi (socket được reset trong parent.connect()).
        """
        t0 = time.perf_counter()
        try:
            self.socket.send_json({"type": "feedback"})
            resp = self.socket.recv_json()
        except zmq.Again:
            self.connect()  # reset socket REQ bị kẹt
            return None, None
        except zmq.error.ZMQError:
            self.connect()
            return None, None
        rtt_ms = (time.perf_counter() - t0) * 1000.0
        return resp, rtt_ms


def get_t_sample(resp):
    """Lấy t_sample hợp lệ (> 0) từ response, hoặc None nếu thiếu/0."""
    if not resp:
        return None
    t = resp.get("t_sample", 0.0)
    return t if (isinstance(t, (int, float)) and t > 0) else None


def summarize(name, arr, unit="ms"):
    """In mean/median/p95/min/max/jitter cho 1 mảng số."""
    if not arr:
        print(f"  {name:<22}: (không có dữ liệu)")
        return None
    a = sorted(arr)
    n = len(a)
    p95 = a[min(n - 1, int(0.95 * n))]
    s = {
        "n": n,
        "mean": statistics.mean(a),
        "median": statistics.median(a),
        "p95": p95,
        "min": a[0],
        "max": a[-1],
        "jitter": statistics.pstdev(a) if n > 1 else 0.0,
    }
    print(
        f"  {name:<22}: mean={s['mean']:7.2f} | median={s['median']:7.2f} | "
        f"p95={s['p95']:7.2f} | max={s['max']:7.2f} | jitter={s['jitter']:6.2f} {unit}"
    )
    return s


def run_measurement(left, right, count, hz, jsonl_file):
    period = 1.0 / hz if hz > 0 else 0.0
    print("\n" + "=" * 78)
    print(f"ĐO ĐỘ TRỄ IMU  |  count={count}  hz={hz}")
    print("  RTT: đo được ngay (không cần chrony).")
    print("  age/skew: chỉ hiện khi Pi đã đóng dấu t_sample + chrony đã đồng bộ.")
    print("=" * 78)

    print("\n⏳ Warm-up server 3s...")
    time.sleep(3)

    rtt_L, rtt_R = [], []
    age_L, age_R = [], []
    skew = []
    ts_missing = 0
    fail = 0

    t_next = time.perf_counter()
    try:
        for i in range(count):
            respL, rL = left.read_full()
            respR, rR = right.read_full()
            now = time.time()  # đồng hồ tường (đã đồng bộ) - dùng cho age

            if respL is None or respR is None:
                fail += 1
            else:
                if rL is not None:
                    rtt_L.append(rL)
                if rR is not None:
                    rtt_R.append(rR)

                tL = get_t_sample(respL)
                tR = get_t_sample(respR)
                if tL is not None and tR is not None:
                    aL = (now - tL) * 1000.0
                    aR = (now - tR) * 1000.0
                    sk = (tL - tR) * 1000.0
                    age_L.append(aL)
                    age_R.append(aR)
                    skew.append(sk)
                    if jsonl_file:
                        jsonl_file.write(json.dumps({
                            "iter": i, "t": round(now, 4),
                            "rtt_L": rL, "rtt_R": rR,
                            "age_L": aL, "age_R": aR, "skew": sk,
                        }) + "\n")
                        jsonl_file.flush()
                else:
                    ts_missing += 1

            # In tiến độ mỗi 50 vòng
            if (i + 1) % 50 == 0:
                msg = f"  [{i+1:4d}/{count}] RTT_L={rL if rL else 0:.1f} RTT_R={rR if rR else 0:.1f} ms"
                if skew:
                    msg += f" | skew={skew[-1]:+.1f} age_L={age_L[-1]:.1f} ms"
                print(msg)

            # Pacing theo hz
            t_next += period
            dt = t_next - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                t_next = time.perf_counter()

    except KeyboardInterrupt:
        print("\n👋 Dừng sớm bởi người dùng.")

    # ── Tổng kết ──────────────────────────────────────────────────────────────
    print("\n" + "=" * 78)
    print(f"KẾT QUẢ  (fail={fail}, t_sample thiếu={ts_missing})")
    print("=" * 78)
    print("\n[1] RTT khứ hồi (không cần chrony):")
    sL = summarize("RTT LEFT", rtt_L)
    sR = summarize("RTT RIGHT", rtt_R)
    if sL and sR:
        print(f"  => một chiều ~ RTT/2: LEFT≈{sL['mean']/2:.1f}ms  RIGHT≈{sR['mean']/2:.1f}ms")

    if skew:
        print("\n[2] AGE - tuổi mẫu (cần chrony đồng bộ):")
        summarize("age LEFT", age_L)
        summarize("age RIGHT", age_R)
        print("\n[3] SKEW - lệch 2 IMU |t_L - t_R| (cần chrony đồng bộ):")
        abs_skew = [abs(x) for x in skew]
        summarize("skew (signed)", skew)
        summarize("skew (abs)", abs_skew)
        print("\nĐọc số:")
        print("  - |skew| ~ offset chrony (vài ms) => 2 IMU coi như đồng pha, TỐT.")
        print("  - |skew| lớn & ỔN ĐỊNH  => lệch hệ thống, bù được bằng nội suy.")
        print("  - |skew| lớn & GIẬT LOẠN => bằng chứng nên chuyển sang MCU.")
        print("  - jitter (RTT & skew) QUAN TRỌNG hơn giá trị trung bình.")
    else:
        print("\n⚠️  Chưa đo được age/skew: response KHÔNG có 't_sample' hợp lệ.")
        print("    => Cần làm Bước 1-2 (đóng dấu t_sample trên Pi) trước.")
        print("    Hiện tại chỉ có số RTT ở trên (vẫn dùng để đánh giá đường truyền).")


def parse_endpoint(s):
    host, port = s.rsplit(":", 1)
    return host, int(port)


def main():
    ap = argparse.ArgumentParser(description="Đo độ trễ IMU (RTT/age/skew)")
    ap.add_argument("--left", default="mobile2.local:5556", help="host:port chân TRÁI")
    ap.add_argument("--right", default="mobile1.local:5555", help="host:port chân PHẢI")
    ap.add_argument("--count", type=int, default=500, help="số vòng đo")
    ap.add_argument("--hz", type=float, default=50.0, help="tần số đo (mô phỏng nhịp policy)")
    args = ap.parse_args()

    lh, lp = parse_endpoint(args.left)
    rh, rp = parse_endpoint(args.right)

    jsonl_file = setup_logging()
    try:
        left = MeasureController(lh, lp, "LEFT")
        right = MeasureController(rh, rp, "RIGHT")
        run_measurement(left, right, args.count, args.hz, jsonl_file)
    except Exception as e:
        print(f"❌ Lỗi khởi tạo: {e}")
        print("\n📋 Đảm bảo 2 leg server đang chạy trên Pi (cổng 5556 trái / 5555 phải).")
    finally:
        jsonl_file.close()


if __name__ == "__main__":
    main()
