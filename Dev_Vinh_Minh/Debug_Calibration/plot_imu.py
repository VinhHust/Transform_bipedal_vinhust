#!/usr/bin/env python3
# Vẽ roll / pitch / yaw theo thời gian từ file CSV log của gyroacce.
# Chỉ vẽ 4 thứ: t (thời gian), roll, pitch, yaw. Không thêm gì khác.
#
# Dùng:
#   python3 plot_imu.py                 # vẽ tất cả file trong logs/
#   python3 plot_imu.py logs/45deg.csv  # vẽ đúng 1 (hoặc nhiều) file chỉ định

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    """Đọc cột t, roll, pitch, yaw từ 1 file CSV. Trả về 4 list số."""
    t, roll, pitch, yaw = [], [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            roll.append(float(row["roll"]))
            pitch.append(float(row["pitch"]))
            yaw.append(float(row["yaw"]))
    return t, roll, pitch, yaw


def plot_one(path):
    t, roll, pitch, yaw = read_csv(path)

    plt.figure(figsize=(10, 5))
    plt.plot(t, roll, label="roll")
    plt.plot(t, pitch, label="pitch")
    plt.plot(t, yaw, label="yaw")

    plt.xlabel("Thời gian (s)")
    plt.ylabel("Góc (độ)")
    plt.title(Path(path).stem)
    plt.legend()

    out = Path(path).with_suffix(".png")
    plt.savefig(out, dpi=120, bbox_inches="tight")
    print(f"✓ Đã lưu {out}")
    plt.close()


if __name__ == "__main__":
    # Lấy danh sách file: từ dòng lệnh, hoặc mặc định tất cả CSV trong logs/
    args = sys.argv[1:]
    if args:
        files = [Path(p) for p in args]
    else:
        files = sorted((Path(__file__).resolve().parent / "logs").glob("*.csv"))

    if not files:
        print("Không tìm thấy file CSV nào.")
        sys.exit(1)

    for f in files:
        plot_one(f)