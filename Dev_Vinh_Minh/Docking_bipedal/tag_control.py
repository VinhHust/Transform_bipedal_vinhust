# đọc tag và solvePnP, biết tag cách camera bao xa, nghiêng bn, lệch tâm bn
# state machine in ra + logic tìm lại khi mất dấu


import cv2
import numpy as np
from pupil_apriltags import Detector
import csv
import os
import time

# ======== Load calibration file ========
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
data = np.load(os.path.join(SCRIPT_DIR, "calibdatanew.npz"))
camera_matrix = data["mtx"]
dist_coeffs = data["dist"]

# ======== Logging ra CSV ========
# Mỗi tag detect được = 1 dòng. Cột pose (x,y,z,yaw) chỉ có ở tag được chọn (id 0 đầu tiên).
# hamming: số bit phải sửa để ra ID này (tag thật = 0)
# decision_margin: độ tự tin của detector (tag thật gần thường > 50)
# tag_px: cạnh tag trong ảnh (px), càng nhỏ pose càng rung
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(LOG_DIR, time.strftime("tag_log_%Y%m%d_%H%M%S.csv"))
# buffering=1: ghi xuống đĩa từng dòng, để Ctrl+C giữa chừng không mất dữ liệu
log_file = open(log_path, "w", newline="", buffering=1)
log_writer = csv.writer(log_file)
log_writer.writerow(
    [
        "t",
        "frame",
        "n_tags",
        "tag_id",
        "chosen",
        "hamming",
        "decision_margin",
        "center_x",
        "center_y",
        "tag_px",
        "x",
        "y",
        "z",
        "yaw_deg",
        "offset_x",
        "control_text",
    ]
)
print(f"[LOG] Ghi vào {log_path}")
t0 = time.time()
frame_idx = 0

TAG_SIZE = 0.034  # m
half_size = TAG_SIZE / 2
object_points = np.array(
    [
        [-half_size, -half_size, 0],
        [half_size, -half_size, 0],
        [half_size, half_size, 0],
        [-half_size, half_size, 0],
    ],
    dtype=np.float32,
)

at_detector = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0)

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # nén ảnh xuống MJPG


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


CENTER_X = 320  # ảnh rộng 640, mốc giữa ảnh là 320px, mốc gốc khi tag đặt ở giữa
TOLERANCE_X = 20  # khoảng lệch vẫn coi là thẳng hàng
TOLERANCE_YAW = 5  # khoảng lệch vẫn coi là vuông góc
NEAR_DISTANCE = 0.20  # dưới 20cm được coi là tới nơi -> kết thúc

prev_cmd = "Searching..."
recovery_mode = False
recovery_direction = None

# SHOW=False khi chạy trên Pi (headless): bỏ toàn bộ chi phí vẽ + hiển thị
SHOW = True  # chạy trên pi thì đặt lại = false

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)
    frame_idx += 1
    t_now = time.time() - t0

    control_text = "Searching for tag..."
    tag_found = False
    chosen_tag = None
    pose_row = [None, None, None, None, None]  # x, y, z, yaw_deg, offset_x

    for tag in tags:
        if tag.tag_id == 0:
            tag_found = True
            chosen_tag = tag
            recovery_mode = False  # reset recovery if tag found

            corners = tag.corners.astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                object_points, corners, camera_matrix, dist_coeffs
            )

            if success:
                # KIỂM TRA LẠI PHẦN NÀY
                R, _ = cv2.Rodrigues(rvec)
                R[:, 0] *= -1
                R[:, 2] *= -1

                yaw_rad = np.arctan2(R[0, 2], R[2, 2])
                yaw_deg = (np.degrees(yaw_rad) + 180) % 360 - 180

                offset_x = tag.center[0] - CENTER_X
                x, y, z = tvec.ravel()
                pose_row = [x, y, z, yaw_deg, offset_x]
                print(
                    f"[DEBUG] X: {x:+.3f} m | Y: {y:+.3f} m | Dist(Z): {z:.3f} m | Yaw: {yaw_deg:+.2f}°"
                    f" | offset_x: {offset_x:+.1f}px | ham: {tag.hamming} | margin: {tag.decision_margin:.1f} | n_tags: {len(tags)}"
                )

                if z < NEAR_DISTANCE:
                    if offset_x >= -128.7:
                        control_text = "Slide LEFT"
                    else:
                        control_text = "Docking complete - STOP"
                elif abs(yaw_deg) > TOLERANCE_YAW:
                    control_text = "Turn LEFT" if yaw_deg > 0 else "Turn RIGHT"
                elif abs(offset_x) > TOLERANCE_X:
                    control_text = "Slide LEFT" if offset_x > 0 else "Slide RIGHT"
                else:
                    control_text = "Move FORWARD"

                # Vẽ tag
                for i in range(4):
                    pt1 = tuple(corners[i].astype(int))
                    pt2 = tuple(corners[(i + 1) % 4].astype(int))
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                center = tuple(tag.center.astype(int))
                cv2.putText(
                    frame,
                    f"ID: {tag.tag_id}",
                    center,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
            else:
                control_text = "solvePnP failed"
            break

    # Nếu không thấy tag
    if not tag_found:
        if not recovery_mode:
            if "Turn RIGHT" in prev_cmd:
                recovery_direction = "Slide LEFT"
            elif "Turn LEFT" in prev_cmd:
                recovery_direction = "Slide RIGHT"
            else:
                recovery_direction = "Searching..."

            recovery_mode = True  # bật chế độ khôi phục

        # Nếu đang khôi phục, giữ lệnh liên tục
        control_text = f"{recovery_direction} to recover tag"

    prev_cmd = control_text

    # Ghi log: 1 dòng cho mỗi tag detect được trong frame này.
    # Nếu không có tag nào vẫn ghi 1 dòng để biết frame đó bị mất dấu.
    if tags:
        for tag in tags:
            # cạnh tag trong ảnh ≈ trung bình 4 cạnh của hình vuông
            c = tag.corners
            tag_px = np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)])
            is_chosen = tag is chosen_tag
            log_writer.writerow(
                [
                    f"{t_now:.3f}",
                    frame_idx,
                    len(tags),
                    tag.tag_id,
                    int(is_chosen),
                    tag.hamming,
                    f"{tag.decision_margin:.1f}",
                    f"{tag.center[0]:.1f}",
                    f"{tag.center[1]:.1f}",
                    f"{tag_px:.1f}",
                    *(
                        [f"{v:.4f}" for v in pose_row]
                        if is_chosen and pose_row[0] is not None
                        else [""] * 5
                    ),
                    control_text if is_chosen else "",
                ]
            )
    else:
        log_writer.writerow([f"{t_now:.3f}", frame_idx, 0] + [""] * 12 + [control_text])

    if SHOW:
        cv2.putText(
            frame, control_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2
        )
        cv2.imshow("AprilTag Docking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
log_file.close()
print(f"[LOG] Đã lưu {frame_idx} frame vào {log_path}")
cv2.destroyAllWindows()
