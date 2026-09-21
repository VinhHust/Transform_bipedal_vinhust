import cv2
import numpy as np
from pupil_apriltags import Detector
import matplotlib.pyplot as plt

# ======== Load calibration file ========
data = np.load(
    "/home/nam/Lekiwi_ws/src/lerobot/lerobot/common/utils/arm1_calib_data.npz"
)
camera_matrix = data["mtx"]
dist_coeffs = data["dist"]

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

cap = cv2.VideoCapture(0)
CENTER_X = 320
TOLERANCE_X = 20
TOLERANCE_YAW = 5
NEAR_DISTANCE = 0.20

prev_cmd = "Searching..."
recovery_mode = False
recovery_direction = None

plt.ion()
fig, ax = plt.subplots()
im = ax.imshow([[0]])
plt.title("AprilTag Docking Simulation")
plt.axis("off")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)

    control_text = "Searching for tag..."
    tag_found = False

    for tag in tags:
        if tag.tag_id == 0:
            tag_found = True
            recovery_mode = False  # reset recovery if tag found

            corners = tag.corners.astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                object_points, corners, camera_matrix, dist_coeffs
            )

            if success:
                R, _ = cv2.Rodrigues(rvec)
                R[:, 0] *= -1
                R[:, 2] *= -1

                yaw_rad = np.arctan2(R[0, 2], R[2, 2])
                yaw_deg = (np.degrees(yaw_rad) + 180) % 360 - 180

                offset_x = tag.center[0] - CENTER_X
                x, y, z = tvec.ravel()
                print(
                    f"[DEBUG] Distance: {z:.3f} m | Yaw: {yaw_deg:.2f}° | Offset X: {offset_x:.1f}px"
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

    # Show control text
    cv2.putText(
        frame, control_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2
    )

    im.set_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    fig.canvas.draw()
    fig.canvas.flush_events()

cap.release()
cv2.destroyAllWindows()
