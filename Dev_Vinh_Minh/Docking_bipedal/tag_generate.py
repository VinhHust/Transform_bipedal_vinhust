import cv2
from pupil_apriltags import Detector
import matplotlib.pyplot as plt

# Tạo detector cho tag36h11
at_detector = Detector(families='tag36h11')

# Mở webcam
cap = cv2.VideoCapture(0)

# Khởi tạo cửa sổ matplotlib
plt.ion()  # chế độ tương tác (interactive)
fig, ax = plt.subplots()

im = ax.imshow([[0]], cmap='gray')
plt.title("AprilTag Detection")
plt.axis('off')

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags = at_detector.detect(gray)

    for tag in tags:
        if tag.tag_id == 0:
            corners = tag.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.putText(frame, f"ID: {tag.tag_id}", center,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Cập nhật ảnh trong matplotlib
    im.set_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    fig.canvas.draw()
    fig.canvas.flush_events()

cap.release()
plt.ioff()
plt.close()
