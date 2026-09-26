import os

import cv2
import numpy as np

# Load dữ liệu calib (file nằm cạnh script, chạy từ thư mục nào cũng được)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
with np.load(os.path.join(SCRIPT_DIR, 'calibdatanew.npz')) as data:
    mtx = data['mtx']
    dist = data['dist']

print("Camera Matrix:\n", mtx)
print("Distortion Coefficients:\n", dist)

# Mở camera (sử dụng cùng camera ID như lúc calib)
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("\n📸 Hướng camera vào các đường thẳng (cạnh bàn, mép cửa) hoặc checkerboard.")
print("Nhấn 'ESC' để thoát.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không thể kết nối với camera.")
        break
    
    cv2.flip(frame, 1)

    h, w = frame.shape[:2]
    # Lấy ma trận camera tối ưu dựa trên free scaling parameter
    # alpha=1: Giữ lại tất cả các pixel của ảnh gốc, có thể có vệt đen ở viền
    # alpha=0: Cắt bỏ bớt ảnh để loại bỏ vệt đen
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # Khử méo ảnh (undistort)
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # Nếu dùng alpha = 0, bạn có thể crop ảnh lại theo ROI:
    # x, y, w_roi, h_roi = roi
    # dst = dst[y:y+h_roi, x:x+w_roi]

    # Hiển thị ảnh gốc và ảnh sau khi khử méo cạnh nhau
    # Thay đổi kích thước dst về cùng kích thước với frame nếu cần ghép ảnh (nếu đã crop)
    combined = np.hstack((frame, dst))
    
    # Vẽ text lên để dễ phân biệt
    cv2.putText(combined, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(combined, "Undistorted (Khử méo)", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Verify Calibration', combined)

    if cv2.waitKey(1) & 0xFF == 27: # Nhấn ESC để thoát
        break

cap.release()
cv2.destroyAllWindows()
