import cv2
import numpy as np

# 📐 Cấu hình checkerboard (ô bên trong)
CHECKERBOARD = (8, 5)  # tức là cần in 9x6 ô vuông
SQUARE_SIZE = 0.019  # mét (19mm)

# Tạo các điểm 3D thực tế
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # Điểm 3D
imgpoints = []  # Điểm 2D

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("📸 Nhấn phím 'c' để chụp khi phát hiện checkerboard. Nhấn ESC để thoát.")
print("🔎 Lưu ý: Phải thấy rõ toàn bộ bảng caro 10x7 ô vuông!")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    print("found =", found)  # debug console

    if found:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)
        cv2.putText(frame, "Checkerboard detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "No checkerboard", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow('Calibration', frame)
    key = cv2.waitKey(100) & 0xFF

    if (key == ord('c') or key == ord('C')) and found:
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        print(f"[+] Đã chụp {len(objpoints)} ảnh.")

        # Chờ người dùng nhả phím 'c'
        while True:
            key2 = cv2.waitKey(100) & 0xFF
            if key2 != ord('c') and key2 != ord('C'):
                break

    elif key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 5:
    print("❌ Cần ít nhất 5 ảnh checkerboard.")
    exit()

# Calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("\n✅ KẾT QUẢ CALIBRATION:")
print("Camera matrix (mtx):")
print(mtx)
print("\nDistortion coefficients:")
print(dist.ravel())

np.savez("arm1_calib_data.npz", mtx=mtx, dist=dist)
print("📁 Đã lưu vào file: arm1_calib_data.npz")
