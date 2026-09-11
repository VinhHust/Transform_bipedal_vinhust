import zmq
    
    # 1. Cấu hình địa chỉ IP của Pi (thay IP cho đúngvới máy mobile2 của bạn)
server_ip = "192.168.x.x" # VD: "localhost" nếu chạycùng máy, hoặc IP của Pi nếu chạy từ laptop
server_port = 5556        # Port 5556 là chân trái,
  5555 là chân phải
    
    # Thiết lập kết nối
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://{server_ip}:{server_port}")
socket.setsockopt(zmq.RCVTIMEO, 5000) # Cài đặt
timeout 5 giây để tránh bị treo
    
    # 2. Gửi yêu cầu lấy data
print("Đang gửi yêu cầu lấy data...")
socket.send_json({"type": "feedback"})
    
    # 3. Chờ và đọc data trả về
try:
    response = socket.recv_json()
        
    if response.get("status") == "success":
        print(" Lấy data thành công!")
        print(f"- IMU Quaternion (w,x,y,z):{response.get('quat')}")
        print(f"- Gyroscope (gx,gy,gz): {response.get('gyro')}")
        print(f"- Vị trí 6 Servos (ticks): {response.get('servo_pos')}")
        print(f"- Tốc độ Servos: {response.get('servo_speed')}")
        print(f"- Nhiệt độ Servos: {response.get('servo_temp')}")
    else:
        print("Lỗi từ server:", response)
            
except zmq.error.Again:
    print(" Timeout! Không nhận được phản hồi (hãy kiểm tra lại IP, Port hoặc xem server đã chạy chưa).")
