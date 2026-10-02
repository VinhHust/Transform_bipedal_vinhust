# FILE NÀY DÙNG ĐỂ ĐỌC GHI 2 BÁNH XE TRÊN 1 MODULE

# Đọc/ghi 2 bánh xe (ID 1, 3) trên bus Feetech của một module.
#
# Nhận `bus` (= robot.bus) làm tham số -> file này KHÔNG import lerobot, test được bằng bus giả.
# KHÔNG tự khoá serial: server bọc mọi lời gọi bằng self.serial_lock (chỉ khoá ở một tầng).
# Chỉ đụng 2 tên motor dưới đây -> khớp 4–9 không bao giờ bị ghi từ file này.

import logging

logger = logging.getLogger(__name__)

LEFT = "base_left_wheel"  # ID 3
RIGHT = "base_right_wheel"  # ID 1
WHEELS = [LEFT, RIGHT]

# Operating_Mode của Feetech: 0 = position, 1 = velocity.
# Chính là lerobot OperatingMode.VELOCITY.value; viết số thẳng để khỏi import lerobot.
VELOCITY_MODE = 1


def write_raw(bus, left_raw: int, right_raw: int, num_retry: int = 0) -> None:
    """Ghi Goal_Velocity cho 2 bánh trong 1 gói sync_write.

    normalize=False: số truyền vào đã là raw; driver tự mã hoá dấu âm.
    Lỗi bus -> để exception bay lên cho server báo, không nuốt lỗi.
    """
    bus.sync_write(
        "Goal_Velocity",
        {LEFT: int(left_raw), RIGHT: int(right_raw)},
        normalize=False,
        num_retry=num_retry,
    )


def stop(bus) -> None:
    """Ghi 0 cho 2 bánh. Lệnh an toàn nên cho thử lại nhiều lần."""
    write_raw(bus, 0, 0, num_retry=5)


def read_raw(bus) -> tuple[int, int] | None:
    """Đọc Present_Velocity 2 bánh -> (trái, phải) raw.

    Đọc lỗi trả None, KHÔNG trả (0, 0): "không biết" khác hẳn "đang đứng yên".
    """
    try:
        values = bus.sync_read("Present_Velocity", WHEELS, normalize=False)
    except Exception as e:
        logger.warning(f"Đọc tốc độ bánh lỗi: {e}")
        return None
    return int(values[LEFT]), int(values[RIGHT])


def is_velocity_mode(bus) -> bool:
    """Cả 2 bánh đang ở velocity mode? Đọc lỗi cũng trả False: không chắc thì không cho lái."""
    try:
        modes = bus.sync_read("Operating_Mode", WHEELS, normalize=False)
    except Exception as e:
        logger.warning(f"Đọc Operating_Mode bánh lỗi: {e}")
        return False
    return all(modes[name] == VELOCITY_MODE for name in WHEELS)
