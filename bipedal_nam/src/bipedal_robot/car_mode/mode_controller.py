# Bộ não mode BIPEDAL/CAR trên MỘT Pi (PLAN_DIFF_DRIVE_CONTROLLER.md §7).
#
# File này lo 3 việc:
# 1. Nhớ robot đang ở mode nào -> quyết định lệnh nào được chạy, lệnh nào bị từ chối.
# 2. Mỗi lần đổi mode phát một "vé" (session) mới -> lệnh mang vé cũ bị bỏ.
# 3. Giữ lệnh bánh mới nhất; tick() đều đặn đẩy xuống bánh (có ramp), mất lệnh thì tự dừng.
#
# Server gọi mọi hàm ở đây từ MỘT luồng (vòng chính run()). Mọi lần chạm bus đi qua serial_lock.
# clock/sleep truyền vào được -> test dùng đồng hồ giả, không phải chờ thật.

import logging
import secrets
import time
from enum import Enum

from . import diff_drive, wheel_bus

logger = logging.getLogger(__name__)

TICK_PERIOD_S = 0.02  # ghi bánh tối đa 50 Hz
STOP_WAIT_S = 1.0  # CAR -> BIPEDAL: chờ bánh dừng hẳn tối đa bao lâu
STOP_TOL_RAW = 20  # |Present_Velocity| từ mức này trở xuống coi như đã dừng


# RobotMode dùng để chuyển mode, import vào leg_server
class RobotMode(str, Enum):
    BIPEDAL = "BIPEDAL"
    CAR = "CAR"


def _error(message: str) -> dict:
    return {"status": "error", "message": message}


class ModeController:
    def __init__(self, bus, cfg, serial_lock, clock=time.monotonic, sleep=time.sleep):
        """Tạo SAU robot.configure() (lúc đó bus đã có và bánh vừa được ghi 0).

        cfg = None nghĩa là thiếu config bánh: bipedal vẫn chạy bình thường, chỉ từ chối sang CAR.
        """
        self.bus = bus
        self.cfg = cfg
        self.serial_lock = serial_lock
        self.clock = clock
        self.sleep = sleep

        self.mode = RobotMode.BIPEDAL
        self.session = None  # None = chưa ai đổi mode -> client cũ gửi move không vé vẫn chạy
        self.drive_armed = False  # True = đang được phép lái
        self.last_seq = -1
        self.last_drive_time = None  # lúc nhận drive gần nhất; None = chưa có -> chưa đếm timeout

        self.target = (0.0, 0.0)  # tốc độ bánh (trái, phải) rad/s client muốn
        self.current = (0.0, 0.0)  # tốc độ bánh đang ra lệnh, sau ramp
        self.written_raw = (0, 0)  # raw đã ghi thành công lần cuối; configure() vừa ghi 0
        self.last_tick = clock()

    # ======== Đổi mode ========

    def set_mode(self, mode_name) -> dict:
        try:
            mode = RobotMode(str(mode_name).upper())
        except ValueError:
            return _error(f"Mode không hợp lệ: {mode_name!r}")
        if mode is RobotMode.CAR:
            return self._enter_car()
        return self._enter_bipedal()

    def _enter_car(self) -> dict:
        # Dừng mọi thứ của phiên cũ TRƯỚC, rồi mới kiểm tra điều kiện.
        if not self._stop_wheels():
            return _error("Không ghi được 0 cho bánh")
        if self.cfg is None:
            return _error("Thiếu config bánh -> không cho sang CAR")
        with self.serial_lock:
            velocity_ok = wheel_bus.is_velocity_mode(self.bus)
        if not velocity_ok:
            return _error("Bánh không ở velocity mode (hoặc đọc lỗi)")
        self._new_session(RobotMode.CAR)
        self.drive_armed = True
        return self._ack()

    def _enter_bipedal(self) -> dict:
        if not self._stop_wheels():
            return _error("Không ghi được 0 cho bánh")
        if not self._wait_wheels_stopped():
            return _error("Bánh chưa dừng hẳn -> chưa cho về BIPEDAL")
        self._new_session(RobotMode.BIPEDAL)
        return self._ack()

    def _new_session(self, mode: RobotMode) -> None:
        self.mode = mode
        # Ngẫu nhiên chứ không đếm 1, 2, 3: server khởi động lại cũng không trùng vé cũ.
        self.session = secrets.token_hex(4)
        self.last_seq = -1
        self.last_drive_time = None
        logger.info(f"Mode -> {mode.value}, session {self.session}")

    def _ack(self) -> dict:
        return {"status": "success", "mode": self.mode.value, "session": self.session}

    def _wait_wheels_stopped(self) -> bool:
        """Ghi 0 chưa đủ: đọc lại tới khi bánh thật sự đứng. Hết giờ mà chưa chắc -> False."""
        deadline = self.clock() + STOP_WAIT_S
        while True:
            with self.serial_lock:  # chỉ khoá lúc đọc, KHÔNG khoá lúc ngủ
                raw = wheel_bus.read_raw(self.bus)
            if raw is not None and max(abs(raw[0]), abs(raw[1])) <= STOP_TOL_RAW:
                return True
            if self.clock() >= deadline:
                return False
            self.sleep(TICK_PERIOD_S)

    # ======== Lệnh bánh ========

    def drive(self, command: dict) -> dict:
        """Lệnh drive từ PULL: {"v", "omega", "session", "seq"}. Chỉ đổi target, tick() mới ghi bus."""
        reason = self._check_drive(command)
        if reason:
            return _error(reason)
        try:
            v, omega = diff_drive.clamp_body(self.cfg, float(command["v"]), float(command["omega"]))
        except (KeyError, TypeError, ValueError) as e:
            return _error(f"Lệnh drive sai: {e}")
        left, right = diff_drive.body_to_wheels(self.cfg, v, omega)
        self.target = diff_drive.saturate_wheels(self.cfg, left, right)
        self.last_seq = command["seq"]
        self.last_drive_time = self.clock()
        return {"status": "success"}

    def _check_drive(self, command: dict):
        if self.mode is not RobotMode.CAR:
            return "Không ở CAR"
        if not self.drive_armed:
            return "Chưa bật lái: gọi lại set_mode CAR"
        if command.get("session") != self.session:
            return "Sai session"
        seq = command.get("seq")
        if not isinstance(seq, int) or seq <= self.last_seq:
            return "seq thiếu, lặp lại hoặc cũ"
        return None

    def stop_drive(self) -> dict:
        """Mode nào cũng được, không cần vé: dừng là lệnh an toàn."""
        if not self._stop_wheels():
            return _error("Ghi 0 cho bánh lỗi")
        return {"status": "success", "message": "Wheels stopped"}

    def _stop_wheels(self) -> bool:
        """Ghi 0 NGAY, không ramp, và thu hồi quyền lái. Trả False nếu ghi lỗi."""
        self.drive_armed = False
        self.target = (0.0, 0.0)
        self.current = (0.0, 0.0)
        try:
            with self.serial_lock:
                wheel_bus.stop(self.bus)
        except Exception as e:
            self.written_raw = None  # không chắc bánh đang chạy gì -> tick() sẽ ghi lại
            logger.error(f"Dừng bánh lỗi: {e}")
            return False
        self.written_raw = (0, 0)
        return True

    def tick(self) -> None:
        """Server gọi mỗi vòng lặp chính. Tự giới hạn 50 Hz."""
        now = self.clock()
        dt = now - self.last_tick
        if dt < TICK_PERIOD_S or self.cfg is None:
            return
        self.last_tick = now

        # Đồng hồ canh: đã lái mà lâu quá không có lệnh mới -> dừng NGAY, thu hồi quyền lái.
        if self.drive_armed and self.last_drive_time is not None:
            if now - self.last_drive_time > self.cfg.command_timeout_s:
                logger.warning("Mất lệnh drive quá lâu -> dừng bánh")
                self._stop_wheels()
                return

        # Vòng chính từng bị chặn lâu (vd chờ bánh dừng) -> dt to -> ramp nhảy một bước lớn. Kẹp lại.
        dt = min(dt, 2 * TICK_PERIOD_S)
        self.current = diff_drive.ramp_wheels(self.cfg, self.current, self.target, dt)
        raw = diff_drive.wheels_to_raw(self.cfg, *self.current)
        if raw == self.written_raw:
            return  # không đổi thì khỏi ghi -> đỡ chiếm bus của chân/tay
        try:
            with self.serial_lock:
                wheel_bus.write_raw(self.bus, *raw)
            self.written_raw = raw
        except Exception as e:
            self.written_raw = None
            logger.error(f"Ghi bánh lỗi: {e}")

    # ======== Quyền cho lệnh chân/tay ========

    def check_move(self, command: dict):
        """move/home của chân. None = cho chạy; chuỗi = lý do từ chối."""
        if self.mode is not RobotMode.BIPEDAL:
            return "Đang ở CAR: không nhận lệnh chân"
        # session None = chưa ai đổi mode -> giữ tương thích client cũ không gửi vé.
        if self.session is not None and command.get("session") != self.session:
            return "Sai session"
        return None

    def check_command(self, command: dict):
        """Trạm gác chung cho server: xếp lệnh vào đúng hàm kiểm tra.

        None = cho chạy (hoặc không phải lệnh cử động chân/tay); chuỗi = lý do từ chối.
        """
        cmd_type = command.get("type")
        if cmd_type in ("move", "home"):
            return self.check_move(command)
        if cmd_type == "arm_move":
            return self.check_arm_move(command)
        return None

    def check_arm_move(self, command: dict):
        """arm_move (khớp 4–9 làm tay gắp ở CAR). None = cho chạy; chuỗi = lý do từ chối."""
        if self.mode is not RobotMode.CAR:
            return "Không ở CAR"
        if command.get("session") != self.session:
            return "Sai session"
        return None

    # ======== Phản hồi ========

    def feedback(self) -> dict:
        measured = None  # None = đọc lỗi, KHÔNG có nghĩa là đang đứng yên
        if self.cfg is not None:
            with self.serial_lock:
                raw = wheel_bus.read_raw(self.bus)
            if raw is not None:
                v, omega = diff_drive.wheels_to_body(
                    self.cfg, *diff_drive.raw_to_wheels(self.cfg, *raw)
                )
                measured = {"raw": list(raw), "v": v, "omega": omega}
        age = None if self.last_drive_time is None else self.clock() - self.last_drive_time
        return {
            "status": "success",
            "mode": self.mode.value,
            "drive_armed": self.drive_armed,
            "cmd_age_s": age,
            "target_wheels": list(self.target),
            "current_wheels": list(self.current),
            "measured": measured,
        }
