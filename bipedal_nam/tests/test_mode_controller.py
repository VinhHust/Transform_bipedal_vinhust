"""Test bộ não mode (car_mode/mode_controller.py) bằng bus giả + đồng hồ giả. Không cần robot."""

import math
import threading

import pytest

from bipedal_robot.car_mode.diff_drive import DiffDriveConfig
from bipedal_robot.car_mode.mode_controller import STOP_WAIT_S, ModeController, RobotMode
from bipedal_robot.car_mode.wheel_bus import LEFT, RIGHT


class FakeClock:
    """Đồng hồ giả: chỉ nhảy khi test cho nhảy -> test 1 giây chạy xong trong tích tắc."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeWheelBus:
    """Bánh lý tưởng: ghi Goal_Velocity bao nhiêu thì Present_Velocity bằng bấy nhiêu."""

    def __init__(self):
        self.writes = []  # các lần ghi Goal_Velocity: {motor: raw}
        self.velocity = {LEFT: 0, RIGHT: 0}
        self.modes = {LEFT: 1, RIGHT: 1}
        self.stuck = False  # True = bánh không chịu đổi tốc độ

    def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
        self.writes.append(dict(values))
        if not self.stuck:
            self.velocity.update(values)

    def sync_read(self, data_name, motors, *, normalize=True, num_retry=0):
        table = {"Present_Velocity": self.velocity, "Operating_Mode": self.modes}[data_name]
        return {m: table[m] for m in motors}

    def last_raw(self):
        return self.writes[-1][LEFT], self.writes[-1][RIGHT]


CFG = DiffDriveConfig(
    wheel_radius_m=0.05,
    wheel_separation_m=0.24145,
    left_direction=1,
    right_direction=1,
    motor_to_wheel_ratio=1.0,
    motor_steps_per_rev=4096,
    max_raw=1500,
    max_v_m_s=0.1,
    max_omega_rad_s=0.5,
    max_wheel_accel_rad_s2=4.0,
    command_timeout_s=0.25,
)


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def bus():
    return FakeWheelBus()


@pytest.fixture
def ctrl(bus, clock):
    return ModeController(bus, CFG, threading.Lock(), clock=clock, sleep=clock.sleep)


def drive_cmd(session, seq, v, omega=0.0):
    return {"type": "drive", "v": v, "omega": omega, "session": session, "seq": seq}


def run(ctrl, clock, seconds, session=None, v=0.0, omega=0.0):
    """Vòng chính giả: 10 ms tick một lần. Có session thì gửi drive mỗi 20 ms như client thật."""
    for i in range(round(seconds / 0.01)):
        clock.now += 0.01
        if session is not None and i % 2 == 0:
            ctrl.drive(drive_cmd(session, ctrl.last_seq + 1, v, omega))
        ctrl.tick()


# ======== Mặc định BIPEDAL ========


def test_starts_bipedal_and_wheels_stay_silent(ctrl, bus, clock):
    assert ctrl.mode is RobotMode.BIPEDAL
    assert ctrl.drive(drive_cmd(None, 0, 0.1))["status"] == "error"
    run(ctrl, clock, 0.5)
    assert bus.writes == []  # không ghi gì xuống bánh


def test_old_client_move_without_session_still_works(ctrl):
    assert ctrl.check_move({"type": "move", "positions": [0] * 6}) is None


# ======== Sang CAR ========


def test_car_needs_config(bus, clock):
    ctrl = ModeController(bus, None, threading.Lock(), clock=clock, sleep=clock.sleep)
    assert ctrl.set_mode("car")["status"] == "error"
    assert ctrl.mode is RobotMode.BIPEDAL


def test_car_needs_velocity_mode(ctrl, bus):
    bus.modes[RIGHT] = 0  # bánh phải lỡ ở position mode
    assert ctrl.set_mode("car")["status"] == "error"
    assert ctrl.mode is RobotMode.BIPEDAL


def test_car_blocks_legs_allows_arm(ctrl):
    session = ctrl.set_mode("car")["session"]
    assert ctrl.check_move({"type": "move"}) is not None
    assert ctrl.check_arm_move({"type": "arm_move", "session": session}) is None
    assert ctrl.check_arm_move({"type": "arm_move", "session": "sai"}) is not None


# ======== Lái ========


def test_drive_ramps_up_then_reaches_target(ctrl, bus, clock):
    session = ctrl.set_mode("car")["session"]
    run(ctrl, clock, 0.05, session, v=0.1)
    first = bus.writes[1]  # writes[0] là lệnh 0 lúc set_mode
    assert 0 < first[LEFT] < 100  # ramp: nhích dần, không nhảy thẳng lên 1304
    run(ctrl, clock, 1.0, session, v=0.1)
    assert bus.last_raw() == (1304, 1304)  # 0.1 m/s = 2 rad/s = 1304 raw


def test_repeated_or_old_seq_rejected(ctrl):
    session = ctrl.set_mode("car")["session"]
    assert ctrl.drive(drive_cmd(session, 5, 0.1))["status"] == "success"
    assert ctrl.drive(drive_cmd(session, 5, 0.1))["status"] == "error"
    assert ctrl.drive(drive_cmd(session, 4, 0.1))["status"] == "error"
    assert ctrl.drive(drive_cmd(session, 6, 0.1))["status"] == "success"


def test_wrong_session_rejected(ctrl):
    ctrl.set_mode("car")
    assert ctrl.drive(drive_cmd("vé-giả", 0, 0.1))["status"] == "error"


def test_nan_rejected(ctrl):
    session = ctrl.set_mode("car")["session"]
    assert ctrl.drive(drive_cmd(session, 0, math.nan))["status"] == "error"
    assert ctrl.target == (0.0, 0.0)


# ======== Dừng ========


def test_timeout_stops_at_once_and_disarms(ctrl, bus, clock):
    session = ctrl.set_mode("car")["session"]
    run(ctrl, clock, 1.0, session, v=0.1)
    run(ctrl, clock, 0.3)  # client im lặng > 0.25 s
    assert bus.last_raw() == (0, 0)
    assert not ctrl.drive_armed
    # lệnh muộn cùng vé cũng không làm bánh chạy lại
    assert ctrl.drive(drive_cmd(session, ctrl.last_seq + 1, 0.1))["status"] == "error"


def test_rearm_after_timeout_gives_new_session(ctrl, clock):
    old = ctrl.set_mode("car")["session"]
    run(ctrl, clock, 0.1, old, v=0.1)
    run(ctrl, clock, 0.3)
    new = ctrl.set_mode("car")["session"]
    assert new != old
    assert ctrl.drive(drive_cmd(old, 100, 0.1))["status"] == "error"
    assert ctrl.drive(drive_cmd(new, 0, 0.1))["status"] == "success"


def test_no_timeout_before_first_drive(ctrl, clock):
    ctrl.set_mode("car")
    run(ctrl, clock, 2.0)  # chưa gửi drive nào -> đồng hồ canh chưa đếm
    assert ctrl.drive_armed


def test_stop_drive_is_immediate(ctrl, bus, clock):
    session = ctrl.set_mode("car")["session"]
    run(ctrl, clock, 1.0, session, v=0.1)
    ctrl.stop_drive()
    assert bus.last_raw() == (0, 0)  # ghi 0 ngay, không ramp xuống từ từ
    assert not ctrl.drive_armed


# ======== Về BIPEDAL ========


def test_back_to_bipedal_after_wheels_stop(ctrl, bus, clock):
    session = ctrl.set_mode("car")["session"]
    run(ctrl, clock, 1.0, session, v=0.1)
    ack = ctrl.set_mode("bipedal")
    assert ack["status"] == "success"
    assert ctrl.mode is RobotMode.BIPEDAL
    assert bus.last_raw() == (0, 0)


def test_back_to_bipedal_refused_if_wheel_stuck(ctrl, bus, clock):
    ctrl.set_mode("car")
    bus.velocity = {LEFT: 500, RIGHT: 500}
    bus.stuck = True  # ghi 0 nhưng bánh vẫn quay
    start = clock.now
    assert ctrl.set_mode("bipedal")["status"] == "error"
    assert ctrl.mode is RobotMode.CAR
    assert clock.now - start == pytest.approx(STOP_WAIT_S, abs=0.05)  # đã chờ đủ rồi mới bỏ


def test_every_switch_gives_new_session(ctrl):
    s1 = ctrl.set_mode("car")["session"]
    s2 = ctrl.set_mode("bipedal")["session"]
    s3 = ctrl.set_mode("car")["session"]
    assert len({s1, s2, s3}) == 3
    # vé CAR lần 1 không dùng được cho CAR lần 2
    assert ctrl.check_arm_move({"type": "arm_move", "session": s1}) is not None
    # về BIPEDAL rồi thì client chân phải mang vé
    ctrl.set_mode("bipedal")
    assert ctrl.check_move({"type": "move"}) is not None


# ======== Phản hồi ========


def test_feedback_read_error_is_none(ctrl, bus):
    def broken(*args, **kwargs):
        raise ConnectionError("bus hỏng")

    bus.sync_read = broken
    assert ctrl.feedback()["measured"] is None
