"""Test đọc/ghi bánh (car_mode/wheel_bus.py) bằng bus giả. Không cần robot."""

import pytest

from bipedal_robot.car_mode import wheel_bus
from bipedal_robot.car_mode.wheel_bus import LEFT, RIGHT


class FakeBus:
    """Bus giả: ghi sổ mọi lệnh ghi, lệnh đọc trả số test đặt sẵn."""

    def __init__(self):
        self.writes = []  # (thanh ghi, {motor: giá trị}, normalize, num_retry)
        self.registers = {
            "Present_Velocity": {LEFT: 0, RIGHT: 0},
            "Operating_Mode": {LEFT: 1, RIGHT: 1},
        }
        self.broken = False

    def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
        self.writes.append((data_name, dict(values), normalize, num_retry))

    def sync_read(self, data_name, motors, *, normalize=True, num_retry=0):
        if self.broken:
            raise ConnectionError("bus giả hỏng")
        return {m: self.registers[data_name][m] for m in motors}


@pytest.fixture
def bus():
    return FakeBus()


def test_write_touches_only_wheels(bus):
    wheel_bus.write_raw(bus, 300, -300)
    # đúng 1 lệnh ghi, chỉ có 2 bánh, không có khớp 4–9
    assert bus.writes == [("Goal_Velocity", {LEFT: 300, RIGHT: -300}, False, 0)]


def test_stop_writes_zero_with_retry(bus):
    wheel_bus.stop(bus)
    name, values, normalize, num_retry = bus.writes[-1]
    assert (name, values, normalize) == ("Goal_Velocity", {LEFT: 0, RIGHT: 0}, False)
    assert num_retry > 0


def test_read_returns_left_right(bus):
    bus.registers["Present_Velocity"] = {LEFT: 120, RIGHT: -80}
    assert wheel_bus.read_raw(bus) == (120, -80)


def test_read_failure_is_none_not_zero(bus):
    bus.broken = True
    assert wheel_bus.read_raw(bus) is None


def test_velocity_mode_check(bus):
    assert wheel_bus.is_velocity_mode(bus)
    bus.registers["Operating_Mode"][RIGHT] = 0  # 1 bánh lỡ ở position mode
    assert not wheel_bus.is_velocity_mode(bus)


def test_velocity_mode_unknown_is_false(bus):
    bus.broken = True
    assert not wheel_bus.is_velocity_mode(bus)
