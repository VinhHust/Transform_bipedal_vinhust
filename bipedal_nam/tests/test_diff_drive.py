"""Test toán xe vi sai (bipedal_robot/diff_drive.py). Không cần robot, chạy trên laptop."""

import dataclasses
import math

import pytest

from bipedal_robot.car_mode.diff_drive import (
    DiffDriveConfig,
    body_to_raw,
    body_to_wheels,
    clamp_body,
    load_config,
    ramp_wheels,
    raw_to_wheels,
    saturate_wheels,
    wheels_to_body,
    wheels_to_raw,
)


@pytest.fixture
def cfg():
    """Số tròn để nhẩm tay: r = 5 cm, b = 20 cm (plan §10 bước 2). KHÔNG phải số xe thật."""
    return DiffDriveConfig(
        wheel_radius_m=0.05,
        wheel_separation_m=0.20,
        left_direction=1,
        right_direction=1,
        motor_to_wheel_ratio=1.0,
        motor_steps_per_rev=4096,
        max_raw=3000,
        max_v_m_s=1.0,
        max_omega_rad_s=5.0,
        max_wheel_accel_rad_s2=4.0,
        command_timeout_s=0.25,
    )


# ======== Thân xe <-> bánh ========


def test_forward(cfg):
    # 0.1 m/s / 0.05 m = 2 rad/s mỗi bánh
    assert body_to_wheels(cfg, 0.10, 0.0) == pytest.approx((2.0, 2.0))


def test_backward(cfg):
    assert body_to_wheels(cfg, -0.10, 0.0) == pytest.approx((-2.0, -2.0))


def test_spin_in_place_left(cfg):
    # quay trái: bánh trái lùi, bánh phải tiến, cùng độ lớn
    assert body_to_wheels(cfg, 0.0, 1.0) == pytest.approx((-2.0, 2.0))


def test_curve(cfg):
    # bánh trái (0.1 - 0.05)/0.05 = 1, bánh phải (0.1 + 0.05)/0.05 = 3
    assert body_to_wheels(cfg, 0.10, 0.5) == pytest.approx((1.0, 3.0))


@pytest.mark.parametrize("v, omega", [(0.1, 0.0), (0.0, 1.0), (-0.05, 0.3), (0.12, -0.7)])
def test_round_trip(cfg, v, omega):
    left, right = body_to_wheels(cfg, v, omega)
    assert wheels_to_body(cfg, left, right) == pytest.approx((v, omega))


def test_clamp_body_keeps_curve(cfg):
    # v gấp đôi max -> cả v và omega cùng co 1/2
    assert clamp_body(cfg, 2.0, 2.0) == pytest.approx((1.0, 1.0))


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_reject_nan_inf(cfg, bad):
    with pytest.raises(ValueError):
        clamp_body(cfg, bad, 0.0)
    with pytest.raises(ValueError):
        clamp_body(cfg, 0.0, bad)


# ======== Giới hạn ========


def test_saturate_keeps_ratio(cfg):
    left, right = saturate_wheels(cfg, 2.0, 9.0)
    assert right == pytest.approx(cfg.max_wheel_rad_s)
    assert right / left == pytest.approx(4.5)


def test_ramp_limits_step(cfg):
    # 4 rad/s² * 0.1 s = đổi tối đa 0.4 rad/s mỗi tick
    assert ramp_wheels(cfg, (0.0, 0.0), (2.0, 2.0), 0.1) == pytest.approx((0.4, 0.4))


def test_ramp_keeps_ratio(cfg):
    left, right = ramp_wheels(cfg, (0.0, 0.0), (1.0, 3.0), 0.1)
    assert (left, right) == pytest.approx((0.4 / 3, 0.4))


def test_ramp_reaches_target(cfg):
    assert ramp_wheels(cfg, (1.9, 1.9), (2.0, 2.0), 0.1) == pytest.approx((2.0, 2.0))


# ======== Bánh <-> raw ========


def test_raw_unit_from_old_code(cfg):
    # 1 rad/s = 4096 / 2π ≈ 651.9 bước/giây -> làm tròn 652
    assert wheels_to_raw(cfg, 1.0, 1.0) == (652, 652)


def test_motor_direction(cfg):
    flipped = dataclasses.replace(cfg, right_direction=-1)
    assert wheels_to_raw(flipped, 1.0, 1.0) == (652, -652)
    # đọc ngược phải ra lại tốc độ bánh dương
    assert raw_to_wheels(flipped, 652, -652) == pytest.approx((1.0, 1.0), abs=1e-3)


def test_body_to_raw_never_exceeds_max_raw(cfg):
    left, right = body_to_raw(cfg, 1.0, 5.0)
    assert max(abs(left), abs(right)) == cfg.max_raw


# ======== Config ========


def test_missing_value_rejected(cfg):
    with pytest.raises(ValueError):
        dataclasses.replace(cfg, wheel_separation_m=None)
    with pytest.raises(ValueError):
        dataclasses.replace(cfg, left_direction=0)


@pytest.mark.parametrize("module", ["left", "right"])
def test_real_config_files_load(module):
    real = load_config(module)
    assert real.wheel_radius_m == pytest.approx(0.05)
    assert real.wheel_separation_m == pytest.approx(0.24145)
