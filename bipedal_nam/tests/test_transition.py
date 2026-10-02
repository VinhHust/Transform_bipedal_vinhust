"""Test vùng chuyển mode của bub (car_mode/transition.py). Không cần robot."""

import pytest

from bipedal_robot.car_mode.mode_controller import RobotMode
from bipedal_robot.car_mode.transition import check_mode_change, mode_from_bub, transition_range

BIP = {4: {"min": 2000, "max": 3200}}
CAR = {4: {"min": 3095, "max": 4050}}
LO, HI = 3095, 3200


def test_vung_giao_tinh_tu_2_bang():
    assert transition_range(BIP, CAR) == (LO, HI)


def test_2_bang_khong_giao_thi_bao_loi():
    with pytest.raises(ValueError):
        transition_range(BIP, {4: {"min": 3300, "max": 4050}})


@pytest.mark.parametrize("bub", [LO, 3150, HI])
def test_trong_vung_giao_thi_cho_doi_mode(bub):
    assert check_mode_change(RobotMode.CAR, "BIPEDAL", bub, LO, HI) is None
    assert check_mode_change(RobotMode.BIPEDAL, "CAR", bub, LO, HI) is None


@pytest.mark.parametrize("bub", [2048, LO - 1, HI + 1, 3800])
def test_ngoai_vung_giao_thi_tu_choi(bub):
    reason = check_mode_change(RobotMode.CAR, "BIPEDAL", bub, LO, HI)
    assert reason and str(bub) in reason


def test_xin_lai_cung_mode_khong_can_ve_vung_giao():
    # CAR -> CAR sau khi watchdog ngắt: tay đang giơ (3800) vẫn phải lấy lại được quyền lái.
    assert check_mode_change(RobotMode.CAR, "car", 3800, LO, HI) is None


@pytest.mark.parametrize(
    "bub, mode", [(2048, RobotMode.BIPEDAL), (LO, RobotMode.CAR), (3800, RobotMode.CAR)]
)
def test_mode_luc_khoi_dong_theo_bub(bub, mode):
    assert mode_from_bub(bub, LO, HI) is mode
