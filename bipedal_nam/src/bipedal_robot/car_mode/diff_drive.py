# Toán xe vi sai 2 bánh cho mode CAR (PLAN_DIFF_DRIVE_CONTROLLER.md §5).
# Chỉ có toán, KHÔNG import driver motor -> test được trên laptop.
#
# Quy ước:
# - v: m/s, dương = tiến về phía mũi xe (chỗ dock).
# - omega: rad/s, dương = quay trái (ngược kim đồng hồ, nhìn từ trên).
# - Tốc độ bánh (rad/s) dương = bánh lăn đẩy xe tiến. Dấu lắp motor chỉ nhân ở bước đổi ra raw.
#
# Một lệnh đi qua các bước:
#   (v, omega) -> clamp_body -> body_to_wheels -> saturate_wheels -> ramp_wheels -> wheels_to_raw
# ramp_wheels cần nhớ lệnh tick trước -> server giữ trạng thái đó, file này chỉ làm phép tính.

import json
import math
from dataclasses import dataclass
from pathlib import Path

# config/ nằm ngay cạnh file này
CONFIG_DIR = Path(__file__).resolve().parent / "config"


# ======== Cấu hình ========


@dataclass(frozen=True)
class DiffDriveConfig:
    wheel_radius_m: float  # bán kính bánh
    wheel_separation_m: float  # tâm bánh trái -> tâm bánh phải
    left_direction: int  # +1/-1: chọn sao cho raw dương -> bánh trái đẩy xe tiến
    right_direction: int
    motor_to_wheel_ratio: float  # vòng trục servo / vòng bánh; gắn thẳng = 1
    motor_steps_per_rev: int  # số bước encoder trong 1 vòng trục servo
    max_raw: int  # trần |Goal_Velocity| mỗi bánh
    max_v_m_s: float
    max_omega_rad_s: float
    max_wheel_accel_rad_s2: float
    command_timeout_s: float

    def __post_init__(self):
        # Số nào chưa đo (null trong JSON -> None) hoặc vô lý thì báo lỗi ngay lúc nạp,
        # không để tới lúc bánh đang quay mới phát hiện.
        positive = (
            "wheel_radius_m",
            "wheel_separation_m",
            "motor_to_wheel_ratio",
            "motor_steps_per_rev",
            "max_raw",
            "max_v_m_s",
            "max_omega_rad_s",
            "max_wheel_accel_rad_s2",
            "command_timeout_s",
        )
        for name in positive:
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} phải là số dương, đang là {value!r}")
        for name in ("left_direction", "right_direction"):
            if getattr(self, name) not in (1, -1):
                raise ValueError(f"{name} phải là 1 hoặc -1, đang là {getattr(self, name)!r}")

    @property
    def raw_per_motor_rad_s(self) -> float:
        # Theo code cũ (_degps_to_raw): 1 raw = 1 bước/giây.
        # 1 vòng = 2π rad = motor_steps_per_rev bước -> 1 rad/s = steps / 2π raw.
        return self.motor_steps_per_rev / (2 * math.pi)

    @property
    def max_wheel_rad_s(self) -> float:
        # max_raw đổi ngược ra tốc độ bánh.
        return self.max_raw / (self.raw_per_motor_rad_s * self.motor_to_wheel_ratio)

    @classmethod
    def from_json(cls, path) -> "DiffDriveConfig":
        # Thiếu key hoặc gõ sai tên key -> cls(**...) báo TypeError. Cố ý: không đoán giá trị.
        with open(path, "r") as f:
            return cls(**json.load(f))


def load_config(module: str) -> DiffDriveConfig:
    """module = "left" | "right" -> đọc config/diff_drive_<module>.json."""
    if module not in ("left", "right"):
        raise ValueError(f"module phải là 'left' hoặc 'right', đang là {module!r}")
    return DiffDriveConfig.from_json(CONFIG_DIR / f"diff_drive_{module}.json")


# ======== Thân xe <-> bánh ========


def clamp_body(cfg: DiffDriveConfig, v: float, omega: float) -> tuple[float, float]:
    """Giảm v và omega CÙNG một hệ số -> đường cua giữ nguyên, chỉ chậm hơn.

    Cùng ý với dock_math.clamp_command. Lệnh NaN/Inf bị từ chối ở đây.
    """
    if not (math.isfinite(v) and math.isfinite(omega)):
        raise ValueError(f"Lệnh không hợp lệ: v={v}, omega={omega}")
    s = 1.0
    if v != 0:
        s = min(s, cfg.max_v_m_s / abs(v))
    if omega != 0:
        s = min(s, cfg.max_omega_rad_s / abs(omega))
    return s * v, s * omega


def body_to_wheels(cfg: DiffDriveConfig, v: float, omega: float) -> tuple[float, float]:
    """(v, omega) -> tốc độ (trái, phải) rad/s. Quay trái thì bánh phải chạy nhanh hơn."""
    half = cfg.wheel_separation_m / 2
    left = (v - omega * half) / cfg.wheel_radius_m
    right = (v + omega * half) / cfg.wheel_radius_m
    return left, right


def wheels_to_body(cfg: DiffDriveConfig, left: float, right: float) -> tuple[float, float]:
    """Ngược lại: tốc độ 2 bánh rad/s -> (v, omega). Dùng khi đọc phản hồi."""
    r = cfg.wheel_radius_m
    v = r * (right + left) / 2
    omega = r * (right - left) / cfg.wheel_separation_m
    return v, omega


# ======== Giới hạn tốc độ và gia tốc bánh ========


def saturate_wheels(cfg: DiffDriveConfig, left: float, right: float) -> tuple[float, float]:
    """Bánh nào vượt max_wheel_rad_s thì giảm CẢ HAI cùng hệ số -> giữ tỉ lệ 2 bánh."""
    biggest = max(abs(left), abs(right))
    if biggest <= cfg.max_wheel_rad_s:
        return left, right
    s = cfg.max_wheel_rad_s / biggest
    return s * left, s * right


def ramp_wheels(
    cfg: DiffDriveConfig,
    prev: tuple[float, float],
    target: tuple[float, float],
    dt: float,
) -> tuple[float, float]:
    """Đi từ lệnh tick trước tới lệnh mới, mỗi tick đổi tối đa max_wheel_accel * dt.

    Hai bánh cùng co một hệ số -> không bánh nào tăng tốc trước làm xe lạng.
    Dừng khẩn (stop/timeout) KHÔNG đi qua hàm này: ghi thẳng 0.
    """
    if dt <= 0:
        return prev
    d_left = target[0] - prev[0]
    d_right = target[1] - prev[1]
    step = cfg.max_wheel_accel_rad_s2 * dt
    biggest = max(abs(d_left), abs(d_right))
    if biggest <= step:
        return target
    s = step / biggest
    return prev[0] + s * d_left, prev[1] + s * d_right


# ======== Bánh <-> raw của servo ========


def wheels_to_raw(cfg: DiffDriveConfig, left: float, right: float) -> tuple[int, int]:
    """rad/s bánh -> số nguyên ghi vào Goal_Velocity. Dấu lắp motor nhân ở đây."""
    k = cfg.motor_to_wheel_ratio * cfg.raw_per_motor_rad_s
    raw_left = round(cfg.left_direction * k * left)
    raw_right = round(cfg.right_direction * k * right)
    # Chốt chặn cuối: gọi sai thứ tự (quên saturate) thì raw vẫn không vượt max_raw.
    raw_left = max(-cfg.max_raw, min(cfg.max_raw, raw_left))
    raw_right = max(-cfg.max_raw, min(cfg.max_raw, raw_right))
    return raw_left, raw_right


def raw_to_wheels(cfg: DiffDriveConfig, raw_left: int, raw_right: int) -> tuple[float, float]:
    """Present_Velocity đọc về -> rad/s bánh: bỏ dấu lắp, bỏ tỉ số truyền."""
    k = cfg.motor_to_wheel_ratio * cfg.raw_per_motor_rad_s
    return cfg.left_direction * raw_left / k, cfg.right_direction * raw_right / k


def body_to_raw(cfg: DiffDriveConfig, v: float, omega: float) -> tuple[int, int]:
    """Cả chuỗi (v, omega) -> raw, KHÔNG ramp. Dùng cho test và thử nhanh."""
    v, omega = clamp_body(cfg, v, omega)
    left, right = saturate_wheels(cfg, *body_to_wheels(cfg, v, omega))
    return wheels_to_raw(cfg, left, right)
