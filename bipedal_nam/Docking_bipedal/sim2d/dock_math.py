# Toán docking 2D: đổi hệ toạ độ, standby, Astolfi.
# Thuần numpy, KHÔNG import matplotlib -> sau này chép sang xe thật được.
#
# Quy ước (docs/PLAN_DOCKING_2D_PYTHON.md §3):
# - Pose = np.array([x, y, theta]), đơn vị mét và radian.
# - Mọi hệ: x hướng trước, y hướng trái, góc dương = ngược kim đồng hồ (nhìn từ trên).
# - pose_X_in_Y = pose của hệ X nhìn trong hệ Y.

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np


# ======== Pose cơ bản ========


def pose(x, y, theta):
    """Tạo pose [x, y, theta] (m, m, rad)."""
    return np.array([x, y, theta], dtype=float)


def wrap_angle(a):
    """Đưa góc về [-pi, pi). Vd: 190° -> -170°, 180° -> -180°."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot(theta):
    """Ma trận quay R(theta): xoay vector 2D ngược kim đồng hồ một góc theta."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def compose(pose_X_in_Y, pose_Z_in_X):
    """Biết X trong Y và Z trong X -> Z trong Y.

    Vị trí: xoay p2 theo góc của X (theta1, KHÔNG phải theta2) rồi cộng p1.
    """
    p = pose_X_in_Y[:2] + rot(pose_X_in_Y[2]) @ pose_Z_in_X[:2]
    return pose(p[0], p[1], wrap_angle(pose_X_in_Y[2] + pose_Z_in_X[2]))


def inverse(pose_X_in_Y):
    """X trong Y -> Y trong X."""
    p = -rot(pose_X_in_Y[2]).T @ pose_X_in_Y[:2]
    return pose(p[0], p[1], wrap_angle(-pose_X_in_Y[2]))


def transform_points(pose_X_in_Y, pts_in_X):
    """Đổi nhiều điểm 2D (mảng N×2) từ hệ X sang hệ Y. Dùng để vẽ thân xe."""
    pts = np.atleast_2d(pts_in_X)
    return pts @ rot(pose_X_in_Y[2]).T + pose_X_in_Y[:2]


# ======== Động học xe vi sai ========


def integrate(pose_A_in_W, v, omega, dt):
    """Xe chạy tiến v (m/s), quay omega (rad/s) trong dt giây -> pose mới.

    Euler: coi như trong dt xe đi thẳng theo hướng ĐẦU bước (dùng cùng theta cho cả x và y),
    rồi mới xoay mũi. Xe vi sai chỉ đi theo hướng mũi -> không có vận tốc ngang.
    """
    x, y, theta = pose_A_in_W
    return pose(
        x + v * np.cos(theta) * dt,
        y + v * np.sin(theta) * dt,
        wrap_angle(theta + omega * dt),
    )


# ======== Camera ảo (phía thế giới thật) ========


def virtual_camera(pose_A_in_W, pose_C_in_A, pose_T_in_W):
    """Tag nằm đâu trong mắt camera. Lý tưởng: không nhiễu, không giới hạn góc nhìn.

    Dùng pose THẬT của A -> chỉ simulator được gọi hàm này, controller thì không.
    """
    pose_C_in_W = compose(pose_A_in_W, pose_C_in_A)
    return compose(inverse(pose_C_in_W), pose_T_in_W)


# ======== Phía controller: chỉ dùng số đo camera + hình học khai báo ========


def estimate_female(pose_T_in_C, pose_C_in_A, pose_F_in_T):
    """Tag camera thấy -> female trong hệ A hiện tại."""
    pose_T_in_A = compose(pose_C_in_A, pose_T_in_C)
    return compose(pose_T_in_A, pose_F_in_T)


def compute_standby(pose_F, pose_M_in_A, D):
    """Pose tâm bánh A cần đạt: male thẳng trục female, chĩa vào female, cách miệng đúng D.

    F cho trong hệ nào thì đích trả về trong hệ đó (controller dùng hệ A hiện tại).
    """
    phi_F = pose_F[2]
    n = np.array([np.cos(phi_F), np.sin(phi_F)])  # trục female, hướng ra ngoài
    theta_G = wrap_angle(phi_F + np.pi - pose_M_in_A[2])  # male chĩa ngược n
    G = pose_F[:2] + D * n - rot(theta_G) @ pose_M_in_A[:2]
    return pose(G[0], G[1], theta_G)


# ======== Bộ điều khiển Astolfi (plan §5.2–5.4) ========


@dataclass(frozen=True)
class ControllerConfig:
    k_rho: float = 0.5  # càng xa càng chạy nhanh
    k_alpha: float = 1.5  # xoay mũi về phía đích; phải lớn hơn k_rho
    k_beta: float = -0.6  # PHẢI ÂM: lái để tới nơi đúng hướng
    k_heading: float = 1.0  # xoay tại chỗ ở cuối: ω = k_heading·θ_G
    v_max: float = 0.15  # m/s
    omega_max: float = 1.0  # rad/s
    # Ngưỡng dừng của controller: dùng CHUNG cho "chuyển sang xoay tại chỗ" và "xong".
    # Astolfi chỉ tiến sát dần -> xe dừng ngay khi vừa lọt ngưỡng -> sai số cuối ≈ ngưỡng.
    # Nên đặt CHẶT hơn dung sai cơ khí (có lề): ở đây bằng 1/2 ngưỡng đạt (1 cm, 2°).
    pos_tol: float = 0.005  # m
    heading_tol: float = float(np.deg2rad(1.0))  # rad


class ControlOutput(NamedTuple):
    v: float  # lệnh sau clamp -> gửi xuống xe
    omega: float
    status: str  # REGULATE | ALIGN | DONE | OUT_OF_DOMAIN
    rho: float
    alpha: float
    beta: float
    v_raw: float  # lệnh trước clamp, để vẽ đồ thị
    omega_raw: float


def clamp_command(v, omega, v_max, omega_max):
    """Giảm v và ω CÙNG một hệ số -> tỉ v/ω giữ nguyên -> đường cua giữ nguyên, chỉ chậm hơn."""
    s = 1.0
    if v != 0:
        s = min(s, v_max / abs(v))
    if omega != 0:
        s = min(s, omega_max / abs(omega))
    return s * v, s * omega


def controller_step(pose_G_in_A, cfg=ControllerConfig()):
    """Một phản xạ Astolfi: đích trong hệ A HIỆN TẠI -> lệnh (v, ω). Không nhớ gì giữa các lần gọi."""
    x_G, y_G, theta_G = pose_G_in_A
    rho = np.hypot(x_G, y_G)  # còn cách bao xa
    alpha = np.arctan2(y_G, x_G)  # đích lệch mũi bao nhiêu
    beta = wrap_angle(theta_G - alpha)  # lao thẳng tới đó thì tới nơi còn lệch hướng bao nhiêu
    heading_err = wrap_angle(theta_G)  # thân còn phải quay thêm bao nhiêu

    if rho > cfg.pos_tol and abs(alpha) > np.pi / 2:
        status, v_raw, omega_raw = "OUT_OF_DOMAIN", 0.0, 0.0  # đích ở nửa sau xe, bản này chỉ tiến
    elif rho > cfg.pos_tol:
        status = "REGULATE"
        v_raw = cfg.k_rho * rho
        omega_raw = cfg.k_alpha * alpha + cfg.k_beta * beta
    elif abs(heading_err) > cfg.heading_tol:
        status, v_raw = "ALIGN", 0.0  # đã tới chỗ: xoay tại chỗ theo θ_G, không dùng α, β (vô định)
        omega_raw = cfg.k_heading * heading_err
    else:
        status, v_raw, omega_raw = "DONE", 0.0, 0.0

    v, omega = clamp_command(v_raw, omega_raw, cfg.v_max, cfg.omega_max)
    return ControlOutput(v, omega, status, rho, alpha, beta, v_raw, omega_raw)


# ======== Chấm điểm ========


def dock_errors(pose_M, pose_F, D):
    """Sai số đầu male so với miệng female (M, F cùng một hệ).

    e_lateral: lệch ngang khỏi trục female (m); dương = bên trái khi đứng ở female nhìn ra
    e_gap:     khe dọc trục trừ D (m); 0 = đúng khe D, dương = còn xa hơn D
    e_angle:   trục male lệch so với hướng chĩa thẳng vào female (rad)
    """
    phi_F = pose_F[2]
    n = np.array([np.cos(phi_F), np.sin(phi_F)])  # dọc trục female
    s = np.array([-np.sin(phi_F), np.cos(phi_F)])  # ngang trục female
    r = pose_M[:2] - pose_F[:2]
    return r @ s, r @ n - D, wrap_angle(pose_M[2] - phi_F - np.pi)
