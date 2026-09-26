# Mô phỏng docking 2D. Chạy trong thư mục sim2d:
#   python3 dock_sim.py                  Mốc 1: cảnh tĩnh
#   python3 dock_sim.py kinematics       Mốc 2: xe A chạy lệnh hằng (animation)
#   python3 dock_sim.py dock             Mốc 3: xe A tự vào standby (animation + đồ thị)
# Cờ:  --no-anim   chỉ vẽ trạng thái cuối
#      --no-show   không mở cửa sổ, chỉ lưu PNG vào out/

import argparse
import os
import sys
from typing import NamedTuple

import matplotlib

if "--no-show" in sys.argv:
    matplotlib.use("Agg")  # không cần màn hình
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrow, Polygon

from dock_math import (
    ControllerConfig,
    ControlOutput,
    compose,
    compute_standby,
    controller_step,
    dock_errors,
    estimate_female,
    integrate,
    inverse,
    pose,
    transform_points,
    virtual_camera,
)

d = np.deg2rad
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# ======== Cấu hình (số TẠM theo hình vẽ tay 24/09/2026 — đo lại khi có CAD) ========
# Hệ A: gốc = tâm quay (điểm giữa trục 2 bánh), x trước, y trái.
# Hệ T: gốc = tâm tag, x = pháp tuyến tag hướng ra phía A tới, y trái.

D = 0.10  # m, khe male–female ở standby
POSE_M_IN_A = pose(0.13, 0.00, 0)  # male trên trục giữa, đầu male cách tâm quay 13 cm
POSE_C_IN_A = pose(0.08, -0.05, 0)  # camera trên mặt trước, lệch PHẢI 5 cm, nhìn thẳng
POSE_F_IN_T = pose(
    0.02, -0.05, 0
)  # đứng ở tag nhìn ra: miệng female nhô trước 2 cm, lệch PHẢI 5 cm

# Cảnh: female ở gốc W, miệng nhìn về -x -> trục dock nằm ngang, số dễ đọc.
# Xoay/dời cả cảnh không đổi kết quả (test G04).
POSE_F_IN_W = pose(0.00, 0.00, np.pi)
POSE_T_IN_W = compose(POSE_F_IN_W, inverse(POSE_F_IN_T))  # chỗ dán tag suy từ female
POSE_A_START_IN_W = pose(-1.00, 0.25, d(-15))  # A lúc bắt đầu: lệch trục 25 cm, lệch hướng 15°

# ======== Kích thước chỉ để VẼ (toán không dùng) ========
A_BACK, A_FRONT, A_HALF_WIDTH = -0.08, 0.08, 0.10  # thân A trong hệ A; camera nằm trên mặt trước
B_LENGTH, B_HALF_WIDTH = 0.16, 0.10  # thân B: mặt trước chứa tag, trục giữa đi qua female
WHEEL_RADIUS, WHEEL_WIDTH = 0.035, 0.025
MALE_WIDTH, FEMALE_WIDTH = 0.03, 0.05
TAG_SIZE = 0.034  # tag đang in; thử 0.10 để xem tag to có đè lên female không
CAMERA_HFOV = d(85)  # góc nhìn ngang (calibdatanew.npz)

# ======== Mốc 2: lệnh hằng ========
DT = 0.02  # s, bước tích phân
RENDER_EVERY = 3  # animation vẽ 1 hình mỗi 3 bước DT (~17 hình/giây), độc lập với DT
KINEMATICS_DURATION = 4.0  # s: vừa đủ nửa vòng khi ω = 45°/s
KINEMATICS_CASES = [  # (tên, v m/s, ω rad/s)
    ("K01 đi thẳng", 0.10, 0.0),
    ("K02 quay tại chỗ", 0.0, np.pi / 4),
    ("K03 cung tròn trái", 0.10, np.pi / 4),
]

# ======== Mốc 3: vòng lặp docking ========
MAX_TIME = 30.0  # s, quá thì TIMEOUT
SUCCESS_TOL = (
    0.01,
    0.01,
    d(2),
)  # (lệch ngang m, sai khe m, lệch góc rad): ngưỡng minh hoạ, chưa phải dung sai ren


class Geometry(NamedTuple):
    """Hình học cố định (đo thước / CAD), camera ảo và controller cùng dùng."""

    C_in_A: np.ndarray  # camera trên A
    F_in_T: np.ndarray  # miệng female so với tag
    M_in_A: np.ndarray  # đầu male trên A
    D: float  # khe ở standby


SCENE_GEOMETRY = Geometry(POSE_C_IN_A, POSE_F_IN_T, POSE_M_IN_A, D)

# Kiểu vẽ đơn giản: xe chỉ vẽ viền, không tô màu.
CAR, GHOST = "k", "0.6"  # xe thật: viền đen; xe ở standby: viền xám nét đứt
START, TARGET = "r", "g"  # mũi tên: đỏ = pose xuất phát, xanh = standby (đích)
CAR_LW = 2.0  # độ đậm viền xe


def fmt(p):
    return f"({p[0]:+.3f}, {p[1]:+.3f}, {np.rad2deg(p[2]):+.1f}°)"


# ======== Vẽ ========


def rect(x0, x1, y0, y1):
    """4 góc hình chữ nhật."""
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def add_shape(ax, pose_X_in_W, pts_in_X, color, ls="-", lw=CAR_LW):
    """Vẽ viền đa giác cho trong hệ X lên W. Trả về patch để animation dời được."""
    patch = Polygon(
        transform_points(pose_X_in_W, pts_in_X), closed=True, fill=False, ec=color, lw=lw, ls=ls
    )
    return ax.add_patch(patch)


def add_arrow(ax, pose_X_in_W, length, color):
    """Mũi tên theo hướng pose."""
    x, y, th = pose_X_in_W
    arrow = FancyArrow(
        x,
        y,
        length * np.cos(th),
        length * np.sin(th),
        width=0.006,
        head_width=0.025,
        head_length=0.03,
        length_includes_head=True,
        color=color,
    )
    return ax.add_patch(arrow)


class CarA:
    """Xe A vẽ viền: thân, 2 bánh, male, camera. ghost = nét đứt xám (xe ở standby).

    Tạo một lần, gọi update(pose) để dời -> dùng chung cho hình tĩnh và animation.
    """

    def __init__(self, ax, pose_A_in_W, ghost=False):
        color, ls = (GHOST, "--") if ghost else (CAR, "-")
        x_tip, y_male = POSE_M_IN_A[0], POSE_M_IN_A[1]
        hw = A_HALF_WIDTH
        self.shapes = [
            rect(A_BACK, A_FRONT, -hw, hw),  # thân
            rect(-WHEEL_RADIUS, WHEEL_RADIUS, hw, hw + WHEEL_WIDTH),  # bánh trái
            rect(-WHEEL_RADIUS, WHEEL_RADIUS, -hw, -hw - WHEEL_WIDTH),  # bánh phải
            rect(A_FRONT, x_tip, y_male - MALE_WIDTH / 2, y_male + MALE_WIDTH / 2),  # male
        ]
        self.patches = [add_shape(ax, pose_A_in_W, pts, color, ls) for pts in self.shapes]
        (self.camera,) = ax.plot([], [], "s", ms=5, mew=CAR_LW, mfc="none", color=color)
        self.artists = [*self.patches, self.camera]
        self.update(pose_A_in_W)

    def update(self, pose_A_in_W):
        """Dời xe tới pose mới. Trả về (đầu male, camera) trong W."""
        for patch, pts in zip(self.patches, self.shapes):
            patch.set_xy(transform_points(pose_A_in_W, pts))
        tip = compose(pose_A_in_W, POSE_M_IN_A)
        cam = compose(pose_A_in_W, POSE_C_IN_A)
        self.camera.set_data([cam[0]], [cam[1]])
        return tip, cam


def draw_car_B(ax):
    """Xe B đứng yên: thân + 2 bánh + female + tag, vẽ trong hệ tag."""
    x_mouth, y_female = POSE_F_IN_T[0], POSE_F_IN_T[1]
    add_shape(
        ax, POSE_T_IN_W, rect(-B_LENGTH, 0, y_female - B_HALF_WIDTH, y_female + B_HALF_WIDTH), CAR
    )
    for side in (1, -1):
        y0 = y_female + side * B_HALF_WIDTH
        x_axle = -B_LENGTH / 2
        add_shape(
            ax,
            POSE_T_IN_W,
            rect(x_axle - WHEEL_RADIUS, x_axle + WHEEL_RADIUS, y0, y0 + side * WHEEL_WIDTH),
            CAR,
        )
    add_shape(
        ax,
        POSE_T_IN_W,
        rect(0, x_mouth, y_female - FEMALE_WIDTH / 2, y_female + FEMALE_WIDTH / 2),
        CAR,
    )
    tag = transform_points(POSE_T_IN_W, [[0, -TAG_SIZE / 2], [0, TAG_SIZE / 2]])
    ax.plot(tag[:, 0], tag[:, 1], "k", lw=5, solid_capstyle="butt")


def draw_female_axis(ax):
    axis_end = compose(POSE_F_IN_W, pose(1.30, 0, 0))
    ax.plot([POSE_F_IN_W[0], axis_end[0]], [POSE_F_IN_W[1], axis_end[1]], "-.", color=GHOST, lw=0.8)
    return axis_end


def note(ax, lines):
    """Dòng chữ ở góc trên trái."""
    return ax.text(0.01, 0.97, "\n".join(lines), transform=ax.transAxes, va="top", fontsize=8)


# ======== Mốc 1: cảnh tĩnh ========


def draw_scene(G_in_W, G_true_in_W):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.set_aspect("equal")
    draw_female_axis(ax)
    draw_car_B(ax)
    male, camera = CarA(ax, POSE_A_START_IN_W).update(POSE_A_START_IN_W)
    male_ghost, camera_ghost = CarA(ax, G_in_W, ghost=True).update(G_in_W)
    add_arrow(ax, POSE_A_START_IN_W, 0.12, START)
    add_arrow(ax, G_in_W, 0.12, TARGET)
    ax.plot(*G_true_in_W[:2], "kx")  # standby thật, phải trùng mũi tên xanh

    # camera: góc nhìn 85° + tia tới tag
    for side in (1, -1):
        edge = compose(
            camera, pose(0.30 * np.cos(CAMERA_HFOV / 2), side * 0.30 * np.sin(CAMERA_HFOV / 2), 0)
        )
        ax.plot([camera[0], edge[0]], [camera[1], edge[1]], "--", color=GHOST, lw=0.8)
    for cam in (camera, camera_ghost):
        ax.plot([cam[0], POSE_T_IN_W[0]], [cam[1], POSE_T_IN_W[1]], ":", color=GHOST, lw=0.8)

    # khe D giữa đầu male (xe ở standby) và miệng female
    y_gap = POSE_F_IN_W[1] + 0.07
    ax.annotate(
        "",
        xy=(POSE_F_IN_W[0], y_gap),
        xytext=(male_ghost[0], y_gap),
        arrowprops=dict(arrowstyle="<->", lw=0.8, shrinkA=0, shrinkB=0),
    )
    ax.text(
        (POSE_F_IN_W[0] + male_ghost[0]) / 2, y_gap + 0.01, f"D = {D:.2f}", ha="center", fontsize=8
    )

    for text, xy in [
        ("A", POSE_A_START_IN_W),
        ("camera", camera),
        ("standby", G_in_W),
        ("tag", POSE_T_IN_W),
        ("B", compose(POSE_T_IN_W, pose(-B_LENGTH / 2, 0, 0))),
    ]:
        ax.annotate(text, xy=xy[:2], xytext=(5, -12), textcoords="offset points", fontsize=8)
    note(ax, [f"Start: {fmt(POSE_A_START_IN_W)}", f"Standby: {fmt(G_in_W)}"])
    fig.tight_layout()
    return fig


def run_static_scene():
    # Phía controller: chỉ dùng tag camera thấy + hình học khai báo
    T_in_C = virtual_camera(POSE_A_START_IN_W, POSE_C_IN_A, POSE_T_IN_W)
    F_in_A = estimate_female(T_in_C, POSE_C_IN_A, POSE_F_IN_T)
    G_in_A = compute_standby(F_in_A, POSE_M_IN_A, D)

    # Phía chấm điểm: được dùng pose thật (chỉ để vẽ/in, không đưa ngược vào controller)
    G_in_W = compose(POSE_A_START_IN_W, G_in_A)
    G_true_in_W = compute_standby(POSE_F_IN_W, POSE_M_IN_A, D)
    e_lat, e_gap, e_ang = dock_errors(compose(G_in_W, POSE_M_IN_A), POSE_F_IN_W, D)

    print("Cảnh tĩnh (số tạm theo hình vẽ tay):")
    print(f"  Tag trong camera lúc bắt đầu    {fmt(T_in_C)}   <- controller chỉ có số này")
    print(f"  Standby controller tính (hệ A)  {fmt(G_in_A)}")
    print(f"  Standby đó đổi sang W           {fmt(G_in_W)}")
    print(f"  Standby thật trong W            {fmt(G_true_in_W)}")
    print(f"  Lệch giữa hai cái               {np.hypot(*(G_in_W - G_true_in_W)[:2]):.1e} m")
    print(
        f"  Tại standby, camera thấy tag ở  {fmt(virtual_camera(G_in_W, POSE_C_IN_A, POSE_T_IN_W))}"
    )
    print(
        f"  Sai số dock tại standby         lệch ngang {e_lat * 1000:+.1f} mm, "
        f"sai khe {e_gap * 1000:+.1f} mm, lệch góc {np.rad2deg(e_ang):+.2f}°"
    )
    return draw_scene(G_in_W, G_true_in_W)


# ======== Mốc 2: lệnh hằng ========


def run_constant(start, v, omega, dt, duration):
    """Chạy lệnh hằng -> (thời gian, mảng pose), mỗi dòng một bước dt."""
    poses = [start]
    for _ in range(round(duration / dt)):
        poses.append(integrate(poses[-1], v, omega, dt))
    return np.arange(len(poses)) * dt, np.array(poses)


def exact_end(v, omega, duration):
    """Pose cuối CHÍNH XÁC khi chạy lệnh hằng từ (0, 0, 0): công thức hình tròn, không qua Euler."""
    if omega == 0:
        return pose(v * duration, 0, 0)
    R = v / omega
    return pose(R * np.sin(omega * duration), R * (1 - np.cos(omega * duration)), omega * duration)


def build_kinematics_figure():
    """3 ô, mỗi ô một lệnh hằng. Trả về (fig, update(frame), số frame)."""
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    panels = []
    print(
        f"Lệnh hằng trong {KINEMATICS_DURATION:.0f} s, dt = {DT} s (Euler) so với công thức chính xác:"
    )
    for ax, (name, v, omega) in zip(axes, KINEMATICS_CASES):
        times, poses = run_constant(pose(0, 0, 0), v, omega, DT, KINEMATICS_DURATION)
        tips = np.array([compose(p, POSE_M_IN_A)[:2] for p in poses])
        end = exact_end(v, omega, KINEMATICS_DURATION)
        print(
            f"  {name:<20} cuối {fmt(poses[-1])}   chính xác {fmt(end)}   "
            f"lệch {np.hypot(*(poses[-1] - end)[:2]) * 1000:.1f} mm"
        )

        ax.set_aspect("equal")
        ax.set_xlim(-0.25, 0.60)
        ax.set_ylim(-0.25, 0.45)
        ax.set_title(f"{name}: v = {v:.2f} m/s, w = {np.rad2deg(omega):.0f} deg/s", fontsize=9)
        add_arrow(ax, end, 0.10, TARGET)  # pose cuối theo công thức chính xác
        (center_trail,) = ax.plot([], [], "b--", lw=1)
        (tip_trail,) = ax.plot([], [], ":", color=GHOST, lw=1)
        car = CarA(ax, poses[0])
        info = note(ax, [""])
        panels.append((times, poses, tips, center_trail, tip_trail, car, info))

    n_steps = len(panels[0][0]) - 1
    n_frames = -(-n_steps // RENDER_EVERY) + 1  # frame cuối luôn là bước cuối

    def update(frame):
        """Vẽ frame thứ `frame`. Trả về các hình đã đổi (để animation chỉ vẽ lại chúng -> nhanh)."""
        k = min(frame * RENDER_EVERY, n_steps)
        changed = []
        for times, poses, tips, center_trail, tip_trail, car, info in panels:
            center_trail.set_data(poses[: k + 1, 0], poses[: k + 1, 1])
            tip_trail.set_data(tips[: k + 1, 0], tips[: k + 1, 1])
            car.update(poses[k])
            info.set_text(f"Time: {times[k]:.2f}\nTheta: {np.rad2deg(poses[k, 2]):.0f} deg")
            changed += [center_trail, tip_trail, *car.artists, info]
        return changed

    fig.tight_layout()
    return fig, update, n_frames


# ======== Mốc 3: xe A tự vào standby ========


class DockRun(NamedTuple):
    log: dict  # mảng theo thời gian: t, A_in_W, G_in_A + mọi trường của ControlOutput
    status: str  # DONE | OUT_OF_DOMAIN | TIMEOUT
    errors: tuple  # (lệch ngang m, sai khe m, lệch góc rad) THẬT ở pose cuối


def simulate_docking(
    A_start_in_W,
    T_in_W=POSE_T_IN_W,
    geom=SCENE_GEOMETRY,
    cfg=ControllerConfig(),
    dt=DT,
    max_time=MAX_TIME,
):
    """Vòng lặp plan §6: camera ảo -> female -> standby -> controller -> xe chạy.

    Pose thật của A chỉ dùng để sinh số đo camera, cho xe chạy và chấm điểm.
    Controller chỉ nhận đích trong hệ A, tính từ tag camera thấy.
    """
    rows = []
    A_in_W = A_start_in_W
    n_max = round(max_time / dt)
    for k in range(n_max + 1):
        T_in_C = virtual_camera(A_in_W, geom.C_in_A, T_in_W)  # thế giới thật -> số đo camera
        F_in_A = estimate_female(T_in_C, geom.C_in_A, geom.F_in_T)  # từ đây: chỉ số đo + hình học
        G_in_A = compute_standby(F_in_A, geom.M_in_A, geom.D)
        out = controller_step(G_in_A, cfg)
        rows.append((k * dt, A_in_W, G_in_A, out))
        if out.status in ("DONE", "OUT_OF_DOMAIN") or k == n_max:
            break
        A_in_W = integrate(A_in_W, out.v, out.omega, dt)
    status = out.status if out.status in ("DONE", "OUT_OF_DOMAIN") else "TIMEOUT"

    log = {
        "t": np.array([row[0] for row in rows]),
        "A_in_W": np.array([row[1] for row in rows]),
        "G_in_A": np.array([row[2] for row in rows]),
    }
    for name in ControlOutput._fields:
        log[name] = np.array([getattr(row[3], name) for row in rows])

    # chấm điểm bằng hình học THẬT (controller không thấy phần này)
    F_in_W = compose(T_in_W, geom.F_in_T)
    errors = dock_errors(compose(A_in_W, geom.M_in_A), F_in_W, geom.D)
    return DockRun(log, status, errors)


def docked(run):
    """DONE và sai số THẬT ở đầu male nằm trong SUCCESS_TOL."""
    return run.status == "DONE" and all(abs(e) < tol for e, tol in zip(run.errors, SUCCESS_TOL))


def print_dock_summary(run, title):
    log = run.log
    e_lat, e_gap, e_ang = run.errors
    clamped = np.mean(
        (np.abs(log["v"] - log["v_raw"]) + np.abs(log["omega"] - log["omega_raw"])) > 1e-12
    )
    print(f"{title}:")
    print(f"  Kết thúc {run.status} sau {log['t'][-1]:.2f} s ({len(log['t']) - 1} bước)")
    print(
        f"  Sai số THẬT ở đầu male: lệch ngang {e_lat * 1000:+.1f} mm, "
        f"sai khe {e_gap * 1000:+.1f} mm, lệch góc {np.rad2deg(e_ang):+.2f}°"
    )
    print(f"  Đạt ngưỡng (1 cm, 1 cm, 2°): {'CÓ' if docked(run) else 'KHÔNG'}")
    print(
        f"  |v| lớn nhất {np.abs(log['v']).max():.3f} m/s, |ω| lớn nhất {np.abs(log['omega']).max():.2f} rad/s, "
        f"{clamped:.0%} số bước bị clamp"
    )
    t_align = log["t"][log["status"] == "ALIGN"]
    if len(t_align):
        print(f"  Xoay tại chỗ (ALIGN) từ {t_align[0]:.2f} s tới {t_align[-1]:.2f} s")


def build_dock_figure(run):
    """Mặt bằng: xe A chạy theo log, để lại vết. Trả về (fig, update(frame), số frame)."""
    log = run.log
    A = log["A_in_W"]
    tips = np.array([compose(p, POSE_M_IN_A)[:2] for p in A])
    cams = np.array([compose(p, POSE_C_IN_A)[:2] for p in A])
    G_true_in_W = compute_standby(POSE_F_IN_W, POSE_M_IN_A, D)

    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.set_aspect("equal")
    axis_end = draw_female_axis(ax)
    draw_car_B(ax)
    CarA(ax, G_true_in_W, ghost=True)
    add_arrow(ax, G_true_in_W, 0.12, TARGET)
    add_arrow(ax, A[0], 0.12, START)
    (center_trail,) = ax.plot([], [], "b--", lw=1)
    (tip_trail,) = ax.plot([], [], ":", color=GHOST, lw=1)
    (ray,) = ax.plot([], [], ":", color=GHOST, lw=0.8)
    car = CarA(ax, A[0])
    info = note(ax, [""])

    xs = np.concatenate([A[:, 0], tips[:, 0], [axis_end[0], B_LENGTH]])
    ys = np.concatenate([A[:, 1], tips[:, 1], [-B_HALF_WIDTH, B_HALF_WIDTH]])
    ax.set_xlim(xs.min() - 0.08, xs.max() + 0.08)
    ax.set_ylim(ys.min() - 0.17, ys.max() + 0.30)  # chừa chỗ cho chữ ở góc trên
    fig.tight_layout()

    n_steps = len(A) - 1
    n_frames = -(-n_steps // RENDER_EVERY) + 1

    def update(frame):
        k = min(frame * RENDER_EVERY, n_steps)
        center_trail.set_data(A[: k + 1, 0], A[: k + 1, 1])
        tip_trail.set_data(tips[: k + 1, 0], tips[: k + 1, 1])
        ray.set_data([cams[k, 0], POSE_T_IN_W[0]], [cams[k, 1], POSE_T_IN_W[1]])
        car.update(A[k])
        info.set_text(
            f"Time: {log['t'][k]:.2f}\n"
            f"Status: {log['status'][k]}\n"
            f"rho = {log['rho'][k]:.3f} m, alpha = {np.rad2deg(log['alpha'][k]):.1f} deg, "
            f"beta = {np.rad2deg(log['beta'][k]):.1f} deg\n"
            f"v = {log['v'][k]:.3f} m/s, w = {log['omega'][k]:.3f} rad/s"
        )
        return [center_trail, tip_trail, ray, *car.artists, info]

    return fig, update, n_frames


def build_dock_plots(run, cfg=ControllerConfig()):
    """4 đồ thị theo thời gian (plan §7.3). Mỗi ô một đơn vị."""
    log = run.log
    t = log["t"]
    regulate = (
        log["status"] == "REGULATE"
    )  # α, β chỉ có nghĩa khi còn chạy Astolfi; sát đích để trống
    fig, (ax_rho, ax_ang, ax_v, ax_w) = plt.subplots(4, 1, figsize=(8, 8), sharex=True)

    ax_rho.semilogy(t, log["rho"], label="rho")
    ax_rho.axhline(cfg.pos_tol, color="k", ls=":", lw=0.8, label="pos_tol")
    ax_rho.set_ylabel("rho (m)")

    ax_ang.plot(t, np.where(regulate, np.rad2deg(log["alpha"]), np.nan), label="alpha")
    ax_ang.plot(t, np.where(regulate, np.rad2deg(log["beta"]), np.nan), label="beta")
    ax_ang.plot(t, np.rad2deg(log["G_in_A"][:, 2]), label="theta_G")
    ax_ang.set_ylabel("angle (deg)")

    ax_v.plot(t, log["v_raw"], "--", label="v raw")
    ax_v.plot(t, log["v"], label="v")
    ax_v.set_ylabel("v (m/s)")

    ax_w.plot(t, log["omega_raw"], "--", label="w raw")
    ax_w.plot(t, log["omega"], label="w")
    ax_w.set_ylabel("w (rad/s)")
    ax_w.set_xlabel("time (s)")

    for ax in (ax_rho, ax_ang, ax_v, ax_w):
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    return fig


# ======== Chạy ========


def animate(fig, update, n_frames):
    """blit: chỉ vẽ lại phần đổi (xe, vết, chữ) -> chạy kịp thời gian thật."""
    return FuncAnimation(
        fig, update, frames=n_frames, interval=RENDER_EVERY * DT * 1000, repeat=False, blit=True
    )


def save(fig, name):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    print(f"[PLOT] Lưu {path}")


def main():
    parser = argparse.ArgumentParser(description="Mô phỏng docking 2D")
    parser.add_argument(
        "scene", nargs="?", default="static", choices=["static", "kinematics", "dock"]
    )
    parser.add_argument("--no-anim", action="store_true", help="chỉ vẽ trạng thái cuối")
    parser.add_argument("--no-show", action="store_true", help="không mở cửa sổ, chỉ lưu PNG")
    args = parser.parse_args()
    use_anim = not (args.no_anim or args.no_show)

    anim = None  # phải giữ biến này tới lúc plt.show(), không thì animation bị xoá
    if args.scene == "static":
        save(run_static_scene(), "scene_static.png")
    elif args.scene == "kinematics":
        fig, update, n_frames = build_kinematics_figure()
        update(n_frames - 1)  # ảnh lưu = trạng thái cuối, đủ vết
        save(fig, "kinematics.png")
        anim = animate(fig, update, n_frames) if use_anim else None
    else:
        run = simulate_docking(POSE_A_START_IN_W)
        print_dock_summary(
            run, f"Docking — cảnh theo hình vẽ tay, xuất phát {fmt(POSE_A_START_IN_W)}"
        )
        fig, update, n_frames = build_dock_figure(run)
        update(n_frames - 1)
        save(fig, "dock_scene.png")
        save(build_dock_plots(run), "dock_plots.png")
        anim = animate(fig, update, n_frames) if use_anim else None
    if not args.no_show:
        plt.show()
    return anim


if __name__ == "__main__":
    main()
