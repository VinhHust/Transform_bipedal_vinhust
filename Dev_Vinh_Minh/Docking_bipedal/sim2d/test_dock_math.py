# Test toán docking 2D. Chạy: cd sim2d && python3 -m unittest -v
# Số mong đợi lấy từ TÍNH TAY (docs/PLAN_DOCKING_2D_PYTHON.md §8.3), không lấy từ code.

import unittest

import numpy as np

from dock_math import (
    ControllerConfig,
    clamp_command,
    compose,
    controller_step,
    compute_standby,
    dock_errors,
    estimate_female,
    integrate,
    inverse,
    pose,
    transform_points,
    virtual_camera,
    wrap_angle,
)
import dock_sim  # simulator (vòng lặp) cho các test C; kéo theo matplotlib nhưng không mở cửa sổ

TOL = 1e-9  # m hoặc rad; hình học lý tưởng nên sai số chỉ còn làm tròn float
d = np.deg2rad


def fmt(p):
    return f"({p[0]:+.4f}, {p[1]:+.4f}, {np.rad2deg(p[2]):+.2f}°)"


class PoseTestCase(unittest.TestCase):
    def assertPoseClose(self, actual, expected, tol=TOL):
        """So x, y trực tiếp; so góc bằng hiệu đã wrap (-180° và 180° là cùng một hướng)."""
        msg = f"ra {fmt(actual)}, mong đợi {fmt(expected)}"
        self.assertAlmostEqual(actual[0], expected[0], delta=tol, msg=msg)
        self.assertAlmostEqual(actual[1], expected[1], delta=tol, msg=msg)
        self.assertAlmostEqual(
            wrap_angle(actual[2] - expected[2]), 0.0, delta=tol, msg=msg
        )

    def assertErrorsClose(self, actual, expected):
        """So bộ (lệch ngang, sai khe, lệch góc) của dock_errors."""
        for name, a, e in zip(("lệch ngang", "sai khe", "lệch góc"), actual, expected):
            self.assertAlmostEqual(a, e, delta=TOL, msg=f"{name}: ra {a}, mong đợi {e}")


# ======== wrap_angle ========


class TestWrapAngle(unittest.TestCase):
    def test_hand_examples(self):
        """190° -> -170°, -190° -> 170°, 180° -> -180°, 360° -> 0°."""
        for deg, expected_deg in [(0, 0), (190, -170), (-190, 170), (180, -180), (360, 0)]:
            with self.subTest(deg=deg):
                self.assertAlmostEqual(wrap_angle(d(deg)), d(expected_deg), delta=TOL)

    def test_range_and_direction(self):
        """Mọi góc từ -20 đến 20 rad: kết quả trong [-pi, pi] và vẫn chỉ cùng hướng."""
        for a in np.linspace(-20, 20, 401):
            w = wrap_angle(a)
            # làm tròn float có thể ra đúng +pi (cùng hướng với -pi) nên cho phép cả hai đầu
            self.assertTrue(-np.pi <= w <= np.pi)
            # cùng hướng <=> cùng sin và cos
            self.assertAlmostEqual(np.sin(w), np.sin(a), delta=TOL)
            self.assertAlmostEqual(np.cos(w), np.cos(a), delta=TOL)


# ======== compose / inverse ========


class TestComposeInverse(PoseTestCase):
    def test_compose_car_facing_up(self):
        """Xe (1, 0, 90°), camera trước tâm bánh 10 cm -> camera ở (1, 0.1, 90°)."""
        self.assertPoseClose(
            compose(pose(1, 0, d(90)), pose(0.1, 0, 0)), pose(1, 0.1, d(90))
        )

    def test_inverse_car_facing_up(self):
        """Xe (1, 0, 90°) -> gốc W nằm bên TRÁI xe 1 m: W trong A = (0, 1, -90°)."""
        self.assertPoseClose(inverse(pose(1, 0, d(90))), pose(0, 1, d(-90)))

    def test_inverse_undoes_compose(self):
        """Ghép với nghịch đảo ra 0. Chỉ là kiểm tra phụ: hai hàm cùng sai vẫn có thể pass."""
        P = pose(0.3, -0.2, d(40))
        self.assertPoseClose(compose(P, inverse(P)), pose(0, 0, 0))
        self.assertPoseClose(compose(inverse(P), P), pose(0, 0, 0))

    def test_transform_points(self):
        """Xe (1, 0, 90°): điểm 10 cm phía trước -> (1, 0.1); điểm 10 cm bên TRÁI -> (0.9, 0)."""
        pts = transform_points(pose(1, 0, d(90)), [[0.1, 0.0], [0.0, 0.1]])
        np.testing.assert_allclose(pts, [[1.0, 0.1], [0.9, 0.0]], atol=TOL)

    def test_inputs_not_modified(self):
        """Hàm trả mảng mới, không sửa mảng đầu vào."""
        P, Q = pose(0.3, -0.2, d(40)), pose(0.1, 0.05, d(-70))
        P0, Q0 = P.copy(), Q.copy()
        compose(P, Q)
        inverse(P)
        np.testing.assert_array_equal(P, P0)
        np.testing.assert_array_equal(Q, Q0)


# ======== G01: fixture 1 (plan §8.3) ========
# Chỉ là số để lập trình, không phải kích thước xe thật.

A_IN_W = pose(-1.20, 0.00, 0)
T_IN_W = pose(0.00, -0.08, np.pi)
C_IN_A = pose(0.10, -0.05, 0)
F_IN_T = pose(0.00, -0.08, 0)
M_IN_A = pose(0.20, 0.00, 0)
D = 0.10


class TestFixture1(PoseTestCase):
    def test_G01_1_camera_in_W(self):
        """Camera trong W = (-1.10, -0.05, 0°)."""
        self.assertPoseClose(compose(A_IN_W, C_IN_A), pose(-1.10, -0.05, 0))

    def test_G01_2_tag_in_camera(self):
        """Tag trong camera = (1.10, -0.03, -180°)."""
        self.assertPoseClose(
            virtual_camera(A_IN_W, C_IN_A, T_IN_W), pose(1.10, -0.03, -np.pi)
        )

    def test_G01_3_female_in_W(self):
        """Female trong W = (0, 0, 180°)."""
        self.assertPoseClose(compose(T_IN_W, F_IN_T), pose(0.00, 0.00, np.pi))

    def test_G01_4_female_in_A(self):
        """Female trong A = (1.20, 0, 180°), chỉ dùng số đo camera."""
        T_in_C = virtual_camera(A_IN_W, C_IN_A, T_IN_W)
        self.assertPoseClose(
            estimate_female(T_in_C, C_IN_A, F_IN_T), pose(1.20, 0.00, np.pi)
        )

    def test_G01_5_standby_in_W(self):
        """Standby trong W = (-0.30, 0, 0°)."""
        F_in_W = compose(T_IN_W, F_IN_T)
        self.assertPoseClose(compute_standby(F_in_W, M_IN_A, D), pose(-0.30, 0.00, 0))

    def test_G01_6_standby_in_A(self):
        """Standby trong A = (0.90, 0, 0°), chỉ dùng số đo camera."""
        T_in_C = virtual_camera(A_IN_W, C_IN_A, T_IN_W)
        F_in_A = estimate_female(T_in_C, C_IN_A, F_IN_T)
        self.assertPoseClose(compute_standby(F_in_A, M_IN_A, D), pose(0.90, 0.00, 0))

    def test_G01_7_male_tip_at_standby(self):
        """A đứng ở standby (số tính tay) -> đầu male trong W = (-0.10, 0)."""
        M_in_W = compose(pose(-0.30, 0.00, 0), M_IN_A)
        self.assertAlmostEqual(M_in_W[0], -0.10, delta=TOL)
        self.assertAlmostEqual(M_in_W[1], 0.00, delta=TOL)

    def test_G01_8_errors_at_standby(self):
        """A đứng ở standby -> lệch ngang, sai khe, lệch góc đều = 0 (khe thật = D)."""
        M_in_W = compose(pose(-0.30, 0.00, 0), M_IN_A)
        F_in_W = compose(T_IN_W, F_IN_T)
        for e in dock_errors(M_in_W, F_in_W, D):
            self.assertAlmostEqual(e, 0.0, delta=TOL)


# ======== G02: fixture 2 góc lẻ (plan §8.4) — BẠN tính tay rồi điền ========
# Fixture 1 chỉ có góc 0° và 180° nên không thấy lỗi sin/cos. Fixture 2 dùng góc lẻ để bắt các lỗi đó.
# Điền (x, y, góc_độ) vào G02_HAND. Còn None thì test tự skip.

A2_IN_W = pose(-1.00, 0.30, d(15))
C2_IN_A = pose(0.10, -0.05, d(10))
T2_IN_W = pose(0.00, -0.08, d(-150))
F2_IN_T = pose(0.00, -0.08, 0)
M2_IN_A = pose(0.20, 0.00, 0)
D2 = 0.10

G02_HAND = {
    "camera_in_W": None,  # vd: (-0.1234, 0.5678, 25)
    "tag_in_camera": None,
    "female_in_W": None,
    "female_in_A": None,
    "standby_in_W": None,
    "standby_in_A": None,
}
TOL_HAND = 5e-4  # 0.5 mm / 0.03°: đủ rộng cho làm tròn tay, vẫn bắt được lỗi cỡ cm


class TestFixture2(PoseTestCase):
    def test_G02_hand_calculation(self):
        """Fixture 2 góc lẻ khớp số BẠN tính tay."""
        missing = [name for name, value in G02_HAND.items() if value is None]
        if missing:
            self.skipTest(f"chờ số tính tay: {', '.join(missing)}")
        T_in_C = virtual_camera(A2_IN_W, C2_IN_A, T2_IN_W)
        F_in_W = compose(T2_IN_W, F2_IN_T)
        F_in_A = estimate_female(T_in_C, C2_IN_A, F2_IN_T)
        computed = {
            "camera_in_W": compose(A2_IN_W, C2_IN_A),
            "tag_in_camera": T_in_C,
            "female_in_W": F_in_W,
            "female_in_A": F_in_A,
            "standby_in_W": compute_standby(F_in_W, M2_IN_A, D2),
            "standby_in_A": compute_standby(F_in_A, M2_IN_A, D2),
        }
        for name, (x, y, deg) in G02_HAND.items():
            with self.subTest(name):
                self.assertPoseClose(computed[name], pose(x, y, d(deg)), tol=TOL_HAND)


# ======== dock_errors: ví dụ có sai số khác 0 ========
# Toàn số 0 thì không bắt được lỗi dấu -> cần một ví dụ tính tay có số khác 0.


class TestDockErrors(unittest.TestCase):
    def test_hand_example(self):
        """Female (0, 0, 180°), male (-0.13, 0.02, 10°), D = 0.10 -> (-0.02, +0.03, 10°)."""
        # Female nhìn ra hướng -x. Đứng ở female nhìn ra thì +y của W nằm bên PHẢI -> lệch ngang âm.
        # Male cách miệng 0.13 dọc trục, D = 0.10 -> còn dư 0.03.
        e_lat, e_gap, e_ang = dock_errors(pose(-0.13, 0.02, d(10)), pose(0, 0, np.pi), D)
        self.assertAlmostEqual(e_lat, -0.02, delta=TOL)
        self.assertAlmostEqual(e_gap, 0.03, delta=TOL)
        self.assertAlmostEqual(e_ang, d(10), delta=TOL)


# ======== compute_standby: kiểm theo định nghĩa, không cần đáp án số ========


class TestStandby(unittest.TestCase):
    def test_male_lands_at_gap_D(self):
        """Đứng ở standby thì male đúng khe D và thẳng trục, kể cả male lắp lệch/nghiêng."""
        cases = [
            (pose(1.20, 0.00, np.pi), pose(0.20, 0.00, 0)),  # fixture 1
            (pose(0.80, -0.50, d(-165)), pose(0.20, 0.00, 0)),  # female nghiêng
            (pose(0.50, 0.30, d(120)), pose(0.15, 0.04, d(10))),  # male lệch ngang + nghiêng
        ]
        for F, M_in_A in cases:
            with self.subTest(F=fmt(F), M=fmt(M_in_A)):
                G = compute_standby(F, M_in_A, D)
                for e in dock_errors(compose(G, M_in_A), F, D):
                    self.assertAlmostEqual(e, 0.0, delta=TOL)


# ======== G03–G05: bất biến — không cần đáp án số ========

F_IN_W = compose(T_IN_W, F_IN_T)  # female của fixture 1 = (0, 0, 180°)
A_OFF_AXIS_IN_W = pose(-1.00, 0.30, d(15))  # xe A đứng lệch, chưa thẳng trục

# (camera trên A, female so với tag). Cả 3 đều là cách lắp hợp lệ, miễn khai báo đúng.
LAYOUTS = {
    "theo hình vẽ (cam phải, tag phải)": (pose(0.08, -0.05, 0), pose(0.02, -0.05, 0)),
    "cam + tag nằm trên trục": (pose(0.08, 0.00, 0), pose(0.02, 0.00, 0)),
    "cam trái, tag lệch xa, lắp nghiêng": (pose(0.05, 0.07, d(8)), pose(0.03, 0.12, d(-5))),
}


def standby_seen_by_controller(A_in_W, T_in_W, C_in_A=C_IN_A, F_in_T=F_IN_T):
    """Chuỗi phía controller: camera thấy tag -> female trong A -> standby trong A."""
    T_in_C = virtual_camera(A_in_W, C_in_A, T_in_W)
    return compute_standby(estimate_female(T_in_C, C_in_A, F_in_T), M_IN_A, D)


class TestInvariance(PoseTestCase):
    def test_G03_camera_tag_placement(self):
        """Lắp camera/tag ở đâu cũng được (khai báo đúng) -> controller ra CÙNG một standby."""
        G_true_in_W = compute_standby(F_IN_W, M_IN_A, D)
        for A_in_W in [A_IN_W, A_OFF_AXIS_IN_W, pose(-0.60, -0.40, d(-40))]:
            for name, (C_in_A, F_in_T) in LAYOUTS.items():
                with self.subTest(A=fmt(A_in_W), layout=name):
                    T_in_W = compose(F_IN_W, inverse(F_in_T))  # đặt tag sao cho female giữ nguyên chỗ
                    G_in_A = standby_seen_by_controller(A_in_W, T_in_W, C_in_A, F_in_T)
                    # đổi sang W bằng pose thật của A chỉ để chấm điểm, không đưa ngược vào controller
                    self.assertPoseClose(compose(A_in_W, G_in_A), G_true_in_W)

    def test_G04_move_whole_scene(self):
        """Dời + xoay CẢ cảnh (cả A lẫn B) -> đích trong hệ A và sai số dock không đổi."""
        G_ref = standby_seen_by_controller(A_OFF_AXIS_IN_W, T_IN_W)
        M_in_W = compose(A_OFF_AXIS_IN_W, M_IN_A)
        errors_ref = dock_errors(M_in_W, F_IN_W, D)  # khác 0 cả 3 thành phần
        for H in [pose(0.70, -1.30, d(123)), pose(-2.00, 0.50, d(-75)), pose(5.0, 5.0, d(40))]:
            with self.subTest(H=fmt(H)):
                A_moved, T_moved = compose(H, A_OFF_AXIS_IN_W), compose(H, T_IN_W)
                self.assertPoseClose(standby_seen_by_controller(A_moved, T_moved), G_ref)
                errors = dock_errors(compose(H, M_in_W), compose(H, F_IN_W), D)
                self.assertErrorsClose(errors, errors_ref)

    def test_G05_angles_near_180(self):
        """Góc sát ±180°: hai bên ranh giới là cùng một hướng -> không nhảy 360°."""
        # (a) female quay sát ±180°, male thẳng trục: lệch góc ≈ 0 chứ không phải ±360°,
        #     và góc đích nằm trong [-180°, 180°] -> xoay 0.001° chứ không xoay 359.999°
        for phi_deg in [179.999, 180.0, -180.0, -179.999]:
            with self.subTest(phi_F=phi_deg):
                F = pose(0.30, -0.20, d(phi_deg))
                G = compute_standby(F, M_IN_A, D)
                self.assertTrue(-np.pi <= G[2] <= np.pi, f"góc đích = {np.rad2deg(G[2])}°")
                self.assertErrorsClose(dock_errors(compose(G, M_IN_A), F, D), (0, 0, 0))
        # (b) xoay cả cảnh qua ranh giới ±180°: đích trong A không đổi
        G_ref = standby_seen_by_controller(A_OFF_AXIS_IN_W, T_IN_W)
        for rot_deg in [179.999, 180.0, -179.999]:
            with self.subTest(rot=rot_deg):
                H = pose(0, 0, d(rot_deg))
                G = standby_seen_by_controller(compose(H, A_OFF_AXIS_IN_W), compose(H, T_IN_W))
                self.assertPoseClose(G, G_ref)


# ======== K01–K03: động học xe vi sai (plan §8.2) ========


def run_constant(start, v, omega, dt, duration):
    """Chạy lệnh hằng (v, omega) trong `duration` giây, mỗi bước dt."""
    p = start
    for _ in range(round(duration / dt)):
        p = integrate(p, v, omega, dt)
    return p


class TestKinematics(PoseTestCase):
    def test_K01_straight(self):
        """v > 0, ω = 0: đi thẳng theo hướng thân. Hướng 30°, 0.1 m/s trong 1 s -> đi 10 cm theo 30°."""
        p = run_constant(pose(1.0, 2.0, d(30)), 0.10, 0.0, 0.02, 1.0)
        expected = pose(1.0 + 0.10 * np.cos(d(30)), 2.0 + 0.10 * np.sin(d(30)), d(30))
        self.assertPoseClose(p, expected)

    def test_K02_spin_left(self):
        """v = 0, ω > 0: quay TRÁI tại chỗ. 0.5 rad/s trong 1 s -> tâm quay đứng yên, hướng +0.5 rad."""
        p = run_constant(pose(1.0, 2.0, d(30)), 0.0, 0.5, 0.02, 1.0)
        self.assertPoseClose(p, pose(1.0, 2.0, d(30) + 0.5))

    def test_K03_arc_left(self):
        """v > 0, ω > 0: cung tròn sang TRÁI, bán kính v/ω. Nửa vòng -> tới (0, 2R), quay đầu 180°."""
        v, omega = 0.10, np.pi / 4  # R = v/ω ≈ 12.7 cm; nửa vòng mất đúng 4 s
        R = v / omega
        errors = {}
        for dt in (0.02, 0.01):
            p = run_constant(pose(0, 0, 0), v, omega, dt, np.pi / omega)
            errors[dt] = np.hypot(p[0] - 0.0, p[1] - 2 * R)
            with self.subTest(dt=dt):
                # Euler vẽ cung tròn bằng nhiều đoạn thẳng -> sai cỡ v·dt, không phải 0
                self.assertLess(errors[dt], 1.5 * v * dt)
                self.assertAlmostEqual(wrap_angle(p[2] - np.pi), 0.0, delta=TOL)
        # giảm dt một nửa -> sai giảm một nửa (Euler là bậc 1)
        self.assertAlmostEqual(errors[0.02] / errors[0.01], 2.0, delta=0.2)


# ======== Bộ điều khiển: kiểm DẤU bằng từng phản xạ đơn lẻ (plan §9.4 bước 1) ========
# Mọi đích cho trong hệ A: xe ở gốc, mũi hướng +x, bên trái là +y.

CFG = ControllerConfig()


class TestController(unittest.TestCase):
    def test_goal_straight_ahead(self):
        """Đích ngay phía trước, cùng hướng -> chạy thẳng (ω = 0), v bị clamp về v_max."""
        out = controller_step(pose(0.5, 0.0, 0))
        self.assertEqual(out.status, "REGULATE")
        self.assertAlmostEqual(out.v_raw, 0.5 * 0.5, delta=TOL)  # k_ρ·ρ
        self.assertAlmostEqual(out.v, CFG.v_max, delta=TOL)
        self.assertAlmostEqual(out.omega, 0.0, delta=TOL)

    def test_goal_ahead_left_turns_left(self):
        """Đích phía trước bên TRÁI, cùng hướng -> rẽ TRÁI (ω > 0)."""
        out = controller_step(pose(0.5, 0.1, 0))
        alpha = np.arctan2(0.1, 0.5)  # β = 0 − α = −α
        self.assertAlmostEqual(out.omega_raw, 1.5 * alpha + (-0.6) * (-alpha), delta=TOL)
        self.assertGreater(out.omega, 0)

    def test_goal_ahead_right_turns_right(self):
        """Đích phía trước bên PHẢI -> rẽ PHẢI (ω < 0)."""
        self.assertLess(controller_step(pose(0.5, -0.1, 0)).omega, 0)

    def test_beta_steers_right_first(self):
        """Đích ngay phía trước nhưng nằm chéo sang TRÁI 30° -> lượn sang PHẢI trước (ω < 0).

        Như đỗ xe vào chỗ nằm chéo bên trái: phải lượn ra phải để có đường vào.
        Đây là test bắt lỗi k_β bị gõ dương.
        """
        out = controller_step(pose(0.5, 0.0, d(30)))
        self.assertAlmostEqual(out.alpha, 0.0, delta=TOL)
        self.assertAlmostEqual(out.beta, d(30), delta=TOL)
        self.assertAlmostEqual(out.omega_raw, -0.6 * d(30), delta=TOL)
        self.assertLess(out.omega, 0)

    def test_goal_behind_out_of_domain(self):
        """Đích ở nửa sau xe -> OUT_OF_DOMAIN, đứng yên (bản này chỉ tiến)."""
        out = controller_step(pose(-0.3, 0.05, 0))
        self.assertEqual(out.status, "OUT_OF_DOMAIN")
        self.assertEqual((out.v, out.omega), (0.0, 0.0))

    def test_near_goal_align(self):
        """Đã tới chỗ (ρ < pos_tol) nhưng lệch hướng 10° -> xoay tại chỗ sang TRÁI, v = 0."""
        out = controller_step(pose(0.003, 0.002, d(10)))
        self.assertEqual(out.status, "ALIGN")
        self.assertEqual(out.v, 0.0)
        self.assertAlmostEqual(out.omega, 1.0 * d(10), delta=TOL)  # k_h·θ_G

    def test_near_goal_done(self):
        """Tới chỗ và lệch hướng < heading_tol -> DONE, đứng yên."""
        out = controller_step(pose(0.003, 0.002, d(0.5)))
        self.assertEqual(out.status, "DONE")
        self.assertEqual((out.v, out.omega), (0.0, 0.0))

    def test_no_dead_zone(self):
        """Một ngưỡng chung: còn xa hơn pos_tol thì v > 0 -> không có chỗ đứng im ngoài vạch đích."""
        self.assertGreater(controller_step(pose(CFG.pos_tol + 0.001, 0, 0)).v, 0)
        self.assertEqual(controller_step(pose(CFG.pos_tol - 0.001, 0, 0)).status, "DONE")

    def test_mirror_symmetry(self):
        """Lật gương trái <-> phải: v giữ nguyên, ω đổi dấu, cùng trạng thái."""
        for G in [pose(0.5, 0.1, 0), pose(0.4, -0.2, 0.5), pose(0.3, 0.05, -0.8),
                  pose(0.003, 0.002, 0.3), pose(-0.3, 0.1, 0)]:
            with self.subTest(G=fmt(G)):
                out, mirrored = controller_step(G), controller_step(pose(G[0], -G[1], -G[2]))
                self.assertEqual(out.status, mirrored.status)
                self.assertAlmostEqual(out.v, mirrored.v, delta=TOL)
                self.assertAlmostEqual(out.omega, -mirrored.omega, delta=TOL)

    def test_controller_output_is_clamped_keeping_curve(self):
        """Lệnh RA KHỎI controller: không vượt giới hạn và vẫn giữ tỉ v/ω của lệnh gốc."""
        for G in [pose(0.5, 0.1, 0), pose(0.5, 0.0, d(30)), pose(0.8, 0.6, d(-40)), pose(0.3, -0.4, d(60))]:
            with self.subTest(G=fmt(G)):
                out = controller_step(G)
                self.assertLessEqual(abs(out.v), CFG.v_max + TOL)
                self.assertLessEqual(abs(out.omega), CFG.omega_max + TOL)
                # nhân chéo thay cho chia (tránh chia cho 0): ω/v = ω_raw/v_raw
                self.assertAlmostEqual(out.omega * out.v_raw, out.omega_raw * out.v, delta=TOL)

    def test_clamp_keeps_ratio(self):
        """Clamp giảm v, ω cùng hệ số: không vượt giới hạn, tỉ v/ω (độ cong) giữ nguyên."""
        cases = [  # (v, ω) -> mong đợi, với v_max = 0.15, ω_max = 1.0
            ((0.30, 1.0), (0.15, 0.5)),
            ((0.30, -2.0), (0.15, -1.0)),
            ((0.05, 0.2), (0.05, 0.2)),
            ((0.0, 3.0), (0.0, 1.0)),
            ((0.0, 0.0), (0.0, 0.0)),
        ]
        for (v, omega), expected in cases:
            with self.subTest(v=v, omega=omega):
                np.testing.assert_allclose(clamp_command(v, omega, 0.15, 1.0), expected, atol=TOL)


# ======== C: chạy cả vòng lặp (plan §8.2), dùng simulator trong dock_sim ========


class TestDocking(unittest.TestCase):
    def assertDocked(self, run):
        """DONE và sai số THẬT ở đầu male < (1 cm, 1 cm, 2°)."""
        e_lat, e_gap, e_ang = run.errors
        self.assertEqual(run.status, "DONE")
        self.assertLess(abs(e_lat), 0.01, f"lệch ngang {e_lat * 1000:.1f} mm")
        self.assertLess(abs(e_gap), 0.01, f"sai khe {e_gap * 1000:.1f} mm")
        self.assertLess(abs(e_ang), d(2), f"lệch góc {np.rad2deg(e_ang):.2f}°")

    def test_scene_start_docks(self):
        """Cảnh theo hình vẽ tay: từ (-1, 0.25, -15°) vào standby, sai số thật trong ngưỡng."""
        self.assertDocked(dock_sim.simulate_docking(dock_sim.POSE_A_START_IN_W))

    def test_speed_limits_really_applied(self):
        """Đo tốc độ từ quãng đường xe THẬT SỰ đi được, không tin số controller báo."""
        A = dock_sim.simulate_docking(dock_sim.POSE_A_START_IN_W).log["A_in_W"]
        speed = np.hypot(*np.diff(A[:, :2], axis=0).T) / dock_sim.DT
        turn = np.abs(wrap_angle(np.diff(A[:, 2]))) / dock_sim.DT
        self.assertLessEqual(speed.max(), CFG.v_max + TOL)
        self.assertLessEqual(turn.max(), CFG.omega_max + TOL)

    def test_C01_head_on(self):
        """C01: xuất phát ngay trên trục, đúng hướng -> chạy thẳng vào, ω = 0 suốt đường."""
        run = dock_sim.simulate_docking(pose(-1.00, 0.00, 0))
        self.assertDocked(run)
        self.assertLess(np.abs(run.log["omega"]).max(), 1e-9)

    def test_C04_half_dt(self):
        """C04: dt giảm một nửa -> quỹ đạo gần như trùng (so vị trí ở cùng thời điểm)."""
        runs = {dt: dock_sim.simulate_docking(dock_sim.POSE_A_START_IN_W, dt=dt) for dt in (0.02, 0.01)}
        for dt, run in runs.items():
            with self.subTest(dt=dt):
                self.assertDocked(run)
        for t_check in (1.0, 3.0, 6.0, 9.0):
            with self.subTest(t=t_check):
                p1 = runs[0.02].log["A_in_W"][round(t_check / 0.02)]
                p2 = runs[0.01].log["A_in_W"][round(t_check / 0.01)]
                self.assertLess(np.hypot(*(p1 - p2)[:2]), 0.005)  # < 5 mm

    def test_C05_goal_behind(self):
        """C05: xe quay lưng về phía B -> đích ở sau -> OUT_OF_DOMAIN ngay bước đầu, không chạy bậy."""
        run = dock_sim.simulate_docking(pose(-0.60, 0.00, np.pi))
        self.assertEqual(run.status, "OUT_OF_DOMAIN")
        self.assertEqual(len(run.log["t"]), 1)

    def test_timeout_does_not_hang(self):
        """k_ρ = 0 (xe không tiến) -> TIMEOUT đúng lúc max_time, không treo."""
        cfg = ControllerConfig(k_rho=0.0, k_beta=0.0)
        run = dock_sim.simulate_docking(dock_sim.POSE_A_START_IN_W, cfg=cfg, max_time=2.0)
        self.assertEqual(run.status, "TIMEOUT")
        self.assertAlmostEqual(run.log["t"][-1], 2.0, delta=TOL)


if __name__ == "__main__":
    unittest.main()
