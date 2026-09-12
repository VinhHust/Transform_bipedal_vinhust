import zmq
import json
import struct
import time
import threading
from typing import Dict, List, Optional
import logging
from collections import deque
import sys
from pathlib import Path
import qwiic_icm20948
import math
import imufusion
import numpy as np

# CHÂN TRÁI LÀ LEG SERVER 2
# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from bipedal_robot.bipedal_left import BipedalRobot, BipedalConfig

# Configure logging.
# force=True la BAT BUOC: dong import lerobot o tren da gan san mot handler vao
# root logger, ma basicConfig() mac dinh se lang le khong lam gi khi root da co
# handler -> mat ca format lan level=INFO, va moi log INFO o tam module (dong
# bao calib, dong bao IMU) bi nuot vi root van dung o muc WARNING.
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - [MCU_SERVER_LEFT] - %(message)s",
    force=True,
)
logger = logging.getLogger(__name__)

# ==============================
# Load calibration
# ==============================
CALIB_DIR = Path("/home/mobile2/Transform_bipedal/bipedal_nam/Calib/READ")
GYRO_CALIB_PATH = CALIB_DIR / "vinhgyrocalib.json"
ACCEL_CALIB_PATH = CALIB_DIR / "vinh_accel_calib_ellipsoid.json"

with open(ACCEL_CALIB_PATH, "r") as f:
    accel_calib = json.load(f)

with open(GYRO_CALIB_PATH, "r") as f:
    gyro_calib = json.load(f)

# calib từ accel
accel_SM = np.array(accel_calib["accel"]["SM"])
accel_bias = np.array(accel_calib["accel"]["bias"])

# calib từ gyro
gx_bias = gyro_calib["gx_bias"]
gy_bias = gyro_calib["gy_bias"]
gz_bias = gyro_calib["gz_bias"]
GYRO_SENSITIVITY = gyro_calib["gyro_sensitivity"]

logger.info(f"Calib loaded: accel={ACCEL_CALIB_PATH.name}, gyro={GYRO_CALIB_PATH.name}")


# Initialize IMU
IMU = qwiic_icm20948.QwiicIcm20948()
if not IMU.connected:
    logger.error("IMU not found! Exiting...")
    # sys.exit(1) chu KHONG exit(): exit() thoat voi ma 0 = "thanh cong", systemd
    # va script bao boc se tuong server tat binh thuong va khong restart.
    sys.exit(1)

IMU.begin()
IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)  # Khớp FSR ±500 dps lúc calib
IMU.setFullScaleRangeAccel(qwiic_icm20948.gpm4)  # Khớp FSR ±4g lúc calib

# DLPF cho accel và gyro
IMU.setDLPFcfgAccel(qwiic_icm20948.acc_d23bw9_n34bw4)
IMU.setDLPFcfgGyro(qwiic_icm20948.gyr_d23bw9_n35bw9)
IMU.enableDlpfAccel(True)
IMU.enableDlpfGyro(True)
logger.info("ICM-20948 IMU initialized (FSR: ±500dps, ±4g, output in m/s²)")

# khởi tạo cho thư viện imufusion
RATE = 50.0  # Tần số mong muốn (Hz)
G = 9.80665

# 1. Khởi tạo AHRS
ahrs_settings = imufusion.AhrsSettings(
    RATE,  # sample_rate (50 Hz)
    imufusion.CONVENTION_NWU,  # Hệ trục chuẩn Z-hướng-lên
    0.5,  # gain
    500.0,  # gyroscope_range (±500 dps)
    10.0,  # acceleration_rejection (độ - tự lọc xung giật khi bước chân)
    0.0,  # magnetic_rejection (không dùng từ kế)
    5.0,  # rejection_timeout (giây)
)
ahrs = imufusion.Ahrs()
ahrs.set_settings(ahrs_settings)

# khởi tạo AHRS học trôi cho gyroscope
bias_settings = imufusion.BiasSettings(RATE, 3.0, 3.0)
bias = imufusion.Bias()
bias.set_settings(bias_settings)

# bắt đầu bằng giá trị đầu đã calib cho gyro
GB = np.array([gx_bias, gy_bias, gz_bias])
bias.set_offset(GB / GYRO_SENSITIVITY)


class MCUServerLeft:
    """MCU control server for LEFT leg on Raspberry Pi 5."""

    def __init__(self, zmq_port: int = 5556, serial_port: str = "/dev/ttyACM0"):
        """
        Initialize LEFT leg server - motor 4-9 (LEFT leg only).

        Args:
            zmq_port: ZeroMQ socket port
            serial_port: Serial port for servo communication
        """
        self.zmq_port = zmq_port
        self.serial_port = serial_port
        self.running = False

        # Robot instance (BipedalRobot from bipedal_left.py)
        self.config = BipedalConfig(port=serial_port, baudrate=1_000_000)
        self.robot = None

        #  Motor mapping - LEFT leg (motor 4-9)
        self.servo_map = {
            4: "bubleft_joint",  # LEFT leg_bub (sts3095)
            5: "hipleft_joint",  # LEFT leg_hip (sts3095)
            6: "twistleft_joint",  # LEFT leg_twist (sts3215)
            7: "kneeleft_joint",  # LEFT leg_knee (sts3095)
            8: "footleft_joint",  # LEFT leg_foot (sts3215)
            9: "gripperleft_joint",  # LEFT leg_gripper (sts3215)
        }

        # Servo control parameters
        self.servo_speed = 1000  # Goal velocity
        self.servo_accel = 254  # Acceleration

        #  Servo limits for LEFT leg
        self.servo_limits = {
            4: {"min": 996, "max": 2146},
            5: {"min": 1676, "max": 2396},
            6: {"min": 1030, "max": 3060},
            7: {"min": 996, "max": 3051},
            8: {"min": 1416, "max": 2711},
            9: {"min": 1802, "max": 2646},
        }

        # Current positions (6 servos: motor 4-9)
        self.current_positions = [0] * 6
        self.target_positions = [0] * 6

        # Servo feedback history
        self.feedback_history = deque(maxlen=100)

        # State data (6 servos)
        self.state_data = {
            "imu": [0.0, 0.0, 0.0, 0.0],
            "gyro_rad": [0.0, 0.0, 0.0],  # gyro da loc, rad/s - tra ve cho client
            "imu_t_sample": 0.0,  # ✅ THÊM: thời điểm lấy mẫu IMU (đóng dấu tại nguồn)
            "distance": [0, 0, 0, 0],
            "servo_pos": [0] * 6,
            "servo_speed": [0] * 6,
            "servo_load": [0] * 6,
            "servo_voltage": [0] * 6,
            "servo_current": [0] * 6,
            "servo_temp": [0] * 6,
        }

        # Moc thoi gian mau IMU truoc do, dung de tinh dt cho imufusion.
        # Dung dong ho monotonic (khong bao gio nhay lui) chu KHONG dung
        # time.time() - NTP chinh gio co the lam dt am, pha hong tich phan gyro.
        self.t_prev_imu = None

        # ZeroMQ context and socket. Khoi tao None tu day: neu init_zmq() hong
        # giua chung, finally trong main() van goi shutdown() va se cham vao
        # cac thuoc tinh nay - chua co thi AttributeError se che mat loi that.
        self.context = zmq.Context()
        self.socket_rep = None
        self.socket_pull = None
        self.poller = None
        self._shutdown_done = False

        # Update loop control
        self.control_rate = 50  # Hz
        self.last_update_time = time.time()
        self.update_thread = None
        self.imu_thread = None
        self.servo_thread = None

        # Thread locks
        self.read_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        self.serial_lock = threading.Lock()

        logger.info(f"MCUServerLeft initialized (6 LEFT leg servos 4-9) on port {zmq_port}")

    def init_zmq(self) -> bool:
        """Initialize ZeroMQ sockets - BOTH REQ/REP and PUSH/PULL"""
        try:
            self.socket_rep = self.context.socket(zmq.REP)
            self.socket_rep.setsockopt(zmq.RCVTIMEO, 100)
            self.socket_rep.bind(f"tcp://*:{self.zmq_port}")  # 5556
            logger.info(f"✓ REQ/REP socket bound to port {self.zmq_port} (feedback)")

            self.socket_pull = self.context.socket(zmq.PULL)
            self.socket_pull.setsockopt(zmq.RCVTIMEO, 100)
            self.socket_pull.bind(f"tcp://*:{self.zmq_port + 100}")  # 5656
            logger.info(f"✓ PUSH/PULL socket bound to port {self.zmq_port + 100} (async commands)")

            self.poller = zmq.Poller()
            self.poller.register(self.socket_rep, zmq.POLLIN)
            self.poller.register(self.socket_pull, zmq.POLLIN)

            return True
        except Exception as e:
            logger.error(f"✗ Failed to initialize ZeroMQ: {e}")
            return False

    def init_robot(self) -> bool:
        """Initialize BipedalRobot (LEFT leg) with initial feedback read"""
        try:
            self.robot = BipedalRobot(self.config)
            self.robot.connect()
            self.robot.configure()

            logger.info("✓ Robot (LEFT leg) connected and configured")
            logger.info(f"✓ Available motors: {list(self.robot.bus.motors.keys())}")

            # Initialize feedback
            logger.info("Initializing servo feedback (reading actual positions)...")
            time.sleep(0.5)
            self.update_servo_feedback()

            # Verify feedback
            with self.read_lock:
                current_pos = self.state_data["servo_pos"].copy()

            logger.info(f"Initial positions read: {current_pos}")

            if all(p == 0 for p in current_pos):
                logger.warning("  Initial feedback all zeros!")
                logger.warning("   Retrying after 1 second...")
                time.sleep(1)
                self.update_servo_feedback()

                with self.read_lock:
                    current_pos = self.state_data["servo_pos"].copy()

                logger.info(f"After retry: {current_pos}")

                if all(p == 0 for p in current_pos):
                    logger.error(" Feedback still all zeros!")
                    return False

            logger.info(f" Initial positions: {current_pos}")
            logger.info(" Robot initialization complete")
            return True

        except Exception as e:
            logger.error(f" Failed to initialize robot: {e}")
            import traceback

            traceback.print_exc()
            return False

    def update_servo_feedback(self) -> None:
        """
         OPTIMIZED: Read ONLY servo positions (no speed/load/voltage/current/temp)
        This is 5x faster and enough for policy control!
        """
        try:
            if not self.robot or not self.robot.is_connected:
                logger.debug("Robot not connected")
                return

            for servo_id in range(4, 10):
                motor_name = self.servo_map[servo_id]

                if motor_name not in self.robot.bus.motors:
                    logger.warning(f" Motor {motor_name} (ID {servo_id}) NOT in bus!")
                    continue

                try:
                    idx = servo_id - 4

                    # CHỈ đọc position (1 read per motor ~20ms)
                    with self.serial_lock:
                        pos = self.robot.bus.read("Present_Position", motor_name, normalize=False)

                    if pos is not None and pos > 0:
                        with self.read_lock:
                            self.state_data["servo_pos"][idx] = int(pos)
                    else:
                        with self.read_lock:
                            old_value = self.state_data["servo_pos"][idx]
                        logger.warning(f" {motor_name}: read failed, keeping {old_value}")

                    # (Chỉ enable khi debug hoặc monitor robot health)

                except Exception as e:
                    logger.error(f"Exception reading {motor_name}: {e}")

        except Exception as e:
            logger.error(f"Error in update_servo_feedback: {e}")

    # HÀM NÀY ĐƯỢC SỬA CHO KHỚP VỚI CẤU TRÚC IMUFUSION MỚI
    def update_imu_data(self) -> bool:
        """Update IMU data with calibration and store in state_data.

        Tra ve True neu vua xu ly xong mot mau moi, False neu chip chua co
        du lieu san (de imu_loop biet nen cho ngan roi thu lai, thay vi ngu
        tron mot chu ky 20ms).

        KHONG bat exception o day: loi that (I2C rot, chip treo) phai noi len
        cho imu_loop - no ngu 10ms roi dat lai nhip. Neu nuot loi va tra False,
        imu_loop hieu nham la "chip chua san sang", chi ngu 1ms roi thu lai
        -> vong lap loi 1000 Hz, ngap log va an het mot loi CPU.
        """
        if not IMU.dataReady():
            return False

        IMU.getAgmt()
        t_mono = time.monotonic()  # do dt - dong ho chi tien
        t_sample = time.time()  # dau thoi gian gui ve laptop - gio treo tuong

        # Read raw data
        araw = np.array([IMU.axRaw, IMU.ayRaw, IMU.azRaw], dtype=float)
        graw = np.array([IMU.gxRaw, IMU.gyRaw, IMU.gzRaw], dtype=float)

        # chuyển đơn vị của accel thành g và gyro thành độ/s
        acc = (accel_SM @ araw - accel_bias) / G  # Đơn vị: g
        gyr_deg = bias.update(graw / GYRO_SENSITIVITY)  # Đơn vị: °/s

        # tính dt và kẹp lại giá trị của dt trong khoảng tránh giá trị dt lệch quá làm hỏng filter
        if self.t_prev_imu is None:
            dt = 1.0 / RATE
        else:
            dt = t_mono - self.t_prev_imu
        self.t_prev_imu = t_mono

        dt_clamped = min(max(dt, 0.5 / RATE), 2.0 / RATE)
        ahrs.set_sample_period(dt_clamped)

        # chạy fusion filter ahrs
        ahrs.update_no_magnetometer(gyr_deg, acc)
        q_new = np.asarray(ahrs.get_quaternion(), dtype=float)  # [w, x, y, z]

        #  Store quaternion in state_data
        with self.imu_lock:
            self.state_data["imu"] = q_new.tolist()
            self.state_data["imu_t_sample"] = t_sample
            # Lưu gyro chuyển sang rad/s để trả về cho client
            self.state_data["gyro_rad"] = (gyr_deg * (math.pi / 180.0)).tolist()

        return True

    def update_distance_sensors(self) -> None:
        """Update distance sensors (placeholder)."""
        pass

    def get_state_data_bytes(self) -> bytes:
        """Get state data as bytes (90 bytes total)"""
        try:
            imu_bytes = struct.pack("<4f", *self.state_data["imu"])
            dist_bytes = struct.pack("<4H", *self.state_data["distance"])
            pos_bytes = struct.pack("<6H", *self.state_data["servo_pos"])
            speed_bytes = struct.pack("<6h", *self.state_data["servo_speed"])
            load_bytes = struct.pack("<6h", *self.state_data["servo_load"])
            voltage_bytes = struct.pack("<6H", *self.state_data["servo_voltage"])
            current_bytes = struct.pack("<6h", *self.state_data["servo_current"])
            temp_bytes = struct.pack("<6B", *self.state_data["servo_temp"])

            state_bytes = (
                imu_bytes
                + dist_bytes
                + pos_bytes
                + speed_bytes
                + load_bytes
                + voltage_bytes
                + current_bytes
                + temp_bytes
            )

            logger.debug(f"State data packed: {len(state_bytes)} bytes")
            return state_bytes

        except Exception as e:
            logger.error(f"Error packing state data: {e}")
            return b""

    def apply_new_positions(self, positions: List[int]) -> bool:
        """Apply new positions to LEFT leg servos"""
        try:
            if not self.robot or not self.robot.is_connected:
                logger.warning("Robot not connected")
                return False

            if len(positions) != 6:
                logger.error(f"Expected 6 positions, got {len(positions)}")
                return False

            success_count = 0
            fail_count = 0

            # serial_lock (khong phai write_lock): giu cong serial suot ca 6 lenh ghi
            # de luong doc feedback khong chen vao giua
            with self.serial_lock:
                for servo_id in range(4, 10):
                    motor_name = self.servo_map[servo_id]
                    pos_idx = servo_id - 4
                    target_pos = positions[pos_idx]

                    if motor_name not in self.robot.bus.motors:
                        logger.warning(f"Motor {motor_name} (ID {servo_id}) not found")
                        fail_count += 1
                        continue

                    limits = self.servo_limits[servo_id]
                    clamped_pos = max(limits["min"], min(limits["max"], target_pos))

                    try:
                        logger.debug(
                            f"Sending to {motor_name} (ID {servo_id}): pos={clamped_pos}, "
                            f"speed={self.servo_speed}, accel={self.servo_accel}"
                        )

                        # KHONG khoa serial_lock o day: vong lap ngoai da giu roi,
                        # threading.Lock khong reentrant -> khoa lai se treo
                        self.robot.write_pos_ex(
                            motor_name=motor_name,
                            position=clamped_pos,
                            speed=self.servo_speed,
                            acceleration=self.servo_accel,
                            normalize=False,
                        )
                        success_count += 1

                    except Exception as e:
                        logger.error(f"Failed to set {motor_name} (ID {servo_id}): {e}")
                        fail_count += 1

            self.target_positions = positions.copy()

            result = success_count > 0 and fail_count == 0
            # debug chu khong info: client gui move ~25-33 lan/giay, o muc info
            # se do tung ay dong log moi giay vao journald - ton I/O tren Pi.
            logger.debug(
                f"Applied positions (LEFT leg 4-9): {success_count} success, {fail_count} failed"
            )
            if fail_count:
                logger.warning(f"Move: {success_count} success, {fail_count} FAILED")
            return result

        except Exception as e:
            logger.error(f"Error in apply_new_positions: {e}")
            return False

    def process_command(self, command: Dict) -> Dict:
        """Process incoming command"""
        try:
            cmd_type = command.get("type")

            if cmd_type == "move":
                positions = command.get("positions", [])
                success = self.apply_new_positions(positions)
                return {
                    "status": "success" if success else "error",
                    "current_positions": self.target_positions,
                }

            elif cmd_type == "feedback":
                # ✅ THÊM: Trả về gyro data
                with self.imu_lock:
                    imu_quat = self.state_data["imu"].copy()
                    imu_gyro = self.state_data["gyro_rad"].copy()
                    imu_t = self.state_data["imu_t_sample"]  # ✅ THÊM

                # Copy duoi read_lock: servo_loop ghi vao list nay o thread khac,
                # tra ve thang chinh list se gui di mot ban nua cu nua moi.
                with self.read_lock:
                    servo_pos = self.state_data["servo_pos"].copy()

                return {
                    "status": "success",
                    "quat": imu_quat,
                    "gyro": imu_gyro,  # rad/s, da tru bias boi imufusion.Bias
                    "t_sample": imu_t,  # ✅ THÊM: gửi dấu thời gian về laptop
                    "servo_pos": servo_pos,
                    "servo_speed": self.state_data["servo_speed"],
                    "servo_load": self.state_data["servo_load"],
                    "servo_voltage": self.state_data["servo_voltage"],
                    "servo_current": self.state_data["servo_current"],
                    "servo_temp": self.state_data["servo_temp"],
                }

            elif cmd_type == "home":
                home_pos = [2048, 2048, 2048, 2048, 2048, 2048]
                success = self.apply_new_positions(home_pos)
                return {
                    "status": "success" if success else "error",
                    "message": "Home position reached",
                }

            elif cmd_type == "stop":
                # Giu vi tri DANG DO DUOC, khong dung target_positions: chua co
                # lenh move nao thi target_positions van la [0]*6, gui di se bi
                # kep ve gioi han duoi -> chan GIAT MANH xuong day hanh trinh
                # dung luc dang muon dung khan.
                with self.read_lock:
                    stop_pos = self.state_data["servo_pos"].copy()
                success = self.apply_new_positions(stop_pos)
                return {
                    "status": "success" if success else "error",
                    "message": "Servos (4-9) stopped",
                }

            elif cmd_type == "config":
                if "speed" in command:
                    self.servo_speed = command["speed"]
                if "acceleration" in command:
                    self.servo_accel = command["acceleration"]
                return {
                    "status": "success",
                    "message": f"Config updated: speed={self.servo_speed}, accel={self.servo_accel}",
                }

            else:
                return {
                    "status": "error",
                    "message": f"Unknown command type: {cmd_type}",
                }

        except Exception as e:
            logger.error(f"Error processing command: {e}")
            return {"status": "error", "message": str(e)}

    def imu_loop(self) -> None:
        """✅ IMU thread - 50Hz independent loop"""
        logger.info("IMU loop started (50Hz - independent)")

        # Ngu TOI MOC dich thay vi ngu them 20ms: thoi gian doc I2C + fusion
        # cong don vao moi vong, neu ngu them thi chu ky that > 20ms. Khi chu
        # ky vuot 2/RATE = 40ms thi dt bi ket o tren -> tich phan gyro thieu
        # thoi gian -> goc troi. Xem docs/fusion.md.
        t_next = time.monotonic()

        while self.running:
            try:
                if not self.update_imu_data():
                    # chip chua san sang - cho ngan roi thu lai, dung bo ca chu ky
                    time.sleep(0.001)
                    continue

                t_next += 1.0 / RATE
                sleep_s = t_next - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    # da tre hon nhip - bo qua phan no va bat lai tu bay gio
                    t_next = time.monotonic()
            except Exception as e:
                logger.error(f"Error in IMU loop: {e}")
                time.sleep(0.01)
                t_next = time.monotonic()

        logger.info("IMU loop stopped")

    def servo_loop(self) -> None:
        """✅ Servo thread - 25Hz independent loop"""
        logger.info("Servo loop started (25Hz - independent)")

        while self.running:
            try:
                self.update_servo_feedback()

                self.update_distance_sensors()
                time.sleep(0.04)  # 25Hz = 40ms

            except Exception as e:
                logger.error(f"Error in Servo loop: {e}")
                time.sleep(0.01)

        logger.info("Servo loop stopped")

    def update_loop(self) -> None:
        """⚠️ KHÔNG DÙNG NỮA"""
        logger.info("⚠️  Main update loop (deprecated)")
        while self.running:
            time.sleep(0.1)
        logger.info("Main update loop stopped")

    def run(self) -> None:
        """Main server loop - Monitor cả REQ/REP và PUSH/PULL"""
        self.running = True

        # ✅ START IMU thread (50Hz)
        self.imu_thread = threading.Thread(target=self.imu_loop, daemon=True)
        self.imu_thread.start()
        logger.info("✓ IMU loop thread started (50Hz)")

        # ✅ START Servo thread (25Hz)
        self.servo_thread = threading.Thread(target=self.servo_loop, daemon=True)
        self.servo_thread.start()
        logger.info("✓ Servo loop thread started (25Hz)")

        # ✅ WARMUP IMU
        logger.info("⏳ Warming up IMU (filter convergence)...")
        warmup_time = 5.0
        warmup_start = time.time()
        imu_samples = 0
        valid_count = 0

        while time.time() - warmup_start < warmup_time:
            time.sleep(0.02)
            imu_samples += 1

            with self.imu_lock:
                imu_data = self.state_data["imu"].copy()

            if imu_data != [0.0, 0.0, 0.0, 0.0]:
                valid_count += 1
                if valid_count % 50 == 0:
                    logger.info(f"  ✓ IMU valid: {valid_count} samples")

        logger.info(f"✅ Warmup complete: {imu_samples} samples, {valid_count} valid")
        logger.info("MCU Server started with separate IMU/Servo loops...")

        try:
            while self.running:
                # ⭐ THÊM: Use poller để monitor cả 2 sockets
                socks = dict(self.poller.poll(timeout=10))

                # ✅ Process REQ/REP (feedback queries)
                if self.socket_rep in socks:
                    try:
                        message = self.socket_rep.recv_json()
                        logger.debug(f"REQ: {message}")
                        response = self.process_command(message)
                        self.socket_rep.send_json(response)
                    except zmq.Again:
                        pass
                    except Exception as e:
                        logger.error(f"REQ/REP error: {e}")

                # ⭐ THÊM: Process PUSH/PULL (async commands)
                if self.socket_pull in socks:
                    try:
                        command = self.socket_pull.recv_json()
                        logger.debug(f"PUSH: {command}")
                        # ✅ SỬA: Chỉ process move commands (không cần response)
                        if command.get("type") == "move":
                            positions = command.get("positions", [])
                            self.apply_new_positions(positions)
                            logger.debug(f"Async move applied: {positions}")
                    except zmq.Again:
                        pass
                    except Exception as e:
                        logger.error(f"PUSH/PULL error: {e}")

        except KeyboardInterrupt:
            logger.info("Server interrupted by user")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Shutdown the server"""
        # run() va main() deu co finally goi shutdown() -> chay 2 lan. Lan 2 se
        # disconnect robot lan nua va in lai log ket thuc.
        if self._shutdown_done:
            return
        self._shutdown_done = True

        self.running = False

        # Wait for threads
        if self.imu_thread:
            self.imu_thread.join(timeout=2.0)
            logger.info("IMU thread stopped")

        if self.servo_thread:
            self.servo_thread.join(timeout=2.0)
            logger.info("Servo thread stopped")

        if self.update_thread:
            self.update_thread.join(timeout=2.0)

        # ⭐ THÊM: Close cả 2 sockets
        if self.socket_rep:
            self.socket_rep.close()
        if self.socket_pull:
            self.socket_pull.close()

        if self.robot:
            try:
                self.robot.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting robot: {e}")

        self.context.term()
        logger.info("MCU Server shutdown complete")


def main():
    """Start the LEFT leg MCU server."""
    import argparse

    parser = argparse.ArgumentParser(description="Bimo LEFT Leg MCU Control Server (6 Servos 4-9)")
    parser.add_argument("--port", type=int, default=5556, help="ZeroMQ port")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="Servo serial port")
    parser.add_argument("--speed", type=int, default=3400, help="Default servo speed")
    parser.add_argument("--acceleration", type=int, default=254, help="Default servo acceleration")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")  # ✅ Thêm

    args = parser.parse_args()

    # ✅ Set log level based on --debug flag
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    else:
        logging.getLogger().setLevel(logging.INFO)

    server = MCUServerLeft(zmq_port=args.port, serial_port=args.serial_port)
    server.servo_speed = args.speed
    server.servo_accel = args.acceleration

    try:
        if not server.init_zmq():
            return

        if not server.init_robot():
            return

        server.run()

    except Exception as e:
        logger.error(f"Failed to start server: {e}")
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
