# SCRIPT NÀY ĐƯỢC DÙNG ĐỂ FUSION CHÂN TRÁI VÀ CHÂN PHẢI THÀNH CÙNG 1 HƯỚNG
# Thay đổi: loại bỏ phần quay 2 imu trùng nhau vì nó đã trùng nhau sẵn ở phần cứng

import zmq
import math
import numpy as np
import time
import json
import logging
import threading
from typing import List, Tuple, Dict, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - [IMU_FUSION] - %(message)s"
)
logger = logging.getLogger(__name__)


class IMUController:
    """ZeroMQ controller to read IMU data from a single leg"""

    def __init__(self, host: str, port: int, side: str = "LEFT"):
        self.host = host
        self.port = port
        self.side = side

        # ZeroMQ socket setup
        self.context = zmq.Context()
        self.socket = None
        self.lock = threading.Lock()  # Thread lock

        # Cache cho IMU data. "stale" = so nay KHONG phai mau moi.
        self.last_imu_data = {
            "quat": [1, 0, 0, 0],
            "gyro": [0, 0, 0],
            "timestamp": 0,
            "t_sample": None,
            "stale": True,  # chua doc duoc gi -> chua co so that
        }
        self.last_valid_time = 0

        # Dau thoi gian cua mau IMU lan truoc, do leg_server dong tai NGUON.
        # Chi dung de SO SANH voi chinh no ("co phai van mau cu khong"), tuyet
        # doi khong lay time.time() cua may nay tru di - hai may khac dong ho,
        # hieu so giua chung vo nghia.
        self.last_t_sample = None

        self.connect()

    def connect(self):
        """Create a new socket"""
        with self.lock:  # Protect socket creation
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass

            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, 2000)  # 2s timeout (faster reconnect)
            self.socket.setsockopt(zmq.LINGER, 0)

            try:
                self.socket.connect(f"tcp://{self.host}:{self.port}")
                logger.info(f"✓ {self.side} IMU controller connected to {self.host}:{self.port}")
            except Exception as e:
                logger.error(f"✗ {self.side} connection failed: {e}")
                raise

    def _cached(self) -> Dict:
        """Tra lai so cu, nhung DAN NHAN stale=True.

        Tra so cu khi mang hut la dung - nhay ve 0 con te hon. Cai sai cua
        ban cu la tra ma KHONG NOI, khien ben goi tuong day la mau moi.
        """
        d = dict(self.last_imu_data)
        d["stale"] = True
        return d

    def read_imu(self) -> Optional[Dict]:
        """Read IMU data - THREAD-SAFE VERSION"""
        try:
            with self.lock:  # Protect socket access
                try:
                    # 1. Send request
                    self.socket.send_json({"type": "feedback"}, zmq.NOBLOCK)

                    # 2. Wait for response (with timeout)
                    response = self.socket.recv_json()

                    # 3. Extract IMU data
                    imu_data = {
                        "quat": response.get("quat", [1, 0, 0, 0]),
                        "gyro": response.get("gyro", [0, 0, 10]),
                        # leg_server dong dau ngay luc lay mau (imu_t_sample).
                        # None = server chua gui truong nay -> khong the biet cu/moi.
                        "t_sample": response.get("t_sample"),
                        "stale": False,
                    }

                    # 4. Validate data
                    if len(imu_data["quat"]) == 4 and len(imu_data["gyro"]) == 3:
                        # Mau MOI hay van la mau CU? So dau thoi gian nguon voi
                        # lan truoc. Server chay 50Hz, client hoi 20Hz, nen binh
                        # thuong moi lan hoi phai ra mot dau khac. Trung nhau =
                        # vong IMU ben server khong nhich -> treo, hoac ta doc
                        # nhanh hon server san xuat.
                        t_s = imu_data["t_sample"]
                        if t_s is not None and t_s == self.last_t_sample:
                            imu_data["stale"] = True
                        else:
                            self.last_t_sample = t_s
                            self.last_valid_time = time.time()

                        self.last_imu_data = imu_data
                        return imu_data
                    else:
                        logger.debug(f"{self.side}: Invalid IMU data format, using cache")
                        return self._cached()

                except zmq.Again:
                    logger.debug(f"{self.side}: Timeout waiting for IMU, using cache")
                    self.socket.close()  # ✅ FIX: Close broken socket
                    self.socket = self.context.socket(zmq.REQ)  # Create new socket
                    self.socket.setsockopt(zmq.RCVTIMEO, 2000)
                    self.socket.setsockopt(zmq.LINGER, 0)
                    self.socket.connect(f"tcp://{self.host}:{self.port}")
                    return self._cached()

                except zmq.error.ZMQError as e:
                    if "Operation cannot be accomplished in current state" in str(e):
                        logger.debug(f"{self.side}: Socket state error, recreating...")
                        self.socket.close()
                        self.socket = self.context.socket(zmq.REQ)
                        self.socket.setsockopt(zmq.RCVTIMEO, 2000)
                        self.socket.setsockopt(zmq.LINGER, 0)
                        self.socket.connect(f"tcp://{self.host}:{self.port}")
                    else:
                        logger.debug(f"{self.side}: ZMQ Error - {e}")
                    return self._cached()

        except Exception as e:
            logger.debug(f"{self.side}: Error in read_imu - {e}")
            return self._cached()

    def close(self):
        """Close connection"""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass


class IMUFusion:
    """API to fuse IMU data from 2 leg sensors"""

    def __init__(
        self,
        left_host: str = "127.0.0.1",
        left_port: int = 5556,
        right_host: str = "127.0.0.1",
        right_port: int = 5555,
        # khi xác định được imu quay bao nhiêu so với base thì điền vào chỗ này
        mount_rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        """
        Initialize IMU Fusion

        Args:
            left_host: Host of left leg IMU server
            left_port: Port of left leg IMU server
            right_host: Host of right leg IMU server
            right_port: Port of right leg IMU server
            mount_rpy_deg: goc LAP DAT cua chip so voi than robot (baselink),
                theo (roll, pitch, yaw) don vi DO.

                PHAN CUNG MOI: hai con IMU lap SONG SONG, cung huong ca 3 truc
                X/Y/Z. Nen chi con MOT phep xoay duy nhat, dung chung cho ca
                hai - khong con "trai khac phai" nhu ban lap cu.

                (0, 0, 0) nghia la truc chip trung truc than: +X ra truoc,
                +Y sang trai, +Z len troi (quy uoc NWU dang dung trong
                leg_server). Neu ca cap bi xoay so voi than thi dien goc do
                vao day - DAY LA CHO DUY NHAT CAN SUA.

                Cach do: dung robot thang, doc quat tu server. Ra gan
                [1,0,0,0] thi de nguyen (0,0,0).
        """
        # Create controller for each leg
        self.left = IMUController(left_host, left_port, "LEFT")
        self.right = IMUController(right_host, right_port, "RIGHT")

        # Goc lap dat, dung chung cho ca 2 con.
        # Giu ca 2 dang: quaternion de xoay HUONG, ma tran de xoay VECTO gyro.
        # Hai dang nay phai luon mo ta cung mot phep xoay - deu sinh tu day.
        r, p, y = [a * math.pi / 180.0 for a in mount_rpy_deg]
        self.mount_rpy_deg = tuple(mount_rpy_deg)
        self.q_mount = self.euler_to_quat(r, p, y)
        self.R_mount = self.euler_to_rot(r, p, y)

        # Dem so lan lien tiep nhan phai mau cu, de canh bao co tiet che
        # (khong spam log 20 dong/giay khi mang chap chon).
        self._stale_streak = 0

    def quat_normalize(self, q: List[float]) -> List[float]:
        """Normalize quaternion to unit norm"""
        norm = math.sqrt(sum(x**2 for x in q))
        if norm > 1e-10:
            return [x / norm for x in q]
        else:
            return [1, 0, 0, 0]

    def euler_to_rot(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles to rotation matrix"""
        Rx = np.array(
            [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]]
        )

        Ry = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )

        Rz = np.array(
            [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
        )

        return Rz @ Ry @ Rx

    def euler_to_quat(self, roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

    def quat_mult(self, q1: List[float], q2: List[float]) -> List[float]:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        result = [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
        return self.quat_normalize(result)

    def quat_conj(self, q: List[float]) -> List[float]:
        """Quaternion conjugate"""
        return [q[0], -q[1], -q[2], -q[3]]

    def transform_quat_to_baselink(self, q_imu: List[float]) -> List[float]:
        """Doi HUONG do duoc tu he truc chip sang he truc than robot.

        Khong con tham so imu_name: hai con lap song song nen dung chung
        mot phep xoay q_mount.

        Cong thuc: q_base = q_imu (x) q_mount   -- NHAN BEN PHAI.

        Vi sao khong phai q_mount (x) q_imu (x) q_mount* (dang cu):
        q_imu la phep quay tu he CHIP sang he THE GIOI. Ta muon phep quay tu
        he THAN sang he THE GIOI. He the gioi (goc la trong luc) la CHUNG cho
        ca hai, khong duoc xoay theo. Nhan hai dau nhu code cu se xoay luon
        he the gioi -> sai.

        Kiem chung: robot dung thang, chip lap lech q_mount thi chip do duoc
        q_imu = q_mount^-1. Nhan phai: q_mount^-1 (x) q_mount = [1,0,0,0],
        dung la "than dang thang". Dang cu cho ra q_mount^-1, sai.

        Ban lap cu chi xoay quanh Z nen loi nay bi che lap (xoay he the gioi
        quanh Z chi lam lech yaw, ma yaw thi khong co tham chieu). Lap moi neu
        co thanh phan roll/pitch thi loi se lo ra ngay.
        """
        return self.quat_mult(self.quat_normalize(q_imu), self.q_mount)

    def transform_gyro_to_baselink(self, gyro: np.ndarray) -> np.ndarray:
        """Doi VAN TOC GOC tu he truc chip sang he truc than robot.

        Gyro la mot vecto do trong he chip. Quan he lap dat la
        v_chip = R_mount @ v_than, nen chieu nguoc lai la
        v_than = R_mount^T @ v_chip.

        Da BO hai dong lat dau GX/GY cua ban cu - do la chinh tay de bu cach
        lap cu, khong co co so hinh hoc. Giu lai se lam gyro X/Y nguoc dau.
        """
        return self.R_mount.T @ np.array(gyro, dtype=float)

    def fuse_gyro(
        self, gyro_left: np.ndarray, gyro_right: np.ndarray, weight_left: float = 0.5
    ) -> np.ndarray:
        """Fuse gyro data from two IMUs using weighted average"""
        weight_right = 1.0 - weight_left
        gyro_left = np.array(gyro_left, dtype=float)
        gyro_right = np.array(gyro_right, dtype=float)
        fused = weight_left * gyro_left + weight_right * gyro_right
        return fused

    def quat_to_euler(self, q: List[float]) -> Tuple[float, float, float]:
        """Convert quaternion to Euler angles (radians)"""
        w, x, y, z = q

        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm > 1e-10:
            w, x, y, z = w / norm, x / norm, y / norm, z / norm

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def fuse_quat(self, q1: List[float], q2: List[float], weight1: float = 0.5) -> List[float]:
        """Fuse two quaternions with weighted average"""
        weight2 = 1 - weight1

        q1 = self.quat_normalize(q1)
        q2 = self.quat_normalize(q2)

        dot = sum(q1[i] * q2[i] for i in range(4))
        if dot < 0:
            q2 = [-x for x in q2]

        fused = [q1[i] * weight1 + q2[i] * weight2 for i in range(4)]

        return self.quat_normalize(fused)

    def get_fused_imu(self) -> Optional[Dict]:
        """Read and fuse IMU data from 2 leg sensors"""
        try:
            # Request IMU data (thread-safe now)
            left_raw = self.left.read_imu()
            right_raw = self.right.read_imu()

            if not left_raw or not right_raw:
                logger.debug("Failed to read IMU data")
                return None

            left_quat = left_raw.get("quat", [1, 0, 0, 0])
            right_quat = right_raw.get("quat", [1, 0, 0, 0])
            left_gyro_raw = left_raw.get("gyro", [0, 0, 0])
            right_gyro_raw = right_raw.get("gyro", [0, 0, 0])

            # Mau nay co that su moi khong? Ben goi PHAI biet dieu nay.
            left_stale = bool(left_raw.get("stale", False))
            right_stale = bool(right_raw.get("stale", False))
            any_stale = left_stale or right_stale

            if any_stale:
                self._stale_streak += 1
                # Keu o lan dau, roi cu 50 lan mot - du de thay, khong ngap log.
                if self._stale_streak == 1 or self._stale_streak % 50 == 0:
                    ben = []
                    if left_stale:
                        ben.append("TRAI")
                    if right_stale:
                        ben.append("PHAI")
                    logger.warning(
                        f"⚠️  Mau IMU CU (khong phai so moi): {'+'.join(ben)} "
                        f"- lien tiep {self._stale_streak} lan"
                    )
            elif self._stale_streak:
                logger.info(f"✓ IMU tuoi tro lai sau {self._stale_streak} mau cu")
                self._stale_streak = 0

            left_q = self.transform_quat_to_baselink(left_quat)
            right_q = self.transform_quat_to_baselink(right_quat)
            fused_q = self.fuse_quat(left_q, right_q)

            left_gyro_transformed = self.transform_gyro_to_baselink(left_gyro_raw)
            right_gyro_transformed = self.transform_gyro_to_baselink(right_gyro_raw)
            fused_gyro = self.fuse_gyro(left_gyro_transformed, right_gyro_transformed)

            fused_euler = self.quat_to_euler(fused_q)

            return {
                "left_quat": left_q,
                "right_quat": right_q,
                "fused_quat": fused_q,
                "fused_euler": fused_euler,
                "left_gyro": left_gyro_transformed.tolist(),
                "right_gyro": right_gyro_transformed.tolist(),
                "fused_gyro": fused_gyro.tolist(),
                "timestamp": time.time(),
                # stale=True nghia la it nhat mot ben KHONG gui mau moi.
                # Cac so o tren van dung dinh dang, nhung dung coi la tuoi.
                "stale": any_stale,
                "left_stale": left_stale,
                "right_stale": right_stale,
                "left_t_sample": left_raw.get("t_sample"),
                "right_t_sample": right_raw.get("t_sample"),
            }

        except Exception as e:
            logger.error(f"Error in get_fused_imu: {e}")
            return None

    def close(self):
        """Close all connections"""
        self.left.close()
        self.right.close()
