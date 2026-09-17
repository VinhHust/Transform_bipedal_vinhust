# IMU FUSION 2 CHAN - BAN PUB/SUB (LOA PHAT THANH)
#
# Ban nay la BAN SAO cua imu.py, chi doi cach LAY SO tu leg_server:
#   imu.py        : REQ/REP - hoi roi DUNG CHO tra loi. Moi lan doc ton mot
#                   vong wifi (~45ms), 2 chan noi tiep ~90ms -> khong bao gio
#                   dat 20Hz. Mang nghen thi vong policy nghen theo.
#   imu_pubsub.py : PUB/SUB - server tu phat 50Hz, ben nay chi moc "cau moi
#                   nhat" ra khoi hop thu, KHONG CHO. read_imu() tra ve ngay
#                   lap tuc; khong co mau moi thi tra mau cu + stale=True.
#
# Phan toan (transform, fuse, quat_to_euler) va giao dien get_fused_imu()
# GIU NGUYEN so voi imu.py, nen policy_run / transformer / collect_imu_fusion
# doi sang file nay khong phai sua gi khac.
#
# Dung voi leg_server_pubsub/leg_Server_*.py (server co socket PUB o port
# zmq_port + 200). Server cu (leg_server/) KHONG phat -> file nay se chi thay
# stale mai mai.

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

# leg_server phat IMU o port = zmq_port + offset nay (5555 -> 5755, 5556 -> 5756).
# Phai TRUNG voi IMU_PUB_PORT_OFFSET trong leg_server_pubsub/leg_Server_*.py.
IMU_PUB_PORT_OFFSET = 200

# Qua bao lau khong nhan duoc mau nao thi coi server chet (canh bao 1 lan).
SUB_DEAD_AFTER_S = 1.0


class IMUController:
    """Nghe IMU cua MOT chan qua ZeroMQ SUB.

    Giao dien giong het IMUController trong imu.py: read_imu() tra ve dict
    {quat, gyro, t_sample, stale} hoac None. Khac biet: KHONG BAO GIO CHAN.
    """

    def __init__(self, host: str, port: int, side: str = "LEFT"):
        self.host = host
        self.port = port  # port REQ/REP cua server; port PUB = port + offset
        self.side = side

        self.context = zmq.Context()
        self.socket = None
        self.lock = threading.Lock()

        # Cache. "stale" = so nay KHONG phai mau moi.
        self.last_imu_data = {
            "quat": [1, 0, 0, 0],
            "gyro": [0, 0, 0],
            "timestamp": 0,
            "t_sample": None,
            "seq": None,
            "stale": True,  # chua nhan duoc gi -> chua co so that
        }
        self.last_valid_time = 0

        # Dem mau de phat hien roi. seq do server danh, tang 1 moi mau phat.
        # Nhan seq=10 roi seq=13 nghia la 11, 12 roi tren duong (hoac bi
        # CONFLATE de len vi ta doc cham hon server phat - deu la "khong thay").
        self.last_seq = None
        self.n_received = 0
        self.n_dropped = 0

        # Luc nhan mau gan nhat, theo dong ho MAY NAY (monotonic). Khong tru
        # t_sample cua server - hai may khac dong ho, hieu so vo nghia.
        self.last_rx_mono = None
        self._dead_warned = False

        self.connect()

    def connect(self):
        """Tao socket SUB va noi toi port PUB cua server."""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass

            pub_port = self.port + IMU_PUB_PORT_OFFSET
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.LINGER, 0)
            # CONFLATE = hop thu chi giu MOT la thu moi nhat. Doc cham thi la cu
            # bi de len, khong bao gio phai "doc bu" ca dong mau cu. Phai dat
            # TRUOC connect. Khong dung duoc voi multipart -> server gui 1 frame.
            self.socket.setsockopt(zmq.CONFLATE, 1)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.connect(f"tcp://{self.host}:{pub_port}")
            logger.info(f"✓ {self.side} IMU SUB connected to {self.host}:{pub_port}")

    def _cached(self) -> Dict:
        """Tra lai so cu, nhung DAN NHAN stale=True."""
        d = dict(self.last_imu_data)
        d["stale"] = True
        return d

    def _check_dead(self, now_mono: float) -> None:
        """Canh bao MOT lan neu qua lau khong nhan duoc gi; bao lai khi song."""
        if self.last_rx_mono is None:
            return
        silent = now_mono - self.last_rx_mono
        if silent > SUB_DEAD_AFTER_S and not self._dead_warned:
            logger.warning(f"⚠️  {self.side}: khong nhan mau IMU nao {silent:.1f}s - server chet hoac mat wifi?")
            self._dead_warned = True

    def read_imu(self) -> Optional[Dict]:
        """Lay mau moi nhat trong hop thu. KHONG CHAN.

        Co mau moi -> tra ve, stale=False. Khong co -> tra mau cu, stale=True.
        Chua tung nhan mau nao -> tra cache mac dinh [1,0,0,0] voi stale=True
        (ben goi dung stale de biet, KHONG duoc coi [1,0,0,0] la "dang thang").
        """
        try:
            with self.lock:
                now_mono = time.monotonic()
                try:
                    # CONFLATE -> toi da mot thu trong hop; NOBLOCK -> khong cho.
                    response = self.socket.recv_json(zmq.NOBLOCK)
                except zmq.Again:
                    self._check_dead(now_mono)
                    return self._cached()

                quat = response.get("quat", [1, 0, 0, 0])
                gyro = response.get("gyro", [0, 0, 0])
                if len(quat) != 4 or len(gyro) != 3:
                    logger.debug(f"{self.side}: Invalid IMU data format, using cache")
                    return self._cached()

                seq = response.get("seq")
                if seq is not None and self.last_seq is not None and seq > self.last_seq + 1:
                    self.n_dropped += seq - self.last_seq - 1
                self.last_seq = seq
                self.n_received += 1

                if self._dead_warned:
                    logger.info(f"✓ {self.side}: IMU song lai sau {now_mono - self.last_rx_mono:.1f}s")
                    self._dead_warned = False
                self.last_rx_mono = now_mono
                self.last_valid_time = time.time()

                imu_data = {
                    "quat": quat,
                    "gyro": gyro,
                    "t_sample": response.get("t_sample"),
                    "seq": seq,
                    "stale": False,
                }
                self.last_imu_data = imu_data
                return imu_data

        except Exception as e:
            logger.debug(f"{self.side}: Error in read_imu - {e}")
            return self._cached()

    def stats(self) -> Dict:
        """So lieu de script kiem tra in ra cuoi: nhan bao nhieu, roi bao nhieu."""
        return {"received": self.n_received, "dropped": self.n_dropped, "last_seq": self.last_seq}

    def close(self):
        """Close connection"""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
                self.socket = None
        self.context.term()


class IMUFusion:
    """API to fuse IMU data from 2 leg sensors (ban PUB/SUB - khong chan).

    left_port/right_port van la port REQ/REP cua server (5556/5555) de giu
    giao dien giong imu.py; port PUB thuc te = port + IMU_PUB_PORT_OFFSET.
    """

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
        """Read and fuse IMU data from 2 leg sensors.

        PUB/SUB: hai lenh read_imu() duoi day KHONG CHAN - chi moc mau moi
        nhat trong hop thu moi ben. Ham nay tra ve trong < 1ms du wifi nghen.
        """
        try:
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
                # seq do server danh - de script kiem tra dem mau roi
                "left_seq": left_raw.get("seq"),
                "right_seq": right_raw.get("seq"),
            }

        except Exception as e:
            logger.error(f"Error in get_fused_imu: {e}")
            return None

    def stats(self) -> Dict:
        """Nhan/roi bao nhieu mau moi ben, de in ket luan cuoi phien."""
        return {"left": self.left.stats(), "right": self.right.stats()}

    def close(self):
        """Close all connections"""
        self.left.close()
        self.right.close()
