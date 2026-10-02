#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from typing import Any, Dict

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .car_mode import wheel_bus  # import file wheel_bus từ car_mode

logger = logging.getLogger(__name__)


class BipedalConfig:
    """Configuration for Bipedal Robot"""

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 1_000_000,
        use_degrees: bool = False,
        disable_torque_on_disconnect: bool = True,
    ):
        self.port = port
        self.baudrate = baudrate
        self.use_degrees = use_degrees
        self.disable_torque_on_disconnect = disable_torque_on_disconnect


class BipedalRobot:
    """
    Bipedal robot with 4 motors controlled via FeetechMotorsBus.
    Motor config: 4(sts3095), 5(sts3095), 7(sts3095), 8(sts3215)
    """

    def __init__(self, config: BipedalConfig = None):
        if config is None:
            config = BipedalConfig()
        self.config = config
        norm_mode_body = (
            MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        )
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                # base
                "base_right_wheel": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_left_wheel": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                # leg
                "bubright_joint": Motor(4, "sts3215", norm_mode_body),
                "hipright_joint": Motor(5, "sts3215", norm_mode_body),
                "twistright_joint": Motor(6, "sts3215", norm_mode_body),
                "kneeright_joint": Motor(7, "sts3215", norm_mode_body),
                "footright_joint": Motor(8, "sts3215", norm_mode_body),
                "gripperright_joint": Motor(9, "sts3215", norm_mode_body),
            },
        )
        self.base_motors = [motor for motor in self.bus.motors if motor.startswith("base")]
        self.leg_motors = [motor for motor in self.bus.motors if motor.startswith("leg")]

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "bub_joint.pos",
                "hip_joint.pos",
                "twist_joint.pos",
                "knee_joint.pos",
                "foot_joint.pos",
                "gripper_joint.pos",
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            self.bus.connect()
        except Exception as e:
            # Bỏ qua lỗi firmware version mismatch
            if "firmware" in str(e).lower():
                logger.warning(f"⚠️  Firmware version mismatch (ignored): {e}")
                # Vẫn tiếp tục kết nối
                pass
            else:
                raise

    def configure(self):
        # Set-up arm actuators (position mode)
        # We assume that at connection time, arm is in a rest position,
        # and torque can be safely disabled to run calibration.
        self.bus.disable_torque()
        for name in self.leg_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus.write("P_Coefficient", name, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 32)

        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self.stop_base()
        self.bus.enable_torque()

    def stop_base(self):
        wheel_bus.stop(self.bus)
        logger.info("Base motors stopped")

    def write_pos_ex(
        self,
        motor_name: str,
        position: int,
        speed: int = 1000,
        acceleration: int = 50,
        normalize: bool = False,
    ) -> None:
        """
        Giống SCServo WritePosEx: set position + speed + acceleration.

        Args:
            motor_name: Motor name (e.g., "leg_knee_right")
            position: Target position (raw ticks hoặc normalized value)
            speed: Goal velocity (raw value)
            acceleration: Acceleration (raw value)
            normalize: If True, convert position from normalized [-100, 100] to raw ticks
        """
        # 1) Set acceleration
        try:
            self.bus.write("Acceleration", motor_name, acceleration, normalize=False)
        except Exception as e:
            logger.warning(f"Failed to set acceleration: {e}")

        # 2) Set speed
        try:
            self.bus.write("Goal_Velocity", motor_name, speed, normalize=False)
        except Exception as e:
            logger.warning(f"Failed to set speed: {e}")

        # 3) Set position
        self.bus.write("Goal_Position", motor_name, position, normalize=normalize)

    # GỬI GÓI DATA CHO ĐỘNG CƠ ĐỒNG THỜI 1 LÚC THAY VÌ CHẠY TRONG FOR LOOP
    def write_leg_positions_sync(
        self,
        positions: Dict[str, int],
        speed: int = 1000,
        acceleration: int = 50,
    ) -> None:
        """
        Ghi đồng bộ vị trí, tốc độ và gia tốc cho nhiều khớp cùng lúc bằng
        sync_write.
        """
        if not positions:
            return

        motor_names = list(positions.keys())

        # 1) Set acceleration (đồng loạt cho các motor)
        try:
            self.bus.sync_write("Acceleration", {m: acceleration for m in motor_names})
        except Exception as e:
            logger.warning(f"Failed to set sync acceleration: {e}")

        # 2) Set speed (đồng loạt cho các motor)
        try:
            self.bus.sync_write("Goal_Velocity", {m: speed for m in motor_names})
        except Exception as e:
            logger.warning(f"Failed to set sync speed: {e}")

        # 3) Set position
        # LƯU Ý QUAN TRỌNG: Phải set normalize=False vì positions là raw ticks.
        # num_retry=5 giúp tăng tính ổn định nếu có nhiễu bus.
        self.bus.sync_write("Goal_Position", positions, normalize=False, num_retry=5)

    def read_motor_position(self, motor_name: str, normalize: bool = False) -> int:
        """Read current position of motor"""
        try:
            return self.bus.read("Present_Position", motor_name, normalize=normalize)
        except Exception as e:
            logger.error(f"Failed to read position: {e}")
            return None

    def read_leg_positions(self, normalize: bool = False) -> Dict[str, int]:
        """Read all leg motor positions"""
        leg_motors = ["leg_bub", "leg_hip", "leg_twist", "leg_knee", "leg_foot"]
        positions = {}
        for motor_name in leg_motors:
            if motor_name in self.bus.motors:
                positions[motor_name] = self.read_motor_position(motor_name, normalize)
        return positions

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)

        logger.info(f"{self} disconnected.")
