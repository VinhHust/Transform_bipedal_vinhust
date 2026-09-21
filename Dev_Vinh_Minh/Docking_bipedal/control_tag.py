# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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
"""
Utilities to control a LeKiwi robot remotely.

This script is designed to run the LeKiwi robot in remote mode, ensuring proper configuration
and safe disconnection.

Example of usage:

- Run the LeKiwi robot in remote mode:
```bash
python control_base.py \
    --robot.type=lekiwi \
    --control.type=remote_robot \
    --control.display_data=true \
    --control.viewer_ip=192.168.1.100 \
    --control.viewer_port=32323
```
"""
#12/06/2025
import sys
sys.path.append("/home/nam/Lekiwi_ws/src/lerobot")

import logging
import os
from dataclasses import asdict
from pprint import pformat

import rerun as rr

from lerobot.common.robot_devices.control_configs import (
    ControlConfig,
    ControlPipelineConfig,
    RemoteRobotConfig,
    TeleoperateControlConfig
)
from lerobot.common.robot_devices.robots.utils import Robot, make_robot_from_config
from lerobot.common.robot_devices.utils import safe_disconnect
from lerobot.common.utils.utils import init_logging
from lerobot.configs import parser
from lerobot.common.robot_devices.control_utils import (
    is_headless,
)

import argparse
import time
import json
import zmq
import numpy as np

def phase_2to5(cfg: ControlPipelineConfig):
    """
    Auto docking
    """
    import cv2
    import numpy as np
    from pupil_apriltags import Detector

    # IP and port of the robot
    ip = cfg.robot.ip
    port = cfg.robot.port

    # ZeroMQ PUSH socket 
    context = zmq.Context()
    cmd_socket = context.socket(zmq.PUSH)
    cmd_socket.connect(f"tcp://{ip}:{port}")
    cmd_socket.setsockopt(zmq.CONFLATE, 1)

    # Load camera calibration
    calib_data = np.load("/home/tatung/lerobot/lerobot/common/utils/arm1_calib_data.npz")
    camera_matrix = calib_data['mtx']
    dist_coeffs = calib_data['dist']

    TAG_SIZE = 0.034  # m
    half_size = TAG_SIZE / 2
    object_points = np.array([
        [-half_size, -half_size, 0],
        [ half_size, -half_size, 0],
        [ half_size,  half_size, 0],
        [-half_size,  half_size, 0],
    ], dtype=np.float32)

    at_detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0)

    cap = cv2.VideoCapture(0)
    CENTER_X = 320
    TOLERANCE_X = 40
    TOLERANCE_YAW = 6
    NEAR_DISTANCE = 0.05

    prev_cmd = "Searching..."
    recovery_mode = False
    recovery_direction = None

    def body_to_wheel_raw(x_cmd, y_cmd, theta_cmd, wheel_radius=0.05, base_radius=0.125, max_raw=3000):
        theta_rad = theta_cmd * (np.pi / 180.0)
        velocity_vector = np.array([x_cmd, y_cmd, theta_rad])
        angles = np.radians(np.array([300, 180, 60]))
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])
        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale
        wheel_raw = [
            int(round(abs(degps) * steps_per_deg)) | (0x8000 if degps < 0 else 0) for degps in wheel_degps
        ]
        return {"left_wheel": wheel_raw[2], "back_wheel": wheel_raw[0], "right_wheel": wheel_raw[1]}

    print("LeKiwi AprilTag auto-docking client started. Press Ctrl+C to quit.")

    try:
        docking_done = False
        phase_3 = False
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray)

            control_text = "Searching for tag..."
            tag_found = False

            if docking_done and not phase_3:
                # Robort has docked, move forward for 1.5 seconds
                control_text = "Move FORWARD after docking"
                x_cmd = 0.0
                y_cmd = -0.08  # Or any other value to move forward
                theta_cmd = 0.0
                print(f"[DEBUG] Move forward 1.5s after docking: x_cmd={x_cmd}, y_cmd={y_cmd}, theta_cmd={theta_cmd}")
                start_time = time.time()
                while time.time() - start_time < 1.5:
                    wheel_commands = body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
                    message = {"raw_velocity": wheel_commands}
                    cmd_socket.send_string(json.dumps(message))
                    time.sleep(0.033)
                # After moving forward, we stop the robot
                x_cmd = 0.0
                y_cmd = 0.0
                theta_cmd = 0.0
                wheel_commands = body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
                message = {"raw_velocity": wheel_commands}
                cmd_socket.send_string(json.dumps(message))
                phase_3 = True

                try:
                    # Create a new ZeroMQ context and socket for lekiwi2
                    context2 = zmq.Context()
                    cmd_socket2 = context2.socket(zmq.PUSH)
                    cmd_socket2.connect("tcp://172.20.10.4:5557")
                    cmd_socket2.setsockopt(zmq.CONFLATE, 1)

                    def body_to_wheel_raw(left, back, right, wheel_radius=0.05):
                        steps_per_rev = 4096
                        deg_per_rad = 180.0 / np.pi
                        steps_per_deg = steps_per_rev / 360.0
                        def encode(val):
                            degps = val * deg_per_rad
                            raw = int(round(abs(degps) * steps_per_deg))
                            if val < 0:
                                raw |= 0x8000
                            return raw
                        return {
                            "left_wheel": encode(left),
                            "back_wheel": encode(back),
                            "right_wheel": encode(right),
                        }

                    wheel_commands = body_to_wheel_raw(-0.866, -1.866, 3.5)
                    message = {"raw_velocity": wheel_commands}

                    print("Sending start autonomous signal to lekiwi2 for 15 seconds...")
                    start_time = time.time()
                    while time.time() - start_time <= 7.0:
                        cmd_socket2.send_string(json.dumps(message))
                        time.sleep(0.033)  # ~30Hz
                        
                    stop_commands = body_to_wheel_raw(0.0, 0.0, 0.0)
                    stop_message = {"raw_velocity": stop_commands}
                    for _ in range(30):  # Send 30 stop commands
                        cmd_socket2.send_string(json.dumps(stop_message))
                        time.sleep(0.033)
                        print("Sent STOP signal to lekiwi2.")

                    print("Done sending signal to lekiwi2.")
                    from control_arm import rest_phase
                    print("Recovering arm to rest position...")
                    rest_phase(cfg)

                    cmd_socket2.close()
                    context2.term()
                    
                except Exception as e:
                    print(f"Error sending signal to lekiwi2: {e}")
                continue
            
            if docking_done:
                # If docking is done, we stop the robot
                control_text = "Docking complete - STOP"
                x_cmd = 0.0
                y_cmd = 0.0
                theta_cmd = 0.0
                # Print debug info
                print(f"[DEBUG] z={z:.3f}, offset_x={offset_x:.2f}, yaw={yaw_deg:.2f}, "
                    f"x_cmd={x_cmd:.3f}, y_cmd={y_cmd:.3f}, theta_cmd={theta_cmd:.3f}, "
                    f"control_text={control_text}")
                wheel_commands = body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
                message = {"raw_velocity": wheel_commands}
                cmd_socket.send_string(json.dumps(message))
                time.sleep(0.033)
                continue

            # Default command
            x_cmd = 0.0
            y_cmd = 0.0
            theta_cmd = 0.0

            z = float('nan')
            offset_x = float('nan')
            yaw_deg = float('nan')

            for tag in tags:
                if tag.tag_id == 0:
                    tag_found = True
                    recovery_mode = False

                    corners = tag.corners.astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, dist_coeffs)
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        R[:, 0] *= -1
                        R[:, 2] *= -1

                        yaw_rad = np.arctan2(R[0, 2], R[2, 2])
                        yaw_deg = (np.degrees(yaw_rad) + 180) % 360 - 180

                        offset_x = tag.center[0] - CENTER_X
                        x, y, z = tvec.ravel()
                        if z < 0.2:
                            if offset_x >= -88:
                                control_text = "Slide LEFT"
                                x_cmd = -0.05
                            else:
                                control_text = "Docking complete - STOP"
                                # STOP
                                x_cmd = 0.0
                                y_cmd = 0.0
                                theta_cmd = 0.0
                                docking_done = True
                        elif abs(yaw_deg) > TOLERANCE_YAW:
                            control_text = "Turn LEFT" if yaw_deg > 0 else "Turn RIGHT"
                            theta_cmd = 15 if yaw_deg > 0 else -15
                        elif abs(offset_x) > TOLERANCE_X:
                            control_text = "Slide LEFT" if offset_x > 0 else "Slide RIGHT"
                            x_cmd = -0.05 if offset_x > 0 else 0.05
                        else:
                            control_text = "Move FORWARD"
                            y_cmd = -0.05
                    else:
                        control_text = "solvePnP failed"
                    break

            if not tag_found:
                # Near docking, if no tag found, handle recovery
                if prev_cmd == "Docking complete - STOP":
                    control_text = "Tag lost near docking - STOP"
                    x_cmd = 0.0
                    y_cmd = 0.0
                    theta_cmd = 0.0
                    recovery_mode = False
                else:
                    if not recovery_mode:
                        if "Turn RIGHT" in prev_cmd:
                            recovery_direction = "Slide LEFT"
                        elif "Turn LEFT" in prev_cmd:
                            recovery_direction = "Slide RIGHT"
                        else:
                            recovery_direction = "Searching..."
                        recovery_mode = True

                    control_text = f"{recovery_direction} to recover tag"
                    # Recovery: move sideways to search for tag
                    if recovery_direction == "Slide LEFT":
                        x_cmd = -0.05
                        y_cmd = 0.0
                        theta_cmd = 0.0
                    elif recovery_direction == "Slide RIGHT":
                        x_cmd = 0.05
                        y_cmd = 0.0
                        theta_cmd = 0.0
                    else:
                        x_cmd = 0.0
                        y_cmd = 0.0
                        theta_cmd = 15  # slow rotate


            prev_cmd = control_text

            print(f"[DEBUG] z={z:.3f}, offset_x={offset_x:.2f}, yaw={yaw_deg:.2f}, "
                  f"x_cmd={x_cmd:.3f}, y_cmd={y_cmd:.3f}, theta_cmd={theta_cmd:.3f}, "
                  f"control_text={control_text}")
            
            # Send cmd to robot
            wheel_commands = body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
            message = {"raw_velocity": wheel_commands}
            cmd_socket.send_string(json.dumps(message))

            time.sleep(0.033)  # ~30Hz

    except KeyboardInterrupt:
        print("Shutting down AprilTag auto-docking client.")
    finally:
        cap.release()
        cmd_socket.close()
        context.term()
        print("Client stopped.")
                    
def _init_rerun(control_config: ControlConfig, session_name: str = "lerobot_control_loop_remote") -> None:
    """Initializes the Rerun SDK for visualizing the control loop.

    Args:
        control_config: Configuration determining data display and robot type.
        session_name: Rerun session name. Defaults to "lerobot_control_loop_remote".

    Raises:
        ValueError: If viewer IP or port is missing for remote configurations with display enabled.
    """
    if (control_config.display_data and not is_headless()) or (
        control_config.display_data and isinstance(control_config, RemoteRobotConfig)
    ):
        # Configure Rerun flush batch size default to 8KB if not set
        batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
        os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size

        # Initialize Rerun based on configuration
        rr.init(session_name)
        if isinstance(control_config, RemoteRobotConfig):
            viewer_ip = control_config.viewer_ip
            viewer_port = control_config.viewer_port
            if not viewer_ip or not viewer_port:
                raise ValueError(
                    "Viewer IP & Port are required for remote config. Set via config file/CLI or disable control_config.display_data."
                )
            logging.info(f"Connecting to viewer at {viewer_ip}:{viewer_port}")
            rr.connect_tcp(f"{viewer_ip}:{viewer_port}")
        else:
            # Get memory limit for rerun viewer parameters
            memory_limit = os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%")
            rr.spawn(memory_limit=memory_limit)

@safe_disconnect
def run_lekiwi_remote(robot: Robot, cfg: RemoteRobotConfig):
    """Runs the LeKiwi robot in remote mode.

    Args:
        robot: The configured robot object.
        cfg: Configuration for remote robot control.
    """
    from lerobot.common.robot_devices.robots.lekiwi_remote import base_control

    _init_rerun(control_config=cfg, session_name="lerobot_control_loop_remote")
    base_control(robot.config, duration_s=3600.0, record_data=False)

@parser.wrap()
def control_base(cfg: ControlPipelineConfig):
    """Main function to control the LeKiwi robot in remote mode.

    Args:
        cfg: Configuration object containing robot and control parameters.
    """
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)

    if isinstance(cfg.control, RemoteRobotConfig):
        from lerobot.common.robot_devices.robots.lekiwi_remote import base_control
        _init_rerun(control_config=cfg.control, session_name="lerobot_control_loop_remote")
        base_control(cfg.robot, duration_s=3600.0, record_data=False)
    elif isinstance(cfg.control, TeleoperateControlConfig):
        phase_2to5(cfg)
    else:
        raise ValueError(f"Unsupported control type: {type(cfg.control)}")

    if robot.is_connected:
        robot.disconnect()

if __name__ == "__main__":
    control_base()