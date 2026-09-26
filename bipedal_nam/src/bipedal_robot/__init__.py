# Copyright 2024 Nam. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

# Import LƯỜI (lazy) BipedalRobot/BipedalConfig.
#
# Lý do: bipedal.py kéo theo `lerobot` (driver motor Feetech), thứ chỉ có trên
# Pi. Laptop chỉ dùng transformer.py + sensors/imu.py, không đụng motor, nên
# không được bắt laptop cài lerobot chỉ để import được package này.
#
# Cách dùng KHÔNG đổi: `from bipedal_robot import BipedalRobot, BipedalConfig`
# trên Pi vẫn chạy y như cũ - chỉ khác là lerobot được import lúc gọi tên,
# không phải lúc import package.

__version__ = "0.1.0"
__all__ = ["BipedalConfig", "BipedalRobot"]


def __getattr__(name):
    if name in ("BipedalConfig", "BipedalRobot"):
        from . import bipedal

        return getattr(bipedal, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
