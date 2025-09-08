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
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from .config_mini_mapper import MiniMapperConfig

logger = logging.getLogger(__name__)


class MiniMapper(Robot):
    """
    A differential drive robot with 2 drive servos and no robotic arm.
    Uses position-based control for differential drive with trailing castor.
    Designed for autonomous mapping and navigation with lidar integration.
    """

    config_class = MiniMapperConfig
    name = "mini_mapper"

    def __init__(self, config: MiniMapperConfig):
        super().__init__(config)
        self.config = config
        norm_mode_base = MotorNormMode.RANGE_M100_100
        
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                # Differential drive base - left wheel ID 7, right wheel ID 8
                "base_left_wheel": Motor(7, "sts3215", norm_mode_base),
                "base_right_wheel": Motor(8, "sts3215", norm_mode_base),
            },
            calibration=self.calibration,
        )
        self.base_motors = [motor for motor in self.bus.motors if motor.startswith("base")]
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "x.vel",
                "y.vel", 
                "theta.vel",
            ),
            float,
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return
        
        logger.info(f"\nRunning calibration of {self}")

        # For differential drive, we only need to calibrate the wheel motors
        motors = self.base_motors

        # Set wheels to position mode for calibration
        self.bus.disable_torque(self.base_motors)
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

        input("Ensure wheels are in neutral position and press ENTER....")

        # For wheel motors, we assume full rotation capability
        homing_offsets = dict.fromkeys(self.base_motors, 0)
        range_mins = dict.fromkeys(self.base_motors, 0)
        range_maxes = dict.fromkeys(self.base_motors, 4095)

        self.calibration = {}
        for name, motor in self.bus.motors.items():
            self.calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=homing_offsets[name],
                range_min=range_mins[name],
                range_max=range_maxes[name],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self):
        # Set-up wheel actuators for velocity mode
        self.bus.disable_torque()
        self.bus.configure_motors()
        
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)

        self.bus.enable_torque()

    def setup_motors(self) -> None:
        for motor in reversed(self.base_motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        """Convert degrees per second to raw motor velocity command."""
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        # Cap the value to fit within signed 16-bit range (-32768 to 32767)
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF  # 32767 -> maximum positive value
        elif speed_int < -0x8000:
            speed_int = -0x8000  # -32768 -> minimum negative value
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        """Convert raw motor velocity feedback to degrees per second."""
        steps_per_deg = 4096.0 / 360.0
        degps = raw_speed / steps_per_deg
        return degps

    def _body_to_wheel_raw(
        self,
        x: float,
        y: float,
        theta: float,
        wheel_radius: float = 0.03175,
        wheelbase: float = 0.112,
        max_raw: int = 3000,
    ) -> dict:
        """
        Convert desired body-frame velocities into wheel raw commands for differential drive.

        Parameters:
          x         : Linear velocity in x direction (m/s) - forward/backward
          y         : Linear velocity in y direction (m/s) - ignored for differential drive
          theta     : Rotational velocity (deg/s)
          wheel_radius: Radius of each wheel (meters)
          wheelbase : Distance between left and right wheels (meters)
          max_raw   : Maximum allowed raw command (ticks) per wheel

        Returns:
          A dictionary with wheel raw commands:
             {"base_left_wheel": value, "base_right_wheel": value}.

        Notes:
          - For differential drive, y velocity is ignored
          - Differential drive kinematics: v_left = x - (theta_rad * wheelbase/2)
                                          v_right = x + (theta_rad * wheelbase/2)
          - Raw commands are scaled if they exceed max_raw
        """
        # Convert rotational velocity from deg/s to rad/s
        theta_rad = theta * (np.pi / 180.0)
        
        # Differential drive kinematics
        # Left wheel velocity: forward velocity minus rotation component
        # Right wheel velocity: forward velocity plus rotation component
        v_left = x - (theta_rad * wheelbase / 2.0)
        v_right = x + (theta_rad * wheelbase / 2.0)
        
        # Convert wheel linear speeds to angular speeds (rad/s)
        omega_left = v_left / wheel_radius
        omega_right = v_right / wheel_radius
        
        # Convert to deg/s
        wheel_degps = np.array([omega_left * 180.0 / np.pi, omega_right * 180.0 / np.pi])
        
        # Scaling to respect max_raw limits
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale

        # Convert to raw integer commands
        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]

        return {
            "base_left_wheel": -wheel_raw[0],  # Invert left wheel to fix direction
            "base_right_wheel": wheel_raw[1],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed: int,
        right_wheel_speed: int,
        wheel_radius: float = 0.05,
        wheelbase: float = 0.112,
    ) -> dict[str, Any]:
        """
        Convert wheel raw velocity feedback back into body-frame velocities.

        Parameters:
          left_wheel_speed : Raw command for left wheel
          right_wheel_speed: Raw command for right wheel  
          wheel_radius     : Radius of each wheel (meters)
          wheelbase        : Distance between left and right wheels (meters)

        Returns:
          A dict with body velocities: {"x.vel": float, "y.vel": 0.0, "theta.vel": float}
        """
        # Convert raw commands to deg/s
        # Apply same inversion as in forward kinematics
        omega_left_degps = self._raw_to_degps(-left_wheel_speed)  # Invert left wheel
        omega_right_degps = self._raw_to_degps(right_wheel_speed)
        
        # Convert to rad/s
        omega_left_radps = omega_left_degps * np.pi / 180.0
        omega_right_radps = omega_right_degps * np.pi / 180.0
        
        # Convert to linear wheel speeds
        v_left = omega_left_radps * wheel_radius
        v_right = omega_right_radps * wheel_radius
        
        # Differential drive inverse kinematics
        x_vel = (v_left + v_right) / 2.0  # Forward velocity
        theta_vel_radps = (v_right - v_left) / wheelbase  # Angular velocity in rad/s
        theta_vel = theta_vel_radps * 180.0 / np.pi  # Convert to deg/s
        
        return {
            "x.vel": x_vel,
            "y.vel": 0.0,  # No y movement in differential drive
            "theta.vel": theta_vel,
        }

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read wheel velocities
        start = time.perf_counter()
        base_wheel_vel = self.bus.sync_read("Present_Velocity", self.base_motors)

        # Convert wheel velocities to body frame
        base_vel = self._wheel_raw_to_body(
            base_wheel_vel["base_left_wheel"],
            base_wheel_vel["base_right_wheel"],
        )

        obs_dict = base_vel

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command mini_mapper to move with specified velocities.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            dict: the action sent to the motors.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Extract velocity commands
        base_goal_vel = {k: v for k, v in action.items() if k.endswith(".vel")}

        # Convert body velocities to wheel commands
        base_wheel_goal_vel = self._body_to_wheel_raw(
            base_goal_vel["x.vel"], 
            base_goal_vel.get("y.vel", 0.0),  # y.vel should be 0 for differential drive
            base_goal_vel["theta.vel"]
        )

        # Send velocity commands to wheels
        self.bus.sync_write("Goal_Velocity", base_wheel_goal_vel)

        return base_goal_vel

    def stop_base(self):
        """Stop all wheel motors."""
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)
        logger.info("Base motors stopped")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")