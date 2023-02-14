"Common helper classes for pi car"""
import math
import time
from typing import Iterable, Tuple, Union

import numpy as np
import picar_4wd as fc


class Radar:
    """Control and read from the ultrasonic sensor"""

    def __init__(self, angle_range: int = 180, angle_step: int = 18):
        self.angle_range = angle_range
        self.angle_step = angle_step
        self.step_direction = 1
        self.current_angle: int = 0

        self.servo = fc.servo
        self.servo.set_angle(self.current_angle)

    def scan_step(self) -> Tuple[int, float]:
        """scan environment and return angle and distance

        Returns:
            Tuple[int, float]: tuple of angle and distance
        """
        max_angle = self.angle_range // 2
        min_angle = -max_angle

        self.current_angle += self.angle_step * self.step_direction
        if self.current_angle >= max_angle:
            self.current_angle = max_angle
            self.step_direction = -self.step_direction
        elif self.current_angle <= min_angle:
            self.current_angle = min_angle
            self.step_direction = -self.step_direction
        dist = self.get_distance_at(self.current_angle, sleep_duration=0.01)
        if dist < 0:
            dist = 100

        return self.current_angle, dist

    def get_distance_at(self, angle: float, sleep_duration: float = 0.04) -> float:
        """get distance at angle

        Args:
            angle (float): angle of the ultrasonic sensor in degrees
            sleep_duration (float, optional): duration for sensor to turn. Defaults to 0.04.

        Returns:
            float: _description_
        """
        self.servo.set_angle(angle)
        time.sleep(sleep_duration)
        distance = fc.us.get_distance()
        return distance


class Car:
    """Control the pi car movement while keeping track of position and direction"""
    _SPEED = 5
    _SPEED_SCALER = 2
    _TURN_SCALER = (17 / 15) / (2 * math.pi)

    def __init__(self, position: Iterable, dir_in_rad: float):
        self._position = np.array(position)
        self.curr_dir = dir_in_rad

        self.start_time: Union[float, None] = None

    def forward(self):
        """move forward"""
        if not self.start_time:
            fc.forward(self._SPEED)
            self.start_time = time.monotonic()

    def stop(self):
        """stop the car"""
        fc.stop()
        self._position = self.get_position()
        self.start_time = None
        time.sleep(0.5)

    def turn_relative(self, angle_in_rad: float):
        """turn the car by angle in radians

        Args:
            angle_in_rad (float): positive for left, negative for right
        """
        if angle_in_rad == 0:
            return

        if angle_in_rad > 0:
            fc.turn_left(self._SPEED)
        else:
            fc.turn_right(self._SPEED)

        time.sleep(abs(angle_in_rad) * self._SPEED * self._TURN_SCALER)
        fc.stop()
        time.sleep(0.5)

        self.curr_dir += angle_in_rad
        self.curr_dir %= 2 * math.pi

    def turn_absolute(self, dir_in_rad: float):
        """turn the car to face a direction in radians

        Args:
            dir_in_rad (float): direction in radians
        """
        diff = dir_in_rad - self.curr_dir
        self.turn_relative(math.copysign(abs(diff % math.pi), math.sin(diff)))

    def turn_absolute_deg(self, dir_in_deg: float):
        """turn the car to face a direction in degrees

        Args:
            dir_in_deg (float): _description_
        """
        self.turn_absolute(math.radians(dir_in_deg))

    def get_position(self) -> np.ndarray:
        """get current position of the car

        Returns:
            np.ndarray: position of the car in the form of [x, y]
        """
        if self.start_time is not None:
            duration = time.monotonic() - self.start_time
            dir_vec = np.array([np.cos(self.curr_dir),
                                np.sin(self.curr_dir)])
            return self._position + dir_vec * duration * self._SPEED * self._SPEED_SCALER
        else:
            return self._position.copy()
