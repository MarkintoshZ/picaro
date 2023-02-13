import math
import time
from typing import Iterable, Tuple, Union

import numpy as np
import picar_4wd as fc

from map import Mapper, Ray


class Radar:
    def __init__(self, angle_range: int = 180, angle_step: int = 18):
        self.angle_range = angle_range
        self.angle_step = angle_step
        self.step_direction = 1
        self.current_angle: int = 0

        self.servo = fc.servo
        self.servo.set_angle(self.current_angle)

    def scan_step(self) -> Tuple[int, float]:
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
        self.servo.set_angle(angle)
        time.sleep(sleep_duration)
        distance = fc.us.get_distance()
        return distance


class Car:
    _SPEED = 5
    _SPEED_SCALER = 2
    _TURN_SCALER = (17 / 30) / (2 * math.pi)

    def __init__(self, position: Iterable, dir_in_rad: float):
        self._position = np.array(position)
        self.curr_dir = dir_in_rad

        self.start_time: Union[float, None] = None

    def forward(self):
        if not self.start_time:
            fc.forward(self._SPEED)
            self.start_time = time.monotonic()

    def stop(self):
        fc.stop()
        self._position = self.get_position()
        self.start_time = None
        time.sleep(0.5)

    def turn_relative(self, angle_in_rad: float):
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
        diff = dir_in_rad - self.curr_dir
        self.turn_relative(math.copysign(abs(diff % math.pi), math.sin(diff)))

    def turn_absolute_deg(self, dir_in_deg: float):
        self.turn_absolute(math.radians(dir_in_deg))

    def get_position(self):
        if self.start_time is not None:
            duration = time.monotonic() - self.start_time
            dir_vec = np.array([np.cos(self.curr_dir),
                                np.sin(self.curr_dir)])
            return self._position + dir_vec * duration * self._SPEED * self._SPEED_SCALER
        else:
            return self._position.copy()


def main():
    MAP_SIZE = 200
    mapper = Mapper(size=MAP_SIZE, dist_cutoff=50, connect_cutoff=40)
    radar = Radar()
    car = Car(position=(MAP_SIZE // 2, 10),
              dir_in_rad=math.radians(90))

    # # test car turning calibration
    # car.turn_absolute_deg(0)
    # car.turn_absolute_deg(270)
    # assert car.curr_dir == math.radians(270)
    # car.turn_absolute_deg(180)
    # car.turn_absolute_deg(90)
    # car.turn_absolute_deg(180)
    # car.turn_absolute_deg(270)
    # car.turn_absolute_deg(0)
    # car.turn_absolute_deg(90)
    # assert car.curr_dir == math.radians(90)
    #
    # # test car driving and position calibration
    # for i in range(4):
    #     print(f'Edge: {i}')
    #     car.forward()
    #     time.sleep(0.5)
    #     print("Position:", car.get_position(),
    #           "Angle:", car.curr_dir * (180 / math.pi))
    #     time.sleep(0.5)
    #     car.stop()
    #     print("Position:", car.get_position(),
    #           "Angle:", car.curr_dir * (180 / math.pi))
    #     car.turn_relative(math.radians(90))
    #     print("Position:", car.get_position(),
    #           "Angle:", car.curr_dir * (180 / math.pi))

    # initial scan
    for _ in range(15):
        angle, dist = radar.scan_step()
        angle_in_rad = math.radians(-angle) + car.curr_dir
        angle_in_rad %= (2 * math.pi)
        position = car.get_position().round().astype(int)
        ray = Ray(tuple(position), angle_in_rad, round(dist / 2))
        mapper.add_ray(ray)

    start_time = time.monotonic()
    while True:
        car.forward()
        angle, dist = radar.scan_step()
        angle_in_rad = math.radians(-angle) + car.curr_dir
        angle_in_rad %= (2 * math.pi)
        position = car.get_position().round().astype(int)
        ray = Ray(tuple(position), angle_in_rad, round(dist / 2))
        mapper.add_ray(ray)

        if time.monotonic() - start_time > 5:
            break

    fc.stop()

    mapper.plot(save_file="./debug/map.jpg")
    print(mapper.rays)


if __name__ == '__main__':
    try:
        main()
    except:
        fc.stop()

