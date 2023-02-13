import math
import time

import numpy as np
import picar_4wd as fc

from common import Radar, Car


def main():
    _radar = Radar()
    car = Car(position=(0, 0),
              dir_in_rad=math.radians(90))

    # test car turning calibration
    car.turn_absolute_deg(0)
    car.turn_absolute_deg(270)
    assert car.curr_dir == math.radians(270)
    car.turn_absolute_deg(180)
    car.turn_absolute_deg(90)
    car.turn_absolute_deg(180)
    car.turn_absolute_deg(270)
    car.turn_absolute_deg(0)
    car.turn_absolute_deg(90)
    assert car.curr_dir == math.radians(90)

    # test car driving and position calibration
    for i in range(4):
        print(f'Edge: {i}')
        car.forward()
        time.sleep(0.5)
        print("Position:", car.get_position(),
              "Angle:", car.curr_dir * (180 / math.pi))
        time.sleep(0.5)
        car.stop()
        print("Position:", car.get_position(),
              "Angle:", car.curr_dir * (180 / math.pi))
        car.turn_relative(math.radians(90))
        print("Position:", car.get_position(),
              "Angle:", car.curr_dir * (180 / math.pi))


if __name__ == '__main__':
    try:
        main()
    finally:
        fc.stop()
