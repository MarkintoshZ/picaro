import argparse
import math
import time
from typing import List, Tuple

import picar_4wd as fc

from map import Mapper
from common import Car


MAP_SIZE = 150


def navigate(path: List[Tuple[int, int]], car: Car, object_detection: bool):
    if object_detection:
        from detect import run as detect
        detector = detect("efficientdet_lite0.tflite", 0, 640, 480, 4, False)

    i = 0
    while i < len(path) - 1:
        curr_loc, next_loc = path[i], path[i+1]
        delta = (next_loc[0] - curr_loc[0], next_loc[1] - curr_loc[1])
        for j in range(i + 1, len(path)):
            prev_loc, _curr_loc = path[j-1], path[j]
            if _curr_loc[0] - prev_loc[0] != delta[0] or \
                    _curr_loc[1] - prev_loc[1] != delta[1]:
                break
            i = j
        waypoint = path[i]

        # find direction to waypoint
        dir_in_rad = math.atan2(
            waypoint[1] - curr_loc[1], waypoint[0] - curr_loc[0])
        print(math.degrees(dir_in_rad))
        car.turn_absolute(dir_in_rad)
        car.forward()

        while True:
            curr_car_pos = car.get_position().round().astype(int)
            print(curr_car_pos, car.curr_dir)
            if curr_car_pos[0] == waypoint[0] and curr_car_pos[1] == waypoint[1]:
                break

            if object_detection:
                has_stop_sign = next(detector)
                if has_stop_sign:
                    car.stop()
                    time.sleep(3)
                    car.forward()
            else:
                time.sleep(0.05)

        car.stop()


def main(object_detection: bool = False):
    mapper = Mapper(size=MAP_SIZE, dist_cutoff=50, connect_cutoff=40)
    car = Car(position=(12, 15),
              dir_in_rad=math.radians(90))

    path: List[Tuple[int, int]] = [
        (12, 15),
        (12, 80),
        (90, 80),
        (90, 120),
        (115, 120),
        (116, 120),
    ]  # TODO: get A* path from mapper
    navigate(path, car, object_detection)

    # path: List[Tuple[int, int]] = []  # TODO: get A* path from mapper
    # navigate(path, car, object_detection)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-o', '--objectDetection',
        help='Use object detection',
        required=False,
        default=False)
    args = parser.parse_args()

    try:
        main(args.objectDetection)
    finally:
        fc.stop()
