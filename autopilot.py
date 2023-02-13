import math
import time
from typing import List, Tuple

import picar_4wd as fc

from map import Mapper
from common import Car


MAP_SIZE = 200


def navigate(path: List[Tuple[int, int]], car: Car, object_detection: bool):
    i = 0
    while i < len(path) - 1:
        curr_loc, next_loc = path[i], path[i+1]
        delta = (next_loc[0] - curr_loc[0], next_loc[1] - curr_loc[1])
        for j in range(i, len(path)):
            prev_loc, _curr_loc = path[j-1], path[j]
            if _curr_loc[0] - prev_loc[0] != delta[0] or \
                    _curr_loc[1] - prev_loc[1] != delta[1]:
                break
            i = j
        waypoint = path[i]

        # find direction to waypoint
        dir_in_rad = math.atan2(waypoint[1] - curr_loc[0],
                                waypoint[0] - curr_loc[1])
        car.turn_absolute(dir_in_rad)

        while True:
            curr_car_pos = car.get_position().round().astype(int)
            if curr_car_pos[0] == waypoint[0] and curr_car_pos[1] == waypoint[1]:
                break

            if object_detection:
                pass
            else:
                time.sleep(0.1)

        i += 1


def main(object_detection: bool = False):
    mapper = Mapper(size=MAP_SIZE, dist_cutoff=50, connect_cutoff=40)
    car = Car(position=(MAP_SIZE // 2, 10),
              dir_in_rad=math.radians(90))

    path: List[Tuple[int, int]] = []  # TODO: get A* path from mapper
    navigate(path, car, object_detection)

    path: List[Tuple[int, int]] = []  # TODO: get A* path from mapper
    navigate(path, car, object_detection)


if __name__ == '__main__':
    try:
        main()
    finally:
        fc.stop()
