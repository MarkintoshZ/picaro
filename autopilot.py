import argparse
import math
import time
from typing import List, Tuple

import picar_4wd as fc

from map import Mapper, Ray
from common import Car, Radar


MAP_SIZE = 150


def navigate(
    path: List[Tuple[int, int]],
    car: Car,
    radar: Radar,
    object_detection: bool
) -> bool:
    """Attempt to navigate to the given path. Returns True if successful, False 
    otherwise"""
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
            dist = math.sqrt((curr_car_pos[0] - waypoint[0]) ** 2 +
                             (curr_car_pos[1] - waypoint[1]) ** 2)
            if dist < 1:
                break

            for angle in range(-20, 20, 10):
                dist = radar.get_distance_at(angle, sleep_duration=0.04)
                if dist < 20:
                    # encountered obstacle, stop and return
                    car.stop()
                    return False

            if object_detection:
                has_stop_sign = next(detector)
                if has_stop_sign:
                    car.stop()
                    time.sleep(3)
                    car.forward()
            else:
                time.sleep(0.05)

        car.stop()

        return True


def main(object_detection: bool = False):
    radar = Radar()
    mapper = Mapper(size=MAP_SIZE, dist_cutoff=20, connect_cutoff=20)
    car = Car(position=(MAP_SIZE // 2, 25),
              dir_in_rad=math.radians(90))

    dest = (MAP_SIZE // 2, 80)
    while True:
        print("Scanning...")
        for _ in range(10):
            radar.get_distance_at(0, sleep_duration=0.05)
            angle, dist = radar.scan_step()
            angle_in_rad = math.radians(-angle) + car.curr_dir
            angle_in_rad %= (2 * math.pi)
            position = car.get_position().round().astype(int)
            ray = Ray(tuple(position), angle_in_rad, round(dist / 4))
            mapper.add_ray(ray)
        print("Finding path...")
        path = mapper.route(car.get_position().round().astype(int), dest)
        mapper.plot(path=path, save_file=f"./debug/map-{time.time()}.jpg")
        if path is None:
            print("No path found!")
            return
        print("Following path...")
        if navigate(path, car, radar, object_detection):
            break
    print("Reached destination!")


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
    except Exception as e:
        print(e)
    finally:
        fc.stop()
