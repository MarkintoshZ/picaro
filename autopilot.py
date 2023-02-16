from queue import Queue, Empty
from threading import Thread
import argparse
import math
import time
from typing import List, Tuple

import picar_4wd as fc

from map import Mapper, Ray
from common import Car, Radar


MAP_SIZE = 80


terminate_program = False
ignore_stop_sign = False


def navigate(
    path: List[Tuple[int, int]],
    car: Car,
    radar: Radar,
    queue: Queue,
) -> bool:
    """Attempt to navigate to the given path. Returns True if successful, False 
    otherwise"""
    global ignore_stop_sign

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

        prev_dist = math.inf
        while True:
            curr_car_pos = car.get_position().round().astype(int)
            print(curr_car_pos, car.curr_dir)
            dist = math.sqrt((curr_car_pos[0] - waypoint[0]) ** 2 +
                             (curr_car_pos[1] - waypoint[1]) ** 2)
            if prev_dist < dist:
                break
            prev_dist = min(prev_dist, dist)

            stop_sign_detected = False
            if queue is not None:
                print("fetch queue...", end=' ')
                has_stop_sign = None
                while True:
                    try:
                        has_stop_sign = queue.get_nowait()
                    except Empty:
                        break
                print('has stop sign =', has_stop_sign)
                if has_stop_sign is not None and has_stop_sign is True and \
                        not ignore_stop_sign:
                    ignore_stop_sign = True
                    stop_sign_detected = True
                    car.stop()
                    time.sleep(3)
                    car.forward()

            if not stop_sign_detected:
                for angle in range(-20, 20, 10):
                    dist = radar.get_distance_at(angle, sleep_duration=0.04)
                    if dist < 10:
                        # encountered obstacle, stop and return
                        car.stop()
                        return False
        car.stop()

    return True


def object_detection_cv(queue: Queue):
    global terminate_program

    from detect import run as detect
    detector = detect("efficientdet_lite0.tflite", 0, 640, 480, 4, False)
    for has_stop_sign in detector:
        print("[cv] has stop sign =", has_stop_sign)
        queue.put(has_stop_sign)
        if terminate_program or ignore_stop_sign:
            break
        time.sleep(0.5)


def main(object_detection: bool = False):
    global terminate_program

    queue = Queue()

    thread: Thread
    if object_detection:
        thread = Thread(target=object_detection_cv, args=(queue,)).start()

    radar = Radar()
    mapper = Mapper(size=MAP_SIZE, dist_cutoff=10, connect_cutoff=10)
    car = Car(position=(MAP_SIZE // 2, 20),
              dir_in_rad=math.radians(90))

    dest = (MAP_SIZE // 2, 50)
    while True:
        print("Scanning...")
        for _ in range(15):
            angle, dist = radar.scan_step()
            angle_in_rad = math.radians(-angle) + car.curr_dir
            angle_in_rad %= (2 * math.pi)
            position = car.get_position().round().astype(int)
            ray = Ray(tuple(position), angle_in_rad, round(dist / 5))
            mapper.add_ray(ray)
        print("Finding path...")
        path = mapper.route(car.get_position().round().astype(int), dest)
        mapper.plot(path=path, save_file=f"./debug/map-{time.time()}.jpg")
        if path is None:
            print("No path found!")
            return
        print("Following path...")
        if navigate(path, car, radar, queue):
            break
    print("Reached destination!")

    terminate_program = True
    thread.join()


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
