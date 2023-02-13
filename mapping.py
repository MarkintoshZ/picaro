import math
import time

import picar_4wd as fc

from map import Mapper, Ray
from common import Radar, Car


def main():
    MAP_SIZE = 600
    mapper = Mapper(size=MAP_SIZE, dist_cutoff=7, connect_cutoff=7)
    radar = Radar()
    car = Car(position=(MAP_SIZE // 2, MAP_SIZE // 2),
              dir_in_rad=math.radians(90))

    # initial scan
    for _ in range(15):
        angle, dist = radar.scan_step()
        angle_in_rad = math.radians(-angle) + car.curr_dir
        angle_in_rad %= (2 * math.pi)
        position = car.get_position().round().astype(int)
        ray = Ray(tuple(position), angle_in_rad, round(dist / 4))
        mapper.add_ray(ray)

    start_time = time.monotonic()
    while True:
        car.forward()
        angle, dist = radar.scan_step()
        angle_in_rad = math.radians(-angle) + car.curr_dir
        angle_in_rad %= (2 * math.pi)
        position = car.get_position().round().astype(int)
        ray = Ray(tuple(position), angle_in_rad, round(dist / 4))
        mapper.add_ray(ray)

        if time.monotonic() - start_time > 5:
            break

    fc.stop()

    mapper.plot(save_file="./debug/map.jpg")
    print(len(mapper.rays))
    print(mapper.rays)


if __name__ == '__main__':
    try:
        main()
    finally:
        fc.stop()
