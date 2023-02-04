import math
from time import sleep
from typing import List
from queue import Queue
from threading import Thread

import picar_4wd as fc

from common import *


SPEED = 20


def ultrasonic_sensor_scan(us_queue: Queue):
    while True:
        scan = scan_step(35, 3)
        if scan:
            print(scan)
        if scan and len(scan) >= 10:
            us_queue.put(scan)


def find_new_direction(us_queue: Queue):
    drive(Command.RIGHT_SPIN, SPEED)
    while True:
        if us_queue.not_empty:
            scan: List[int] = us_queue.get()
            if min(scan[3:7]) == 2:
                break
        sleep(0.1)
    drive(Command.STOP)
            

def main():
    us_queue = Queue()
    us_thread = Thread(target=ultrasonic_sensor_scan, args=(us_queue,))
    us_thread.start()

    while True:
        us_direction, gs_direction = 0, 0
        
        # get driving direction based on ultrasonic sensor
        if us_queue.not_empty:
            scan: List[int] = us_queue.get()
            left = min(scan[3:6])
            right = min(scan[5:8])
            if left > right and left == 0:
                us_direction = -2
            elif left > right:
                us_direction = -1
            elif left < right and right == 0:
                us_direction = 1
            elif left < right:
                us_direction = 2
            elif left == 0 and right == 0:
                find_new_direction(us_queue)

        # get driving direction based on grayscale sensor
        left, mid, right = get_line_status()
        if left and mid:
            gs_direction = -2
        elif left:
            gs_direction = -1
        elif right and mid:
            gs_direction = 1
        elif right:
            gs_direction = 2
        elif left and mid and right:
            drive(Command.BACKWARD, SPEED)
            sleep(1)
            drive(Command.STOP)

        # direction negotiation
        direction = 0
        prod = us_direction * gs_direction
        if prod < 0 and abs(prod) == 4:
            find_new_direction(us_queue)
        elif prod < 0:
            direction = (us_direction + gs_direction) * 2
        elif prod == 0:
            direction = us_direction + gs_direction
        else:
            direction = math.floor((us_direction + gs_direction) / 2)

        commands = [
                Command.FORWARD,
                Command.RIGHT_TURN,
                Command.RIGHT_SPIN,
                Command.LEFT_SPIN,
                Command.LEFT_TURN,
                ]
        drive(commands[direction], SPEED)


if __name__ == '__main__':
    main()

