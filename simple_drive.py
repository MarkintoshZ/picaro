import picar_4wd as fc

from common import *


SPEED = 10


def main():
    while True:
        if [fc.get_status_at(-15), fc.get_status_at(15)] == [2, 2]:
            drive(Command.FORWARD, SPEED)
        else:
            drive(Command.STOP)
            left_dist = fc.get_status_at(-30)
            right_dist = fc.get_status_at(30)
            if left_dist > right_dist:
                if left_dist == 2:
                    drive(Command.LEFT_TURN, SPEED)
                else:
                    drive(Command.LEFT_SPIN, SPEED)
            else:
                if right_dist == 2:
                    drive(Command.RIGHT_TURN, SPEED)
                else:
                    drive(Command.RIGHT_SPIN, SPEED)


if __name__ == '__main__':
    main()

