import picar_4wd as fc
from common import *

SPEED = 20

def avoidance():
    scan_list = None
    while not scan_list:
        scan_list = fc.scan_step(35)
    tmp = scan_list[3:7]
    print(tmp)
    if tmp != [2, 2, 2, 2]:
        fc.turn_right(SPEED)
    else:
        fc.forward(SPEED)


def main():
    pass


if __name__ == '__main__':
    main()
