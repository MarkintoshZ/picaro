"Common helper functions and datatypes for picar_4wd"""

from enum import Enum
from typing import Tuple, Union

import picar_4wd as fc


__all__ = ["Command", "drive", "left_turn", "right_turn", "get_line_status"]


class Command(Enum):
    """Driving commands"""
    FORWARD = "forward"
    BACKWARD = "backword"
    LEFT_SPIN = "left-spin"
    LEFT_TURN = "left-turn"
    RIGHT_SPIN = "right-spin"
    RIGHT_TURN = "right-turn"
    STOP = "stop"


def left_turn(speed: int, diff: Union[int, None] = None):
    if not diff:
        diff = round(speed * 0.7)
    fc.left_front.set_power(speed - diff)
    fc.left_rear.set_power(speed - diff)
    fc.right_front.set_power(speed + diff)
    fc.right_rear.set_power(speed + diff)


def right_turn(speed: int, diff: Union[int, None] = None):
    if not diff:
        diff = round(speed * 0.7)
    fc.left_front.set_power(speed + diff)
    fc.left_rear.set_power(speed + diff)
    fc.right_front.set_power(speed - diff)
    fc.right_rear.set_power(speed - diff)


def drive(cmd: Command, speed: int = 20) -> None:
    """Execute driving command by setting motor speed"""
    if cmd == Command.STOP:
        fc.stop()
    func = {
        Command.FORWARD: fc.forward,
        Command.BACKWARD: fc.backward,
        Command.LEFT_SPIN: fc.turn_left,
        Command.LEFT_TURN: left_turn,
        Command.RIGHT_SPIN: fc.turn_right,
        Command.RIGHT_TURN: right_turn,
    }[cmd]
    func(speed)


def get_line_status(threshold: int = 400) -> Tuple[bool, bool, bool]:
    """Return a tuple of three bool representing whether left, middle, and 
    right sensors are detecting dark surfaces respectively by comparing the 
    sensor reading to the given threshold"""
    return (
        fc.gs2.read() < threshold,
        fc.gs1.read() < threshold,
        fc.gs0.read() < threshold,
    )
