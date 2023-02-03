"Common helper functions and datatypes for picar_4wd"""

from enum import Enum

import picar_4wd as fc


class Command(Enum):
    """Driving commands"""
    FORWARD = "forward"
    BACKWARD = "backword"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"


def drive(cmd: Command, speed: int) -> None:
    """Execute driving command by setting motor speed"""
    func = {
        Command.FORWARD: fc.forward,
        Command.BACKWARD: fc.backward,
        Command.LEFT: fc.turn_left,
        Command.RIGHT: fc.turn_right,
    }[cmd]
    func(speed)


class LineStatus(Enum):
    """Greyscale sensor readings"""
    CLEAR = "clear"
    CENTER = "center"
    LEFT = "left"
    RIGHT = "right"


def line_status():
    gs_list = fc.get_grayscale_list()
    status = fc.get_line_status(400, gs_list)
    if status is None:
        return LineStatus.CLEAR
    return [LineStatus.CENTER, LineStatus.RIGHT, LineStatus.LEFT][status]


__all__ = [Command, drive, LineStatus, line_status]
