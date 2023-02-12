from numpy import random


def forward(speed):
    # print("Forward", speed)
    pass


def turn_left(speed):
    # print("Turn left", speed)
    pass


def turn_right(speed):
    # print("Turn right", speed)
    pass


def stop():
    # print("Stop")
    pass


class MockUS:
    def get_distance(self):
        return 30 + random.randint(0, 50)


us = MockUS()


class MockServo:
    def set_angle(self, angle):
        # print("Servo set angle", angle)
        pass


servo = MockServo()
