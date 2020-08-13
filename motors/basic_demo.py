import time

import RPi.GPIO as GPIO
import time
from SerialMotor import SerialMotor
from motor_constants import *


class BasicController(ControllerBase):
    """
    Controller implementation with
    just movement and smoothing function. Takes a list of moves.
    each item in list being [left_power, right_power, duration]
    """
    def __init__(self, moves):
        super().__init__()

        self.sm = SerialMotor("/dev/ttyACM1")
        self.old = [0.0, 0.0]
        self.curr = [0.0, 0.0]

        for x in moves:
            self.curr = [x[0], x[1]]
            self.T = x[2]
            self.move()

        self.sm.set_motor(3, 0)  # left
        self.sm.set_motor(4, 0)  # right

    def move(self):
        self.smoothing()
        print("Move: {}".format(str(self.curr)))
        if(abs(self.curr[0]) > 1 or abs(self.curr[1]) > 1):
            print('bad motor limit')
            print(str(self.curr))
            return
        self.sm.set_motor(3, self.curr[0])  # left
        self.sm.set_motor(4, self.curr[1])  # right
        time.sleep(self.T)

    def smoothing(self):
        self.curr[0] = .25*self.old[0] + .75*self.curr[0]
        self.curr[1] = .25*self.old[1] + .75*self.curr[1]
        self.old = self.curr  # update old


def main(args=None):

    moves = [
        [.6, .6, .5],
        [.8, -.8, .5],
        [.6, .6, .5],
        [-.8, .8, .5],
        [.6, .6, .5]
    ]

    bot = BasicController(moves=moves)


if __name__ == '__main__':
    main()
