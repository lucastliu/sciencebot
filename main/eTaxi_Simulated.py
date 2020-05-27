import random
import math

from eTaxiBase import eTaxiBase


class eTaxi_Simulated(eTaxiBase):

    # eTaxi Variables
    x_pos = 0.0
    y_pos = 1000.0
    adj_target_x_pos = 0.0
    adj_target_y_pos = 0.0
    heading = 0.0
    IMU_heading = 0.0

    # Error Vars
    MAX_POS_ERROR = 10
    MAX_IMU_ERROR_DEG = 0.5
    MAX_IMU_ERROR = (MAX_IMU_ERROR_DEG/360) * 2*math.pi
    ACCEPTABLE_TURN_ERROR_DEG = 3
    ACCEPTABLE_TURN_ERROR = math.radians(ACCEPTABLE_TURN_ERROR_DEG)

    TURN_SPEED = 100
    MOVE_SPEED = 100

    def __init__(self):
        print('Simulated eTaxi initialized')

    def get_position(self):
        rad_error = (random.randint(0, 100)/100) * 2*math.pi
        dist_error = random.randint(0, self.MAX_POS_ERROR)
        x_error = dist_error * math.cos(rad_error)
        y_error = dist_error * math.sin(rad_error)
        # print('x_pos_error: ', x_error, ' y_pos_error: ', y_error)
        return (self.x_pos + x_error), (self.y_pos + y_error)

    def move(self, dist):
        x_diff = dist * math.cos(self.heading)
        y_diff = dist * math.sin(self.heading)
        self.x_pos += x_diff
        self.y_pos += y_diff
        # print("x_pos: ", x_pos, " y_pos: ", y_pos)

    def turn_to_heading(self, rads):
        error = ((random.randint(0, 200) - 100) / 100) * self.MAX_IMU_ERROR
        self.heading = rads + error if rads + error < 2 * math.pi else rads + error - (2 * math.pi)

        # print('IMU skew is: ', error)
        # print('Heading: ', heading)

    def get_heading(self):
        return self.heading

    def set_true_position(self, new_x, new_y):
        self.x_pos = new_x
        self.y_pos = new_y

    def set_true_heading(self, new_heading):
        self.heading = new_heading

    def get_true_position(self):
        return self.x_pos, self.y_pos

    def get_true_heading(self):
        return self.heading

