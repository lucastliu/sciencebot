from abc import abstractmethod, ABC
import math


class eTaxiBase(ABC):

    @abstractmethod
    def get_position(self):
        pass

    def move(self, dist_cm):
        pass

    def turn_to_heading(self, rads):
        pass

    def get_heading(self):
        pass

    def angle_between_headings(self, angle_1, angle_2):
        wrapped_delta = abs(angle_1 - angle_2) % (2*math.pi)
        shortest_delta = 2*math.pi - wrapped_delta if wrapped_delta > math.pi else wrapped_delta
        sign = 1 if (angle_1 - angle_2 >= 0 and angle_1 - angle_2 <= math.pi) \
                    or (angle_1 - angle_2 <= -math.pi and angle_1 - angle_2 >= -2*math.pi) else -1
        shortest_delta *= sign
        return shortest_delta
