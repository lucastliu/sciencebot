from abc import ABC, abstractmethod

import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data


class ControllerBase(ABC):
    """
    Base Abstract Class for a Vehicle Control Node.
    Receives information from topic 
    Receives action client requests
    """
    def __init__(self, name, pose_type, pose_name, action_type, action_name):
        super().__init__(name)

        self.group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            node=self,
            action_type=action_type,
            action_name=action_name,
            execute_callback=self.action_callback,
            callback_group=self.group)

        self.subscription = self.create_subscription(
            msg_type=pose_type,
            topic=pose_source,
            callback=self.pose_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.group)

        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.angle_diff = 2 * math.pi

    def calculate_closest_turn(self):
        """
        Minimum angle needed to turn is never more than pi radians.
        No need to turn the long way around.
        """

        a = self.angle - self.steering_angle()
        if a > math.pi:
            a -= 2 * math.pi

        elif a < -1 * math.pi:
            a += 2 * math.pi

        self.angle_diff = a

    def angle_convert(self, a):
        """
        Convert raw angle heading (radians) to proper co-ordinate system
        """
        if a > math.pi:
            a *= -1

        else:
            a = 2*math.pi - a

        return a

    def get_distance(self):
        """
        Euclidean 2D
        """
        return math.sqrt(
            math.pow(self.x_dest - self.x, 2)
            + math.pow(self.y_dest - self.y, 2)
            )

    def steering_angle(self):
        """
        Angle vehicle should face to be on straight line to target.
        Note the python documentation for atan2, considers signs of inputs
        https://docs.python.org/2/library/math.html
        """
        return math.atan2(self.y_dest - self.y, self.x_dest - self.x)

    @abstractmethod
    def linear_correction(self):
        pass

    @abstractmethod
    def angular_correction(self):
        pass

    @abstractmethod
    def action_callback(self, goal_handle):
        pass

    @abstractmethod
    def pose_callback(self, pose):
        pass
