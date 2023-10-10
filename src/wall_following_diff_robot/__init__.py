#!/usr/bin/env python3
import math
import random
import numpy as np
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions


class SerpController(Node):

    def __init__(self) -> None:
        super().__init__("SerpController")

        #!
        # Predefined speed for the robot
        self.linear_speed = 0.2
        self.angular_speed = 1.5

        #!
        # Goal distance from wall
        self.ideal_distance = 0.2

        #!
        # Goal angle with wall
        self.ideal_angle = -math.pi / 2
        
        # **** Create publishers ****
        self.pub : Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)


    #! 
    # @brief Control the robot to follow the wall
    # @param publisher Publisher to publish Twist messages
    # @param distance Minimum distance from the wall from a laser
    # @param angle_with_wall Angle with the wall
    # @return None
    def controlRobot(self, publisher : Publisher, distance : float, angle_with_wall : float):
        # Control the robot to follow the wall
        # distance_error : float = distance - self.ideal_distance
        angle_error : float = angle_with_wall - self.ideal_angle
        twist_msg : Twist = Twist()
        k: float = 2 # Proportional constant
        twist_msg.angular.z : float = angle_error * k
        twist_msg.linear.x : float = self.linear_speed / (1 + abs(angle_error))
        publisher.publish(twist_msg)


    #!
    # @brief Process LiDAR information
    # @param data LiDAR data
    # @return None
    def processLiDAR(self, data : LaserScan):
        numpy_ranges = np.array(data.ranges)
        numpy_ranges = np.nan_to_num(numpy_ranges, nan=1000)
        min_distance_measurement, min_distance_index = numpy_ranges.min(), numpy_ranges.argmin()
        angle_with_wall = min_distance_index * data.angle_increment + data.angle_min
        self.controlRobot(self.pub, min_distance_measurement, angle_with_wall)


def main(args = None):
    rclpy.init()
    controller = SerpController()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
