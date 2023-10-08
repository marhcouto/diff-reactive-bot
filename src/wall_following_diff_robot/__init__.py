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
        self.linear_speed = 0.5
        self.angular_speed = 1.5

        #!
        # Goal distance from wall
        self.ideal_distance = 0.2
        
        # **** Create publishers ****
        self.pub: Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)


    # Send a request to move a model
    def move_model(self, model_name, x, y, theta):
        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = model_name
        request.pose = Pose2D()
        request.pose.x = x
        request.pose.y = y
        request.pose.theta = theta
        client.call_async(request)


    #! 
    # @brief Control the robot to follow the wall
    # @param publisher Publisher to publish Twist messages
    # @param min_distance Minimum distance from the wall from a laser
    # @param angle_with_wall Angle with the wall
    # @return None
    def controlRobot(self, publisher: Publisher, min_distance: float, angle_with_wall: float):
        # Control the robot to follow the wall
        pass


    #!
    # @brief Process LiDAR information
    # @param data LiDAR data
    # @return None
    def processLiDAR(self, data: LaserScan):
        numpy_ranges = np.array(data.ranges)
        number_of_lasers = len(numpy_ranges)
        numpy_ranges = np.vectorize(lambda x: x if x > 0 else 100) (numpy_ranges) 
        min_distance_measurement, min_distance_index = numpy_ranges.min(), numpy_ranges.argmin()
        angle_with_wall = min_distance_index * data.angle_increment
        rclpy.logging.get_logger("SerpController").info("Index: " + str(min_distance_index) + " of " + str(number_of_lasers))
        rclpy.logging.get_logger("SerpController").info("Angle with wall: " + str(angle_with_wall * 180 / math.pi))
        self.controlRobot(self.pub, min_distance_measurement, angle_with_wall)


def main(args = None):
    rclpy.init()
    controller = SerpController()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
