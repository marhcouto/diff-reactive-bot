#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MAX_ABSOLUTE_ANGULAR_SPEED = math.pi / 2


class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        # Declare used node parameters
        self.declare_parameters(namespace="", parameters=[
            ("linear_speed", rclpy.Parameter.Type.DOUBLE),
            ("ideal_distance", rclpy.Parameter.Type.DOUBLE),
            ("invert_direction", rclpy.Parameter.Type.BOOL),
            ("k", rclpy.Parameter.Type.DOUBLE),
            ("radius", rclpy.Parameter.Type.DOUBLE)
        ])

        #!
        # Predefined speed for the robot
        self.linear_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self.get_logger().info(f"linear speed: {self.linear_speed}")

        #!
        # Goal distance from wall
        self.ideal_distance = self.get_parameter("ideal_distance").get_parameter_value().double_value # 0.6 for isInFinalPos2, 0.9 for isInFinalPos1
        self.get_logger().info(f"ideal distance: {self.ideal_distance}")

        #!
        # Goal angle with wall
        self.is_direction_inverted = self.get_parameter("invert_direction").get_parameter_value().bool_value
        self.get_logger().info(f"direction inverted: {self.is_direction_inverted}")

        #!
        # Proportional constant
        self.k = self.get_parameter("k").get_parameter_value().double_value
        self.get_logger().info(f"k: {self.k}")

        #!
        # Goal angle with perpendicular to wall
        self.ideal_angle = math.pi / 2 if self.is_direction_inverted else -math.pi / 2

        #!
        # Predefined speed for the robot ?
        self.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.get_logger().info(f"radius: {self.radius}")

        # Create log file and set current instant
        self.log_file = open("log.txt", "w")
        self.initial_instant = time.time()

        # Prepare robot start
        self.stopped = False
        self.travelled = 0

        # **** Create publishers ****
        self.pub : Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)

        self.brake = 0


    #! 
    # @brief Control the robot to follow the wall
    # @param publisher Publisher to publish Twist messages
    # @param distance Minimum distance from the wall from a laser
    # @param angle_with_wall Angle with the wall
    # @return None
    def controlRobot(self, publisher : Publisher, distance : float, angle_with_wall : float):
        # Calculate errors
        self.brake = 0
        angle_error : float = angle_with_wall - self.ideal_angle
        distance_error : float = distance - self.ideal_distance

        # Create commands
        twist_msg : Twist = Twist()
        clamp = lambda n, minn, maxn: min(max(n, minn), maxn) # Clamp function
        # Angular velocity is proportional to the angle error and simetrically proportional to the distance error
        twist_msg.angular.z : float = clamp(
                angle_error * self.k + distance_error * self.k * (-1 if self.ideal_angle < 0 else 1),
                  -MAX_ABSOLUTE_ANGULAR_SPEED, MAX_ABSOLUTE_ANGULAR_SPEED) # Limit angular velocity
        twist_msg.linear.x : float = self.linear_speed / (1 + abs(angle_error))
        self.travelled += 1 # Collect statistics
        publisher.publish(twist_msg)


    #! 
    # @brief Stop the robot
    # @param publisher Publisher to publish Twist messages
    def stopRobot(self, publisher : Publisher):
        # self.stopped = True
        self.brake = self.brake + 1
        twist_msg : Twist = Twist()
        twist_msg.angular.z : float = 0.0

        self.get_logger().info('Vel: ' + str(self.linear_speed - self.brake * 0.1))
        if self.linear_speed - self.brake * 0.1 > 0:
            twist_msg.linear.x : float = self.linear_speed - self.brake * 0.1
        else:
            twist_msg.linear.x : float = 0.0

        publisher.publish(twist_msg)

        # Collect statistics
        # elapsed = time.time() - self.initial_instant
        # self.log_file.write(f"travelled: {self.travelled}\n")
        # self.log_file.write(f"elapsed: {elapsed}\n")
        # self.log_file.close()


    #! 
    # @brief Check if the robot is in the final position
    # @param distances Array of distances from the laser
    # @param min_distance_measurement Minimum distance from the laser
    # @param min_distance_index Index of the minimum distance from the laser
    def isInFinalPos1(self, distances: [float], min_distance_measurement : float, min_distance_index: float) -> bool:
        
        progress = 0
        angle_between_sensors = (2 * math.pi) / distances.size
        
        for i in range(distances.size):
            d = distances[i] + self.radius

            if progress == 0:
                if d < 99:
                    progress += 1
            
            if progress == 1:
                if d > 99:
                    progress += 1
                    continue
                
                straight_distance_to_wall = (min_distance_measurement + self.radius) / math.cos(abs(i - min_distance_index) * angle_between_sensors)
                if abs(straight_distance_to_wall - d) > 0.2 and abs(straight_distance_to_wall - d) < 3:
                    return False

            if progress == 2:
                if d < 99:
                    return False
        self.get_logger().info('Warning! Wall in the front!')
        return True

      
    #! 
    # @brief Check if the robot is in the final position, assuming that the final straight is short
    # @param distances Array of distances from the laser
    # @param min_distance_measurement Minimum distance from the laser
    # @param min_distance_index Index of the minimum distance from the laser
    def isInFinalPos2(self, distances: [float], min_distance_measurement : float, min_distance_index: float) -> bool:
        
        progress = 0
        angle_between_sensors = (2 * math.pi) / distances.size

        for i in range(distances.size - 1):
            if progress == 0:
                if distances[i] < 99:
                    progress += 1
            
            if progress == 1:
                if distances[i + 1] > 99:
                    progress += 1
                    continue

                distance_between_detected_points = math.sqrt((distances[i] * distances[i]) +
                                                    (distances[i + 1] * distances[i + 1]) -
                                                    2 * distances[i] * distances[i + 1] * math.cos(angle_between_sensors))

                if distance_between_detected_points > 2 * self.radius and distance_between_detected_points < 2:
                    return False

            if progress == 2:
                if distances[i] < 99:
                    return False
        self.get_logger().info('Warning! Wall in the front!')
        return True
    
    
    #!
    # @brief Process LiDAR information
    # @param data LiDAR data
    # @return None
    def processLiDAR(self, data : LaserScan):
        numpy_ranges = np.nan_to_num(np.array(data.ranges), nan=100000) # Replace NaNs with large integer
        min_distance_measurement, min_distance_index = numpy_ranges.min(), numpy_ranges.argmin() # Closest laser measurement
        angle_with_wall = min_distance_index * data.angle_increment + data.angle_min # Angle with wall perpendicular
        
        # if not self.isInFinalPos1(numpy_ranges, min_distance_measurement, min_distance_index):
        if not self.isInFinalPos2(numpy_ranges, min_distance_measurement, min_distance_index):
            self.controlRobot(self.pub, min_distance_measurement, angle_with_wall)
        else:
            self.stopRobot(self.pub)


def main(args = None):
    rclpy.init()
    controller = SerpController()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
