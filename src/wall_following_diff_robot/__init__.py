#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        # Declare used node parameters
        self.declare_parameters(namespace="", parameters=[
            ("linear_speed", rclpy.Parameter.Type.DOUBLE),
            ("ideal_distance", rclpy.Parameter.Type.DOUBLE),
            ("ideal_angle", rclpy.Parameter.Type.DOUBLE),
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
        self.ideal_angle = self.get_parameter("ideal_angle").get_parameter_value().double_value
        self.get_logger().info(f"ideal angle: {self.ideal_angle}")

        #!
        # Proportional constant
        self.k = self.get_parameter("k").get_parameter_value().double_value
        self.get_logger().info(f"k: {self.k}")

        self.max_absolute_angular_speed = math.pi / 2

        #!
        # Goal angle with perpendicular to wall
        self.ideal_angle = -math.pi / 2

        #!
        # Predefined speed for the robot ?
        self.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.get_logger().info(f"radius: {self.radius}")

        # **** Create publishers ****
        self.pub : Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)

        # Create log file and set current instant
        self.log_file = open("log.txt", "w")
        self.initial_instant = time.time()

        # Prepare robot start
        self.stopped = False
        self.travelled = 0

    #! 
    # @brief Control the robot to follow the wall
    # @param publisher Publisher to publish Twist messages
    # @param distance Minimum distance from the wall from a laser
    # @param angle_with_wall Angle with the wall
    # @return None
    def controlRobot(self, publisher : Publisher, distance : float, angle_with_wall : float):
        # Calculate errors
        angle_error : float = angle_with_wall - self.ideal_angle
        distance_error : float = distance - self.ideal_distance

        # Create commands
        twist_msg : Twist = Twist()
        clamp = lambda n, minn, maxn: min(max(n, minn), maxn) # Clamp function
        twist_msg.angular.z : float = clamp(
                angle_error * self.k - distance_error * self.k, -self.max_absolute_angular_speed, 
                self.max_absolute_angular_speed) # Limit angular velocity
        twist_msg.linear.x : float = self.linear_speed / (1 + abs(angle_error))
        self.travelled += 1
        publisher.publish(twist_msg)

    def stopRobot(self, publisher : Publisher):
        self.stopped = True
        twist_msg : Twist = Twist()
        twist_msg.angular.z : float = 0.0
        twist_msg.linear.x : float = 0.0
        publisher.publish(twist_msg)

        # Collect statistics
        elapsed = time.time() - self.initial_instant
        self.log_file.write(f"travelled: {self.travelled}\n")
        self.log_file.write(f"elapsed: {elapsed}\n")
        self.log_file.close()

        
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
                
                straight_distance = (min_distance_measurement + self.radius) / math.cos(abs(i - min_distance_index) * angle_between_sensors)
                if abs(straight_distance - d) > 0.2 and abs(straight_distance - d) < 3:
                    return False

            if progress == 2:
                if d < 99:
                    return False

        return True

      
    # Assuming that the final straight is short
    def isInFinalPos2(self, distances: [float], min_distance_measurement : float, min_distance_index: float) -> bool:
        
        progress = 0

        for i in range(distances.size - 1):
            if progress == 0:
                if distances[i] < 99:
                    progress += 1
            
            if progress == 1:
                if distances[i] > 99:
                    progress += 1
                    continue
                
                if abs(distances[i] - distances[i + 1]) > (1.5 * self.radius) and abs(distances[i] - distances[i + 1]) < 3:
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
        
        if not self.isInFinalPos1(numpy_ranges, min_distance_measurement, min_distance_index):
        # if not self.isInFinalPos2(numpy_ranges, min_distance_measurement, min_distance_index):
            self.controlRobot(self.pub, min_distance_measurement, angle_with_wall)
        else:
            self.stopRobot(self.pub)


def main(args = None):
    rclpy.init()
    controller = SerpController()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
