#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


MAX_ABSOLUTE_ANGULAR_SPEED = math.pi / 2
MAX_ABSOLUTE_LINEAR_ACCELERATION = 0.5

clamp = lambda n, minn, maxn: min(max(n, minn), maxn) # Clamp function, used to limit control variables

class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        # Declare used node parameters
        self.declare_parameters(namespace="", parameters=[
            ("target_velocity", rclpy.Parameter.Type.DOUBLE),
            ("ideal_distance", rclpy.Parameter.Type.DOUBLE),
            ("invert_direction", rclpy.Parameter.Type.BOOL),
            ("detection_by_line", rclpy.Parameter.Type.BOOL),
            ("k_ang", rclpy.Parameter.Type.DOUBLE),
            ("k_lin", rclpy.Parameter.Type.DOUBLE)
        ])

        #!
        # Target speed for the robot
        self.target_velocity : float = self.get_parameter("target_velocity").get_parameter_value().double_value
        self.get_logger().info(f"target velocity: {self.target_velocity}")
        
        #!
        # Current speed of the robot
        self.current_velocity : float = 0.0

        #!
        # Goal distance from wall
        self.ideal_distance : float = self.get_parameter("ideal_distance").get_parameter_value().double_value # 0.6 for isInFinalPos2, 0.9 for isInFinalPos1
        self.get_logger().info(f"ideal distance: {self.ideal_distance}")

        #!
        # Goal angle with wall
        self.is_direction_inverted = self.get_parameter("invert_direction").get_parameter_value().bool_value
        self.get_logger().info(f"direction inverted: {self.is_direction_inverted}")

        #!
        # Goal angle with wall
        self.is_detection_by_line = self.get_parameter("detection_by_line").get_parameter_value().bool_value
        self.get_logger().info(f"Detection by line: {self.is_detection_by_line}")

        #!
        # Proportional constants
        self.k_ang : float = self.get_parameter("k_ang").get_parameter_value().double_value
        self.get_logger().info(f"k_ang: {self.k_ang}")
        self.k_lin : float = self.get_parameter("k_lin").get_parameter_value().double_value
        self.get_logger().info(f"k_lin: {self.k_lin}")

        #!
        # Goal angle with perpendicular to wall
        self.ideal_angle : float = math.pi / 2 if self.is_direction_inverted else -math.pi / 2

        # Create log file and set current instant
        self.log_file = open("log.txt", "w")
        self.initial_instant = time.time()

        # Prepare robot start
        self.stopped = False

        # **** Create publishers ****
        self.pub : Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)
        self.create_subscription(Odometry, "/odom", self.processOdometry, 1)

        self.brake = 0

        # metrics
        self.distances_to_wall = np.array([])


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
        # Limit velocity when turning, proportional to the angle error and target velocity
        target_speed : float = self.target_velocity / (1 + self.target_velocity * abs(angle_error))
        velocity_error : float = self.current_velocity - target_speed

        # Create commands
        twist_msg : Twist = Twist()
        # Angular velocity is proportional to the angle error and simetrically proportional to the distance error
        twist_msg.angular.z : float = clamp(
                angle_error * self.k_ang + distance_error * self.k_ang * (-1 if self.ideal_angle < 0 else 1),
                  -MAX_ABSOLUTE_ANGULAR_SPEED, MAX_ABSOLUTE_ANGULAR_SPEED)
        twist_msg.linear.x : float = clamp(-velocity_error * self.k_lin, -MAX_ABSOLUTE_LINEAR_ACCELERATION,
                    MAX_ABSOLUTE_LINEAR_ACCELERATION) # Linear velocity is proportional to the velocity error
        
        # Publish commands
        publisher.publish(twist_msg)


    #! 
    # @brief Stop the robot
    # @param publisher Publisher to publish Twist messages
    def stopRobot(self, publisher : Publisher):
        twist_msg : Twist = Twist()
        twist_msg.angular.z : float = 0.0
        twist_msg.linear.x : float = clamp(-self.current_velocity * self.k_lin, -MAX_ABSOLUTE_LINEAR_ACCELERATION,
                    MAX_ABSOLUTE_LINEAR_ACCELERATION)

        # Slow down
        if self.current_velocity > 0.001:
            twist_msg.linear.x : float = clamp(-self.current_velocity * self.k_lin, -MAX_ABSOLUTE_LINEAR_ACCELERATION,
                    MAX_ABSOLUTE_LINEAR_ACCELERATION)
            publisher.publish(twist_msg)
        elif self.current_velocity < -0.001:
            twist_msg.linear.x : float = clamp(self.current_velocity * self.k_lin, -MAX_ABSOLUTE_LINEAR_ACCELERATION,
                    MAX_ABSOLUTE_LINEAR_ACCELERATION)
            publisher.publish(twist_msg)
        else:
            self.get_logger().info('Robot stopped!')
            self.stopped = True

        # Collect statistics
        if self.stopped:
            self.collectStatistics()


    #! 
    # @brief Check if the robot is in the final position
    # @param distances Array of distances from the laser
    # @param min_distance_measurement Minimum distance from the laser
    # @param min_distance_index Index of the minimum distance from the laser
    def isInFinalPosByStraightLine(self, distances: [float], min_distance_measurement : float, 
                      min_distance_index: float, angle_increment : float) -> bool:
        
        progress = 0

        for i in range(distances.size):
            d = distances[i]

            if progress == 0:
                if d < 99:
                    progress += 1
            
            if progress == 1:
                if d > 99:
                    progress += 1
                    continue
                
                distance_to_wall = (min_distance_measurement) / math.cos((abs(i - min_distance_index) * angle_increment))
                if abs(distance_to_wall - d) > 0.3 and abs(distance_to_wall - d) < 3:
                    return False

            if progress == 2:
                if d < 99:
                    return False
        # self.get_logger().info('Warning! Wall in the front!')
        return True

      
    #! 
    # @brief Check if the robot is in the final position, assuming that the final straight is short
    # @param distances Array of distances from the laser
    # @param min_distance_measurement Minimum distance from the laser
    # @param min_distance_index Index of the minimum distance from the laser
    def isInFinalPosByDistance(self, distances: [float], angle_increment : float) -> bool:
        
        progress = 0

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
                                                    2 * distances[i] * distances[i + 1] * math.cos(angle_increment))

                if distance_between_detected_points > 0.15 and distance_between_detected_points < 2:
                    return False

            if progress == 2:
                if distances[i] < 99:
                    return False
        # self.get_logger().info('Warning! Wall in the front!')
        return True
    

    #!
    # @brief Process odometry information
    # @param data Odometry data
    # @return None
    def processOdometry(self, data : Odometry):
        if self.stopped: return
        self.current_velocity = math.sqrt(math.pow(data.twist.twist.linear.x, 2) + math.pow(data.twist.twist.linear.y, 2))


    #!
    # @brief Process LiDAR information
    # @param data LiDAR data
    # @return None
    def processLiDAR(self, data : LaserScan):
        if self.stopped: return

        numpy_ranges = np.nan_to_num(np.array(data.ranges), nan=100000) # Replace NaNs with large integer
        min_distance_measurement, min_distance_index = numpy_ranges.min(), numpy_ranges.argmin() # Closest laser measurement
        angle_with_wall = min_distance_index * data.angle_increment + data.angle_min # Angle with wall perpendicular

        self.distances_to_wall = np.append(self.distances_to_wall, [min_distance_measurement])

        if self.is_detection_by_line and not self.isInFinalPosByStraightLine(numpy_ranges, min_distance_measurement, min_distance_index, data.angle_increment):
            self.controlRobot(self.pub, min_distance_measurement, angle_with_wall)
        elif not self.is_detection_by_line and not self.isInFinalPosByDistance(numpy_ranges, data.angle_increment):
            self.controlRobot(self.pub, min_distance_measurement, angle_with_wall)
        else:
            self.stopRobot(self.pub)


    #!
    # @brief Collect and write statistics to a file
    # @return None
    def collectStatistics(self):
        self.get_logger().info('Reached final position! Collecting statistics...')
        elapsed = time.time() - self.initial_instant
        self.log_file.write(f"travelled: {self.distances_to_wall.size}\n")
        self.log_file.write(f"mean_distance_to_wall: {self.distances_to_wall.mean()}\n")
        self.log_file.write(f"elapsed: {elapsed}\n")
        self.log_file.close()


def main():
    rclpy.init()
    controller = SerpController()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
