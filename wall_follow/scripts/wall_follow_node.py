#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.subscription = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # set PID gains
        self.kp = 1.0
        self.kd = 0.1
        self.ki = 0.0

        # store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds / 1e9

        self.L = 0.8 #lookahead distance
        self.desired_dist = 1.0 #desired distance to the wall

        self.angle_min = 0.0
        self.angle_increment = 0.0
        
        # We want to look at a small window of indices (e.g., 5 indices to the left and right)
        # In F1TENTH, 10 indices is roughly a 2 to 3-degree sweep depending on the LiDAR
        self.window_size = 5 

    # def get_range(self, range_data, angle):
    #     """
    #     Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

    #     Args:
    #         range_data: single range array from the LiDAR
    #         angle: angle to look for the range


    #     Returns:
    #         range: range measurement in meters at the given angle

    #     """
    #     index = int((angle - self.angle_min) / self.angle_increment)
    #     if index < 0 or index >= len(range_data):
    #         return 3.0
        
    #     range_val = range_data[index]
    #     if np.isnan(range_val) or np.isinf(range_val):
    #         return 3.0
        
    #     return range_val

    def get_range(self, range_data, angle):
        # Find the center index for our target angle
        center_index = int((angle - self.angle_min) / self.angle_increment)
        
        start_index = max(0, center_index - self.window_size)
        end_index = min(len(range_data), center_index + self.window_size + 1)
        
        # Slice out that small cone of lasers
        window = range_data[start_index:end_index]
        
        # Filter out the invalid inf and nan readings
        valid_ranges = [r for r in window if not np.isinf(r) and not np.isnan(r)]
            
        # Return the MINIMUM distance found in that slice!
        return min(valid_ranges)
    
    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        thetha = np.pi / 4
        angle_b = np.pi / 2.0
        angle_a = np.pi / 4

        # 2. Extract laser distances
        b = self.get_range(range_data, angle_b)
        a = self.get_range(range_data, angle_a)

        numerator =  (a * np.cos(thetha)) - b
        denominator = (a * np.sin(thetha))

        alpha = np.arctan2(numerator, denominator)
        
        Dt= b * np.cos(alpha)
        Dt_plus_1= Dt + (self.L * np.sin(alpha))
        

        error = Dt_plus_1 - dist

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # 1. Calculate Time Step (dt)
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.prev_time
        if dt <= 0.0: 
            dt = 0.01 # Prevent division by zero on the very first loop

        # 2. Calculate PID
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        max_angle = 0.4 
        angle = np.clip(angle, -max_angle, max_angle)

        # Update memory for next loop
        self.prev_error = error
        self.prev_time = current_time

        # 3. Calculate Speed based on steering angle (Per README requirements)
        abs_angle = abs(angle)
        if abs_angle < np.radians(10):
            velocity = 1.5
        elif abs_angle < np.radians(20):
            velocity = 1.0
        else:
            velocity = 0.5

        # 4. Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = float(velocity)
        self.publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # Save LiDAR specs for get_error to use
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        error = self.get_error(msg.ranges, self.desired_dist)
        
        # We pass 0.0 for velocity because pid_control will calculate the true safe speed
        self.pid_control(error, 0.0)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()