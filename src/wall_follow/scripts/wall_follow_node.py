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

        # TODO: create subscribers and publishers
        self.subscription_scan = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10)
        
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 1.5 # was 3 and owrked
        self.kd = 0.2
        self.ki = 0.0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.angle_min = -np.pi * 0.75
        self.angle_max = np.pi * 0.75
        self.angle_inc = (self.angle_max - self.angle_min)/1080

        self.desired_distance = 1.0
        self.look_ahead_distance = 1.0


    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        index = int((angle - self.angle_min) / self.angle_inc)

        if index < 0 or index >= len(range_data):
            return float('inf')  # If out of bounds, return a large distance
    
        # Handle NaN or infinite values
        if np.isnan(range_data[index]) or np.isinf(range_data[index]):
            return float('inf')
        
        return range_data[index]



    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        angle_1 = np.pi / 180 * 90
        angle_2 = np.pi / 180 * 45 # was 45

        theta = angle_1 - angle_2
        b_dist = self.get_range(range_data, angle_1)
        a_dist = self.get_range(range_data, angle_2)
        

        alpha = np.arctan2(a_dist * np.cos(theta) - b_dist, a_dist * np.sin(theta))
        Dt = b_dist * np.cos(alpha)
        Dtp1 = Dt + self.look_ahead_distance * np.sin(alpha)

        error = dist - Dtp1

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
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        dt = 1.0  # Small time step (adjustable)

        # Compute PID terms
        proportional = self.kp * error
        self.integral += error * dt
        derivative = self.kd * (error - self.prev_error) / dt

        # Compute steering angle
        angle = proportional + self.ki * self.integral + derivative

        # Store current error for next loop
        self.prev_error = error

        if abs(angle) <= np.pi / 180 * 10:
            velocity = 1.5
        elif abs(angle) <= np.pi / 180 * 20:
            velocity = 1.0
        else:
            velocity = 0.5



        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = -angle
        # print("velocity")
        # print(velocity)
        # print("angle")
        # print(angle)
        self.publisher_drive.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_inc = msg.angle_increment

        ranges = msg.ranges


        error = self.get_error(ranges, self.desired_distance) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    print(" we made it")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()