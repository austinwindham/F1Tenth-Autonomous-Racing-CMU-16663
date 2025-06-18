#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        angle_increment = scan_msg.angle_increment

        ranges = np.array(scan_msg.ranges)
        
        ittc_ranges = []

        for n in range(len(ranges)):

            current_rdot = self.speed*np.cos(angle_min+n*angle_increment)
            # does this need to be negative


            current_range = ranges[n]
            if current_range > range_max:
                current_range = range_max
            elif current_range < range_min:
                current_range = range_min

            if current_rdot < 0:
        
                ittc_ranges.append(2000)
                continue

            ittc_ranges.append(current_range/max(current_rdot,0))

        


        # TODO: publish command to brake
        # set threshold 
        threshold = 1.5
        avg_fwd_reading = np.mean(np.array(ittc_ranges[530:550]))
    
        if avg_fwd_reading<threshold:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.publisher_drive.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()