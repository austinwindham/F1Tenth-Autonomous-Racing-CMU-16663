#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy.linalg import norm
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # File for logging waypoints
        home = expanduser('~')
        self.file = open('/sim_ws/src/mpc/waypointsfinal.csv', 'w')

        # Subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.save_waypoint,
            10  # QoS profile (buffer size)
        )

        self.get_logger().info("Saving waypoints...")

    def save_waypoint(self, data):
        quaternion = [
            data.pose.pose.orientation.x, 
            data.pose.pose.orientation.y, 
            data.pose.pose.orientation.z, 
            data.pose.pose.orientation.w
        ]

        euler = euler_from_quaternion(quaternion)
        speed = norm([
            data.twist.twist.linear.x, 
            data.twist.twist.linear.y, 
            data.twist.twist.linear.z
        ], 2)

        if data.twist.twist.linear.x > 0.:
            self.get_logger().info(f"Speed: {data.twist.twist.linear.x}")

        # Log waypoint data
        self.file.write(f"{data.pose.pose.position.x}, {data.pose.pose.position.y}, {euler[2]}, {speed}\n")

    def shutdown(self):
        self.file.close()
        self.get_logger().info("Goodbye")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsLogger()
    atexit.register(node.shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
