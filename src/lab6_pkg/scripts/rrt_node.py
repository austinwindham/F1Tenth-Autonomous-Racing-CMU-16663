#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""

import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need

from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from scipy.spatial.transform import Rotation as R

import csv





# class def for tree nodes
# It's up to you if you want to use this
class TreeNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        Node.__init__(self, 'rrt_node')  ## i added this
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        self.path_pub_ = self.create_publisher(Path, '/planned_path', 1)

        self.marker_pub_ = self.create_publisher(MarkerArray, '/rrt_visualization', 1)

        self.occ_grid_pub_ = self.create_publisher(OccupancyGrid, '/rrt_occupancy_grid', 1)

        self.goal_marker_pub_ = self.create_publisher(Marker, '/rrt_goal_marker', 1)




        # class attributes
        # TODO: maybe create your occupancy grid here

        self.grid_size = 0.1   # grid resolution (meters per cell)
        self.grid_width = 100  # cells (10 meters if grid_size=0.1)
        self.grid_height = 100 # cells
        self.occupancy_grid = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)

        # --- Other Variables ---
        self.tree = []   # will hold your tree nodes

        # Waypoints from CSV
        self.csv_waypoints = []  # list of [x,y] waypoints
        self.goal_index = 0      # keep track of which waypoint is current goal

        self.load_csv_waypoints("/sim_ws/src/pure_pursuit/waypointsds.csv")  # adjust path if needed

        self.current_pose = None
        self.laser_data = None

        self.get_logger().info("RRT Node Initialized")

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """

        # Clear old grid (optional but common)
        self.occupancy_grid.fill(0)


        if self.current_pose is None:
            return

        # get car orientation
        q = self.current_pose.orientation
        theta = 2 * math.atan2(q.z, q.w)

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        for i, r in enumerate(ranges):
            if np.isinf(r) or np.isnan(r):
                continue  # skip invalid measurements
            
            # Compute angle of this measurement
            angle = angle_min + i * angle_increment

            # Get obstacle coordinates in car's local frame
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Rotate into map-aligned grid frame
            x_rot = x * math.cos(theta) - y * math.sin(theta)
            y_rot = x * math.sin(theta) + y * math.cos(theta)

            # Convert to grid cell
            grid_x = int((x_rot / self.grid_size) + self.grid_width // 2)
            grid_y = int((y_rot / self.grid_size) + self.grid_height // 2)

            # Bounds check
            # if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            #     self.occupancy_grid[grid_x, grid_y] = 1  # mark as occupied
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                # Inflate obstacle
                inflation_radius = int(0.2 / self.grid_size)  # adjust 0.2 to tune inflation
                for dx in range(-inflation_radius, inflation_radius + 1):
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        nx = grid_x + dx
                        ny = grid_y + dy
                        if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                            self.occupancy_grid[nx, ny] = 1


        #### Coce for publishing occupancy grid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"  # since it's in the car's local frame

        # --- Grid Metadata ---
        grid_msg.info.resolution = self.grid_size
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        # grid_msg.info.origin.position.x = - (self.grid_width * self.grid_size) / 2.0
        # grid_msg.info.origin.position.y = - (self.grid_height * self.grid_size) / 2.0

        if self.current_pose is None:
            return  # wait until we actually get a pose



        grid_msg.info.origin.position.x = self.current_pose.position.x - (self.grid_width * self.grid_size) / 2.0
        grid_msg.info.origin.position.y = self.current_pose.position.y - (self.grid_height * self.grid_size) / 2.0


        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # --- Flatten and convert numpy array ---
        grid_data = (self.occupancy_grid.T * 100).astype(np.int8)  # transpose for ROS row-major order
        grid_msg.data = grid_data.flatten().tolist()

        # --- Publish ---
        self.occ_grid_pub_.publish(grid_msg)

   


    

    def pose_callback(self, pose_msg):
        """
        The pose callback where the main RRT loop happens.
        """

        # --- Get the car's current pose ---
        self.current_pose = pose_msg.pose.pose  # pose_msg is an Odometry message
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y

        # --- Get the car's orientation (theta) using Scipy ---
        q = self.current_pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        rotation = R.from_quat(quaternion)
        _, _, theta = rotation.as_euler('xyz')

        # --- Create the root node ---
        start_node = TreeNode()
        start_node.x = car_x
        start_node.y = car_y
        start_node.parent = None

        self.tree = [start_node]  # Reset tree every cycle

        # # --- Define the goal ---
        # goal_distance = 4.0  # meters ahead
        # --- Define goal from CSV ---
        if len(self.csv_waypoints) == 0:
            self.get_logger().warn("No CSV waypoints loaded")
            return

        goal_x, goal_y = self.csv_waypoints[self.goal_index]# was self.goal_index

        # goal_distance = 4.0  # meters ahead
        # goal_x = car_x + goal_distance * math.cos(theta)
        # goal_y = car_y + goal_distance * math.sin(theta)


        goal_marker = Marker()
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.header.frame_id = "map"
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal_x
        goal_marker.pose.position.y = goal_y
        goal_marker.pose.position.z = 0.0
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color.a = 1.0
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0  # GREEN
        goal_marker.color.b = 0.0

        self.goal_marker_pub_.publish(goal_marker)


        # === RRT main loop ===
        max_iterations = 500

        for _ in range(max_iterations):
            # 1. Sample a point (in car's local frame)
            sample_point_local = self.sample()

            # 2. Transform sample into map frame
            sample_point = (
                car_x + math.cos(theta) * sample_point_local[0] - math.sin(theta) * sample_point_local[1],
                car_y + math.sin(theta) * sample_point_local[0] + math.cos(theta) * sample_point_local[1]
            )

            # 3. Find nearest node
            nearest_index = self.nearest(self.tree, sample_point)
            nearest_node = self.tree[nearest_index]

            # 4. Steer
            new_node = self.steer(nearest_node, sample_point)
            if new_node is None:
                continue

            # 5. Collision check
            if not self.check_collision(nearest_node, new_node):
                continue

            # 6. Add node to tree
            self.tree.append(new_node)
            self.publish_tree()

            # 7. Goal check
            if self.is_goal(new_node, goal_x, goal_y):
                self.get_logger().info("Goal reached!")
                path = self.find_path(self.tree, new_node)
                # path = self.smooth_path(path)

                # === Publish Path ===
                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = "map"

                for node in path:
                    pose = PoseStamped()
                    pose.header.stamp = path_msg.header.stamp
                    pose.header.frame_id = "map"
                    pose.pose.position.x = node.x
                    pose.pose.position.y = node.y
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)

                self.path_pub_.publish(path_msg)


                #--- Check if the car reached the current goal ---
                if len(self.csv_waypoints) > 0:
                    dist_to_goal = math.hypot(car_x - goal_x, car_y - goal_y)
                    if dist_to_goal < 0.5:   # <-- adjust as you want (maybe 0.5 meters tolerance)
                        self.goal_index = (self.goal_index + 50) % len(self.csv_waypoints)
                        self.get_logger().info(f"Switching to waypoint {self.goal_index}")
                        goal_x, goal_y = self.csv_waypoints[self.goal_index]


                break



        return None


    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        # Define the sampling bounds (in meters)
        x_min = 0.0   # Only sample points in front of the car
        x_max = 5.0
        y_min = -5.0
        y_max = 5.0

        # Sample uniformly within the rectangle
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)


        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        min_dist = float('inf')
        nearest_node = 0

        for i, node in enumerate(tree):

            dist = math.hypot(node.x - sampled_point[0], node.y - sampled_point[1])
            if dist < min_dist:
                min_dist = dist
                nearest_node = i


        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        step_size = 0.1  # meters, you can adjust this later

        # Direction vector from nearest to sample
        dx = sampled_point[0] - nearest_node.x
        dy = sampled_point[1] - nearest_node.y
        dist = math.hypot(dx, dy)

        # Normalize direction vector
        if dist == 0:
            return None  # sample is exactly at the node, rare but safe to handle

        dx /= dist
        dy /= dist

        # Create the new node
        new_node = TreeNode()
        new_node.x = nearest_node.x + step_size * dx
        new_node.y = nearest_node.y + step_size * dy
        new_node.parent = nearest_node

        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """


        num_steps = max(1,int(math.hypot(new_node.x - nearest_node.x, new_node.y - nearest_node.y) / self.grid_size) * 2)  # oversample a little

        for i in range(num_steps + 1):
            # Interpolate between the two nodes
            t = i / num_steps
            x = nearest_node.x + t * (new_node.x - nearest_node.x)
            y = nearest_node.y + t * (new_node.y - nearest_node.y)

            # Convert to grid coordinates
            # grid_x = int((x / self.grid_size) + self.grid_width // 2)
            # grid_y = int((y / self.grid_size) + self.grid_height // 2)
            dx = x - self.current_pose.position.x
            dy = y - self.current_pose.position.y

            grid_x = int((dx / self.grid_size) + self.grid_width // 2)
            grid_y = int((dy / self.grid_size) + self.grid_height // 2)


            # Check bounds
            if not (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height):
                return False  # treat out-of-bounds as collision
            
            # Check occupancy
            if self.occupancy_grid[grid_x, grid_y] == 1:
                return False  # collision detected
            

        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        goal_tolerance = 0.1  # meters

        dist_to_goal = math.hypot(latest_added_node.x - goal_x,
                                latest_added_node.y - goal_y)

        return dist_to_goal < goal_tolerance
    

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        node = latest_added_node

        while node is not None:
            path.append(node)
            node = node.parent

        path.reverse()

        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood
    
    def publish_tree(self):
        """
        Publishes the RRT tree as a MarkerArray for RViz visualization.
        """
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line width
        marker.color.a = 1.0
        marker.color.r = 1.0  # red tree

        # Add line segments between each node and its parent
        for node in self.tree:
            if node.parent is not None:
                p1 = Point()
                p1.x = node.x
                p1.y = node.y
                p1.z = 0.0

                p2 = Point()
                p2.x = node.parent.x
                p2.y = node.parent.y
                p2.z = 0.0

                marker.points.append(p1)
                marker.points.append(p2)

        marker_array.markers.append(marker)
        self.marker_pub_.publish(marker_array)

    def load_csv_waypoints(self, filename):
        with open(filename, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) >= 2:
                    self.csv_waypoints.append([float(row[0]), float(row[1])])
        self.csv_waypoints = self.csv_waypoints[120:]
        self.get_logger().info(f"Loaded {len(self.csv_waypoints)} waypoints from CSV")


    def smooth_path(self, path):
        """
        Simple shortcut smoothing for RRT path.

        Args:
            path (list of TreeNode): original path

        Returns:
            smoothed_path (list of TreeNode): smoothed path
        """
        if len(path) < 2:
            return path  # nothing to smooth

        smoothed_path = [path[0]]

        i = 0
        while i < len(path) - 1:
            # Look ahead as far as possible
            j = len(path) - 1
            while j > i + 1:
                if self.check_collision(path[i], path[j]):
                    break  # found a valid shortcut
                j -= 1

            smoothed_path.append(path[j])
            i = j

        return smoothed_path




def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
