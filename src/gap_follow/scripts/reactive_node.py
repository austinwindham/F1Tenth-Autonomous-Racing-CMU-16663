#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.subscription_scan = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10)

        # TODO: Publish to drive
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.angle_min = -np.pi * 0.75
        self.angle_max = np.pi * 0.75
        self.angle_inc = (self.angle_max - self.angle_min)/1080

        self.angle_range = 90 *np.pi/180
        self.start_idx = int(( -self.angle_range - self.angle_min) / self.angle_inc)
        self.end_idx = int((self.angle_range - self.angle_min) / self.angle_inc)

        self.threshold = 2.2 # has been 1.5, maybe try to increase 2.2 worked befor you left

        self.split = 0
        self.max_index = 0



    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges = np.array(ranges)

        ranges[np.isinf(ranges)] = 5.0  # Replace Inf with 3m
        ranges[np.isnan(ranges)] = 0.0  # Replace NaN with 0
        ranges[ranges > 5.0] =5.0  # Cap max range at 3m
        ranges[ranges < 0.0] = 5.0  # Fix negative values (if any)

        window_size = 5
        proc_ranges = np.convolve(ranges, np.ones(window_size)/window_size, mode='same')

        
        return proc_ranges[self.start_idx: self.end_idx]

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        # Detect if an intersection is ahead
        if self.split > 0:
            self.split -=1
            # print(" in side self.split")
            return self.max_index, self.max_index
        num_samples = 30
        mid_idx = len(free_space_ranges)//2

        mid_avg = np.mean(free_space_ranges[mid_idx -num_samples: mid_idx + num_samples])
        right_avg = np.mean(free_space_ranges[mid_idx -2*num_samples: mid_idx -num_samples])
        left_avg = np.mean(free_space_ranges[mid_idx +num_samples: mid_idx + 2*num_samples])

        max_right = np.max(free_space_ranges[0: mid_idx])
        max_left = np.max( free_space_ranges[mid_idx: -1])

        if left_avg > 2.0 and right_avg > 2.0 and left_avg > mid_avg + 0.7 and right_avg > mid_avg + 0.7 and mid_avg < 3.0:
            print("here               here                here")
            print(f" left: {left_avg:.2f}, mid : {mid_avg:.2f}, l right: {right_avg:.2f}")
            print(f" max left: {max_left:.2f}, max_right: {max_right:.2f}")
            
            self.max_index = np.argmax(free_space_ranges)
            self.split = 375
            print(self.max_index)
            # self.threshold = 4.5
            return self.max_index, self.max_index

        
        # print(f" left: {left_avg:.2f}, mid : {mid_avg:.2f}, l right: {right_avg:.2f}")

        

        # print(f" max left: {max_left:.2f}, max_right: {max_right:.2f}")

        curr_range = []
        max_range = []

        for i in range(len(free_space_ranges)):

            
            if free_space_ranges[i] > self.threshold:
                curr_range.append(i)
            else:
                if len(curr_range) > len(max_range):
                    max_range = curr_range
                curr_range = []

        if not max_range:
            farthest_idx = np.argmax(free_space_ranges)
            return farthest_idx, farthest_idx
        
        start_idx = max_range[0]
        stop_idx = max_range[-1]   

        self.threshold = 2.2
        return start_idx, stop_idx
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """

        drive_idx = (end_i + start_i) // 2

        return drive_idx

    ### functions i made until lidar_callback

    def apply_safety_bubble(self, ranges):
        """
        Applies a safety bubble around the closest obstacle by zeroing out nearby LIDAR readings.
        This version explicitly uses tan() instead of small-angle approximation.

        Args:
            ranges: Processed LIDAR scan data (list or numpy array)

        Returns:
            Updated ranges with safety bubble applied.
        """
        min_distance = np.min(ranges[120:600])  # Find closest obstacle
        if min_distance > 1.0:
            return ranges
        min_index = np.argmin(ranges)  + 120# Get its index

        # Define the radius of the safety bubble
        r = 0.215# Example: car radius (adjust based on your car size)

        # Compute how many indices to clear using `tan()`
        angle_per_index = (np.pi / len(ranges))  # Assuming full 180-degree FOV
        indices_to_clear = int(np.arctan(r / min_distance) / angle_per_index)

        # Zero out indices around the closest point
        start_idx = max(min_index - indices_to_clear, 0)
        stop_idx = min(min_index + indices_to_clear, len(ranges) - 1)
        ranges[start_idx:stop_idx] = 0.0  # Set to zero to ignore these points

        

        return ranges 


    def extend_disparities(self, ranges):
        """ Extends obstacles leftward and rightward to account for car width. """
    
        modified_ranges = np.copy(ranges)  # Create a copy so we don't modify while iterating

        for i in range(len(ranges) - 1):
            if abs(ranges[i] - ranges[i + 1]) > 0.5:  # Large jump detected
                num_indices_to_extend = int(np.arctan(0.20 / max(min(ranges[i], ranges[i+1]), 0.1))/ self.angle_inc)  # How many indices to extend

                if ranges[i] > ranges[i + 1]:  # Obstacle on the right → Extend left
                    for j in range(max(0, i - num_indices_to_extend), i + 1):
                        modified_ranges[j] = ranges[i + 1]  # Set to smaller value (closer obstacle)

                else:  # Obstacle on the left → Extend right
                    for j in range(i + 1, min(i + 1 + num_indices_to_extend, len(ranges))):
                        modified_ranges[j] = ranges[i]  # Set to larger value (closer obstacle)

        return modified_ranges  # Return the modified copy



    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # # TODO:
        #Find closest point to LiDAR
        # closest_idx = np.argmin(proc_ranges)
        # closest

        # #Eliminate all points inside 'bubble' (set them to zero) 
        # bubble_size = 10  # Number of points to remove around obstacle
        # proc_ranges[max(0, closest_idx - bubble_size) : min(len(proc_ranges), closest_idx + bubble_size)] = 0

        # Bubble
        proc_ranges = self.extend_disparities(proc_ranges)
        proc_ranges = self.apply_safety_bubble(proc_ranges)

        # Dispairyt extender
        #Find max length gap 
        start_idx, end_idx = self.find_max_gap(proc_ranges)


        #Find the best point in the gap 
        best_idx = self.find_best_point(start_idx, end_idx, proc_ranges)

        #Publish Drive message
        steering_angle = self.angle_min + 3*np.pi/4 - self.angle_range + (best_idx * self.angle_inc)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle  # Convert to radians

        if abs(steering_angle) <= np.pi / 180 * 10:
            velocity = 1.0
        elif abs(steering_angle) <= np.pi / 180 * 20:
            velocity = 0.5
        else:
            velocity = 0.5

        drive_msg.drive.speed = velocity # Set speed (adjust as needed) 

        self.publisher_drive.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()