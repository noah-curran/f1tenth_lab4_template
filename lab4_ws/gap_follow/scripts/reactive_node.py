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

        self._laser_scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self._cmd_drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        
        self.rb = 1.
        self.forward_range = np.deg2rad(30)

    def preprocess_lidar(self, data: LaserScan):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # TODO: item #1 above
        # num_points = len(data.ranges)
        # mid_point = num_points // 2
        # angle_per_step = data.angle_increment
        # start_index = int(mid_point - self.forward_range / angle_per_step)
        # end_index = int(mid_point + self.forward_range / angle_per_step)
        
        # proc_ranges = data.ranges
        # # Set elements from the start to the start_index to inf
        # for i in range(start_index):
        #     proc_ranges[i] = 0

        # # Set elements from end_index + 1 to the end of the array to inf
        # for i in range(end_index + 1, len(proc_ranges)):
        #     proc_ranges[i] = 0
        
        proc_ranges = [0 if r <= 3 else r for r in data.ranges]
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_len = 0
        max_start = 0
        max_end = 0
        current_len = 0
        current_start = None

        for i, value in enumerate(free_space_ranges):
            if value != 0:
                if current_start is None:
                    current_start = i
                current_len += 1
            else:
                if current_len > max_len:
                    max_len = current_len
                    max_start = current_start
                    max_end = i - 1
                current_len = 0
                current_start = None

        # Check last sequence in case the longest non-zero sequence ends at the last element
        if current_len > max_len:
            max_len = current_len
            max_start = current_start
            max_end = len(free_space_ranges) - 1

        return (max_start, max_end)

    
    def find_best_point(self, start_i: int, end_i: int, ranges: list):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # Naive
        best_point = start_i + ranges[start_i:end_i+1].index(max(ranges[start_i:end_i+1]))
        return best_point

    def lidar_callback(self, data: LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        
        proc_ranges = self.preprocess_lidar(data)
        
        #Find closest point to LiDAR
        closest_point_i = proc_ranges.index(min(proc_ranges))
        
        if closest_point_i < len(proc_ranges):

            #Eliminate all points inside 'bubble' (set them to zero) 
            a = proc_ranges[closest_point_i]
            
            left_i = closest_point_i + 1
            if left_i < len(proc_ranges):
        
                b = proc_ranges[left_i]
                theta = data.angle_increment
                while np.sqrt(a**2 + b**2 - 2*a*b*np.cos(theta)) < self.rb:
                    proc_ranges[left_i] = 0.
                    
                    left_i += 1
                    
                    if left_i >= len(proc_ranges):
                        break
                    
                    b = proc_ranges[left_i]
                    theta += data.angle_increment
        
        
            right_i = closest_point_i - 1
            if right_i >= 0:
        
                b = proc_ranges[right_i]
                theta = data.angle_increment
                while np.sqrt(a**2 + b**2 - 2*a*b*np.cos(theta)) < self.rb:
                    proc_ranges[right_i] = 0.
                    
                    right_i -= 1
                    
                    if right_i < 0:
                        break
                    
                    b = proc_ranges[right_i]
                    theta += data.angle_increment

        #Find max length gap 
        max_start, max_end = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_point_i = self.find_best_point(max_start, max_end, proc_ranges)

        #Publish Drive message
        num_points = len(data.ranges)
        mid_point = num_points // 2
        angle_per_step = data.angle_increment
        angle = angle_per_step * (best_point_i - mid_point)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = 1.5
        self._cmd_drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()