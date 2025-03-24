#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor:
    def __init__(self, mode=1):
        self.init_params(mode=mode)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/roi_2d', LaserScan, queue_size=10)

    def init_params(self, mode=1):
        if mode == 1: # left
            self.min_angle = np.deg2rad(90)
            self.max_angle = np.deg2rad(95) # 차량에 포인트 나오는지 확인할 것
            self.min_dist = 0  # 변경된 단위: meters
            self.max_dist = 1.5  # 변경된 단위: meters
        else: # right
            self.min_angle = np.deg2rad(90)
            self.max_angle = np.deg2rad(92.5) # 차량에 포인트 나오는지 확인할 것
            self.min_dist = 0  # 변경된 단위: meters
            self.max_dist = 2.4  # 변경된 단위: meters
       
    def scan_callback(self, data: LaserScan):
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
        scan = np.vstack((angles, ranges)).T
       
        result = self.get_angle_distance_range(scan, self.min_angle, self.max_angle, self.min_dist, self.max_dist)
       
        # rospy.loginfo("Original Scan Data: %s", scan)
        # rospy.loginfo("Filtered Data: %s", result)
       
        filtered_ranges = np.full_like(ranges, np.inf)
        for angle, dist in result:
            index = np.argmin(np.abs(angles - angle))
            filtered_ranges[index] = dist
       
        filtered_scan = LaserScan()
        filtered_scan.header = data.header
        filtered_scan.angle_min = data.angle_min
        filtered_scan.angle_max = data.angle_max
        filtered_scan.angle_increment = data.angle_increment
        filtered_scan.time_increment = data.time_increment
        filtered_scan.scan_time = data.scan_time
        filtered_scan.range_min = data.range_min
        filtered_scan.range_max = data.range_max
        filtered_scan.ranges = filtered_ranges.tolist()
       
        self.pub.publish(filtered_scan)
   
    def get_angle_distance_range(self, scan, min_angle, max_angle, min_dist, max_dist):
        condition = (scan[:, 0] >= min_angle) & (scan[:, 0] <= max_angle) & \
                    (scan[:, 1] >= min_dist) & (scan[:, 1] <= max_dist)
        return scan[condition]

if __name__ == "__main__":
    rospy.init_node('lidar_processor', anonymous=True)
    mode = rospy.get_param('~mode', 1)
    rospy.loginfo(f'ROI_Parking: mode {mode}')
    processor = LidarProcessor(mode=mode)
    rospy.spin()

