#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool
from typing import Tuple

class NodeObstacleDetector():
    def __init__(self,
                 range_min: float = 0.3,
                 range_max: float = 0.8,
                 width: int = 256,
                 height: int = 256,
                 threshold: float = 0.3) -> None:        
        rospy.init_node('obstalce_detector')
        rospy.Subscriber("/scan", LaserScan, self.cb_scan, queue_size=1)        
        
        self.range_min = range_min
        self.range_max = range_max
        self.width, self.height = width,height
        self.half_width, self.half_height = self.width //2, self.height //2
        
        self.threshold = threshold

        self.start_point = (self.half_width, self.half_height)
        self.end_point_pos = (int(self.half_width + self.width * np.cos(np.radians(-45))),
                        int(self.half_height + self.height * np.sin(np.radians(-45))))
        
        self.end_point_neg = (int(self.half_width + self.width * np.cos(np.radians(-135))),
                        int(self.half_height + self.height * np.sin(np.radians(-135))))

    def main_loop(self, loop_hz=30, debug=True):  
        if debug:
            pub_img = rospy.Publisher('obs/debug_img', Image, queue_size=10)
            bridge = CvBridge()
        pub_res = rospy.Publisher('obs/is_detected', Bool, queue_size=1)
        res_msg = Bool()
        rospy.loginfo('NodeObstacleDetector: Waiting -------------------')
        rospy.wait_for_message("/scan", LaserScan)
        rospy.loginfo("NodeObstacleDetector: Scan Ready --------------")

        rate = rospy.Rate(loop_hz)

        while not rospy.is_shutdown():
            ret = self.obs_detect(debug=debug)
            if debug:
                debug_img_msg = bridge.cv2_to_imgmsg(ret[1],encoding="rgb8")
                pub_img.publish(debug_img_msg)
            res_msg.data = ret[0]
            pub_res.publish(res_msg)
            rate.sleep()

    def obs_detect(self, debug: bool = False) -> Tuple[bool, float, float, float, np.ndarray]:
        # initialize
        temp_img = np.zeros((self.width, self.height), dtype=np.uint8)        
        intensity_max = max(self.scan_msg.intensities)

        # filtering out-of-range values
        ranges = np.array(self.scan_msg.ranges)
        ranges[np.where((ranges<self.range_min) | (ranges>self.range_max))] = 0

        # creating top-down view image
        for idx, rg in enumerate(ranges):
            rg = rg / self.range_max * self.half_width # converting the unit of range values from meters to pixels
            curr_angle = self.scan_msg.angle_min + idx*self.scan_msg.angle_increment

            x, y = (self.half_width+int(np.cos(curr_angle)*rg),
                    self.half_height+int(np.sin(curr_angle)*rg))
            if curr_angle>np.radians(135) or curr_angle<np.radians(-135):
                temp_img[x, y] = 255

        # finding center of left and right corns            
        notzero_x_index = np.array(np.where(np.sum(temp_img[:self.half_height,:],axis=0)!=0)) # summing up each column and finding which column sum is not zero

        if debug:
            # creating debug image
            img_bgr = cv2.cvtColor(temp_img, cv2.COLOR_GRAY2BGR)        
            cv2.line(img_bgr, self.start_point, self.end_point_pos, (255, 0, 0), 2)
            cv2.line(img_bgr, self.start_point, self.end_point_neg, (255, 0, 0), 2)
            img_bgr[self.half_height,notzero_x_index,:]=(255,0,255)

            
            if len(notzero_x_index[0]) <= self.half_width*self.threshold:
                return False, img_bgr
            
            center_x_px = int(np.mean(notzero_x_index))
            img_bgr = cv2.circle(img_bgr, (center_x_px,self.half_height), 5, (255,0,255), 5)            
            return True, img_bgr
        else:
            if len(notzero_x_index[0]) <= self.half_width*self.threshold:
                return False, None
            return True, None

    def cb_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg


if __name__ == '__main__':
    obs_detector = NodeObstacleDetector(range_max=1.8, threshold=0.1)
    obs_detector.main_loop(loop_hz=10, debug=True)