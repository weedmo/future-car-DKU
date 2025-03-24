#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
from rocketdan.msg import LaneInfo
from lanedetector import LaneDetector

import rospy
import rospkg
import os
from cv_bridge import CvBridge

class NodeLaneDetect():
    def __init__(self):
        rospy.init_node('lane_detector')
        
        self.initialize_variables()

        rospy.Subscriber(self.cam_topic, CompressedImage, self.cb_cam, queue_size=1)

        self.pub_lane_info = rospy.Publisher('/lane/lane_info', LaneInfo, queue_size=1)
        self.pub_image = rospy.Publisher('/lane/image', Image, queue_size=1)
        if self.crosswalk_mode:
            self.pub_crosswalk = rospy.Publisher('lane/is_crosswalk_detected', Bool, queue_size=1)

    def initialize_variables(self):        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rocketdan')

        weight_fn = rospy.get_param('~weight', 'weights/yololane_n_240804.pt')
        weight_fn = os.path.join(pkg_path, weight_fn)
        persepctive_point_fn = os.path.join(pkg_path, 'lanedetect/perspective_points.yaml')

        lane_width = rospy.get_param('~lane_width', 560)//2
        self.draw = rospy.get_param('~draw', True)
        verbose = rospy.get_param('~verbose', False)
        self.cam_topic = rospy.get_param('~cam_topic', '/usb_cam_lane/image_raw/compressed')
        self.crosswalk_mode = rospy.get_param('~crosswalk_mode', True)

        self.lane_detector = LaneDetector(weight_fn, persepctive_point_fn, 
                                          draw=self.draw, lane_width=lane_width, verbose=verbose,
                                          crosswalk_mode=self.crosswalk_mode)

        self.image = None
        self.lane_info_msg = LaneInfo()
        self.bridge = CvBridge()

    def cb_cam(self, msg: CompressedImage) -> None:
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    def main_loop(self, loop_hz = 30):
        rospy.loginfo('NodeLaneDetector: Waiting Topics --------------')
        rospy.wait_for_message(self.cam_topic, CompressedImage)
        rospy.loginfo(f'NodeLaneDetector: run the main thread at {loop_hz}Hz')
                
        rate = rospy.Rate(loop_hz)
        while not rospy.is_shutdown():
            if self.image is None:
                rate.sleep()
                continue

            ret = self.lane_detector.do(self.image)
            self.lane_info_msg.xpos, self.lane_info_msg.angle = ret[0], ret[1]
            if self.crosswalk_mode and ret[2]:
                self.pub_crosswalk.publish(Bool(True))

            if self.draw:
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(ret[3],encoding='bgr8'))
                
            self.pub_lane_info.publish(self.lane_info_msg)

            rate.sleep()

if __name__ == '__main__':
    node = NodeLaneDetect()
    node.main_loop(loop_hz=30)