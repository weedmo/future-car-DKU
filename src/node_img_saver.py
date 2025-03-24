#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
import cv2
import os
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class NodeImageSaver():
    def __init__(self, path:str='/root/shared/saved_img',
                       topic_name:str='/usb_cam_lane/image_raw/compressed',
                       skip_ms:int=100) -> None:        
        rospy.init_node('image_saver')
        rospy.Subscriber(topic_name, CompressedImage, self.cb_img, queue_size=1)        
        
        if not os.path.exists(path):
            os.makedirs(path)

        self.bridge = CvBridge()
        self.path = path
        self.skip_ms = skip_ms
        self.previous_received_time = time.time()
        self.idx = 0

        rospy.loginfo(f'NodeImageSaver: Waiting for receiving image topic [{topic_name}].')

    def cb_img(self, msg: CompressedImage) -> None:
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        now_time = time.time()*1000
        if now_time - self.previous_received_time < self.skip_ms:
            return
        self.previous_received_time = now_time
        cv2.imwrite(os.path.join(self.path,f'{self.idx:08}.png'), img)
        rospy.loginfo(f'{self.idx:08}.jpg has been saved.')
        self.idx+=1        

if __name__ == '__main__':
    path='/root/shared/saved_img/2'
    topic_name='/usb_cam_light/image_raw/compressed'
    skip_ms=100

    nis = NodeImageSaver(path=path, topic_name=topic_name, skip_ms=skip_ms)
    rospy.spin() 