#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

from sensor_msgs.msg import CompressedImage
import rospy
import rospkg
import cv2
import numpy as np
import yaml
import time
import os
from cv_bridge import CvBridge

class PerspectivePointSelector:
    def __init__(self, cam_topic:str, saved_fn:str, yolo_size=(640,384)):
        rospy.init_node('perspective_point_selector')
        rospy.Subscriber(cam_topic, CompressedImage, self.cb_cam, queue_size=1)

        self.current_point = (0, 0)
        self.is_drawing = False
        self.display_points = [[0,0], [0,0], [0,0], [0,0], [0,0]]
        self.saved_points =   [[0,0], [0,0], [0,0], [0,0], [0,0]]
        self.current_idx = 0
        self.prev_idx = 0
        self.yolo_size = yolo_size

        self.load_points(saved_fn)

        self.cam_topic = cam_topic
        self.bridge = CvBridge()

    def main_loop(self):        
        rospy.loginfo('PersepctiveSelector: Waiting Topics--------------')
        rospy.wait_for_message(self.cam_topic, CompressedImage)
        rospy.loginfo(f'PersepctiveSelector: run the main thread')

        colors = [(255,0,0), (0,255,0), (0,0,255), (255,0,2550), (255,255,255)]
        
        cv2.namedWindow('fig')
        cv2.setMouseCallback("fig", self.cb_mouse)        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            img_display = cv2.resize(self.img.copy(), self.yolo_size)
            if self.is_drawing:
                self.display_points[self.current_idx] = self.current_point
                cv2.line(img_display, (self.display_points[self.current_idx][0],0), (self.display_points[self.current_idx][0], self.yolo_size[1]), colors[self.current_idx], thickness=2)
                if self.current_idx != 4:
                    cv2.line(img_display, (0, self.display_points[self.current_idx][1]), (self.yolo_size[0], self.display_points[self.current_idx][1]), colors[self.current_idx], thickness=2)
            for i in range(4):
                cv2.circle(img_display, tuple(self.display_points[i]), 6, colors[i], thickness=3)
            cv2.line(img_display, (self.display_points[4][0],0), (self.display_points[4][0],self.yolo_size[1]), (255,255,255), thickness=2)
                
            self.put_text(img_display, '1~4: place ith point', (0,30))
            self.put_text(img_display, 'c: place the center point', (0,45))
            self.put_text(img_display, 'p: process perspective transform', (0,60))
            self.put_text(img_display, 's: save the selected points', (0,75))
            self.put_text(img_display, 'q: terminate the program', (0,90))

            cv2.imshow('fig', img_display)
            key = cv2.waitKey(1)

            if key == ord('q'):
                break
            elif key == ord('e'):
                self.is_drawing = not self.is_drawing
                if not self.is_drawing:
                    self.display_points[self.current_idx] = self.saved_points[self.current_idx]
            elif key == ord('1'):
                self.change_idx(0)
            elif key == ord('2'):
                self.change_idx(1)
            elif key == ord('3'):
                self.change_idx(2)
            elif key == ord('4'):
                self.change_idx(3)
            elif key==ord('c'):
                self.change_idx(4)
            elif key == ord('p'):
                img_warp = self.perspective_warp(img_display)
                cv2.imshow('warped', img_warp)
            elif key == ord('s'):
                now = time.localtime()
                self.save_points(f'perspective_points-{now.tm_year}-{now.tm_mon}-{now.tm_mday}-{now.tm_hour}-{now.tm_min}-{now.tm_sec}.yaml')
            
            rate.sleep()

    def save_points(self, fn):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rocketdan')
        fn = os.path.join(pkg_path, f'lanedetect/{fn}')
        
        with open(fn, 'w') as f:
            points = {}
            for i in range(4):
                points[i] = {'x': self.saved_points[i][0], 'y': self.saved_points[i][1]}
            points[4] = {'x': self.saved_points[4][0]}
            yaml.dump(points, f)
        rospy.loginfo(f'saved to {fn}')
    
    def load_points(self, fn):
        if os.path.exists(fn):
            with open(fn) as f:
                saved_dict = yaml.load(f, Loader=yaml.FullLoader)
            for i in range(4):
                self.saved_points[i] = [saved_dict[i]['x'],saved_dict[i]['y']]
            self.saved_points[4][0] = saved_dict.get(4)['x']
            if self.saved_points[4][0] == None: self.saved_points[4][0] = 0
            self.display_points = self.saved_points.copy()

    def change_idx(self, idx):
        self.current_idx = idx
        self.display_points[self.prev_idx] = self.saved_points[self.prev_idx]
        self.prev_idx = self.current_idx
        self.is_drawing = True

    def cb_mouse(self, event, x, y, flag, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.current_point = (x,y)
        elif event == cv2.EVENT_LBUTTONUP and self.is_drawing:
            self.is_drawing = False
            self.saved_points[self.current_idx] = self.current_point
            self.display_points[self.current_idx] = self.saved_points[self.current_idx]

    def perspective_warp(self, img):        
        dst = np.float32([(0,0), (self.yolo_size[0],0), (self.yolo_size[0],self.yolo_size[1]), (0,self.yolo_size[1])])
        M = cv2.getPerspectiveTransform(np.float32(self.saved_points[:4]), dst)
        img_warped = cv2.warpPerspective(img, M, self.yolo_size)
        return img_warped

    def put_text(self, img, txt, pos):
        cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 3)
        cv2.putText(img, txt, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    def cb_cam(self, msg: CompressedImage) -> None:
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

if __name__ == '__main__':    
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rocketdan')
    saved_fn = os.path.join(pkg_path, 'lanedetect/perspective_points.yaml')
    
    cam_topic = '/usb_cam_lane/image_raw/compressed'
    pps = PerspectivePointSelector(cam_topic, saved_fn)

    pps.main_loop()