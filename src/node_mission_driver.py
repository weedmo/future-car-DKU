#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage, Image
import time
from ultralytics import YOLO
import rospkg
import os
from cv_bridge import CvBridge

class NodeMissionDriver():
    def __init__(self, weight_fn='weights/traffic_light.pt',
                       cam_topic='usb_cam_traffic/image_raw/compressed',
                       detour_mode=0):
        self.yolo_loader(weight_fn)

        self.pub_control_enable = rospy.Publisher('lane/control_enable', Bool, queue_size=1)        
        self.pub_lat = rospy.Publisher('latt_cmd', Int16, queue_size=1)
        self.pub_lon = rospy.Publisher('longi_cmd', Int16, queue_size=1)
        self.pub     = rospy.Publisher('lane/set_mode', Int16, queue_size=1)

        rospy.Subscriber('obs/is_detected', Bool, self.cb_obs, queue_size=1)
        rospy.Subscriber('lane/is_crosswalk_detected', Bool, self.cb_crosswalk, queue_size=1)
        rospy.Subscriber(cam_topic, CompressedImage, self.cb_cam,queue_size=1)

        self.init_detour_params(mode=detour_mode)

        self.is_obs_detected = False
        self.obs_detected_count = 0
        self.donot_detect_obs = True
        self.th_obs = 5

        self.is_crosswalk_detected = False
        self.crosswalk_detected_count = 0
        self.donot_detect_crosswalk = True

        self.is_tl_detected = False
        self.tl_detected_count = 0

        self.lane_follower_set_mode()

    def init_detour_params(self, mode: int):
        self.donot_detect_obs_duration = 7 # in secs
        # mode1 젤 가까움, mode 2 중간, mode 3 젤 멀리
                     # speed, duration
        param_mode1 = [[100,    3.5], # phase 0 (left)
                       [100,    3.5], # phase 1 (right)
                       [100,    2.0], # phase 2 (straight)
                       [100,    2.5], # phase 3 (right)
                       [110,    2.5], # phase 4 (align)
                       [110,    0.5]  # phase 5 (straight)
                      ]
        param_mode2 = [[100,    3.5], # phase 0 (left)
                       [100,    3.5], # phase 1 (right)
                       [100,    2.0], # phase 2 (straight)
                       [100,    2.5], # phase 3 (right)
                       [110,    2.5], # phase 4 (align)
                       [110,    0.5]  # phase 5 (straight)
                      ]
        param_mode3 = [[100,    3.5], # phase 0 (left)
                       [100,    3.5], # phase 1 (right)
                       [100,    2.0], # phase 2 (straight)
                       [100,    2.5], # phase 3 (right)
                       [100,    2.5], # phase 4 (align)
                       [100,    0.5]  # phase 5 (straight)
                      ]
        
        if mode == 1:
            self.detour_param = param_mode1
        elif mode == 2:
            self.detour_param = param_mode2
        else:
            self.detour_param = param_mode3 

    def yolo_loader(self, weight_fn):
        rospack = rospkg.RosPack()
        weight_file = os.path.join(rospack.get_path('rocketdan'), weight_fn)
        self.yolo = YOLO(weight_file)
        self.bridge = CvBridge()
        self.pub_yolo_img = rospy.Publisher('/yolo_img', Image, queue_size=1)
        self.img = None
    
    def main_loop(self, hz=30):
        rospy.loginfo('NodeMissionDriver: Start mission')
        rate = rospy.Rate(hz)

        start_time_of_control = rospy.Time.now()
        one_time_flag = True

        ## debug purpose
        # one_time_flag=False
        # self.donot_detect_obs=False
        ################
        while not rospy.is_shutdown():     
            if one_time_flag and ((rospy.Time.now() - start_time_of_control).to_sec() >= self.donot_detect_obs_duration):
                rospy.logerr('NodeMissionDriver: Obs detection enabled')
                self.donot_detect_obs=False
                one_time_flag=False

            if not self.donot_detect_obs and self.is_obs_detected:
                rospy.logerr('NodeMissionDriver: Start detouring')
                self.stop_lane_following()
                self.detour(hz)
                self.donot_detect_obs = True
                self.donot_detect_crosswalk = False

            if not self.donot_detect_crosswalk and self.is_crosswalk_detected:
                rospy.logerr('NodeMissionDriver: Start wating for traffic light')
                self.stop_lane_following()
                self.traffic_light_detect(rate)
                self.donot_detect_crosswalk = True

            # rospy.loginfo(f'{self.is_obs_detected}, {self.obs_detected_count}, {self.donot_detect_obs}, \
            #                {self.is_crosswalk_detected}, {self.crosswalk_detected_count}, {self.donot_detect_crosswalk}, \
            #                 {self.is_tl_detected}, {self.tl_detected_count}')
            rate.sleep()

    def detour(self, hz):
        rate = rospy.Rate(hz)
        avoid_phase = 0
        steer, speed = Int16(), Int16()
        avoid_start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if avoid_phase == 0:
                steer.data = 600
                speed.data = self.detour_param[0][0]
                duration = self.detour_param[0][1]
                rospy.loginfo(f'0, turn left, speed={speed.data}, duration={duration}')
                if (rospy.Time.now() - avoid_start_time).to_sec() > duration:
                    avoid_phase = 1
                    avoid_start_time = rospy.Time.now()

            elif avoid_phase == 1:
                steer.data = -600
                speed.data = self.detour_param[1][0]
                duration = self.detour_param[1][1]
                rospy.loginfo(f'1, turn right, speed={speed.data}, duration={duration}')
                if (rospy.Time.now() - avoid_start_time).to_sec() > duration:
                    avoid_phase = 2
                    avoid_start_time = rospy.Time.now()

            elif avoid_phase == 2:
                steer.data = 0
                speed.data = self.detour_param[2][0]
                duration = self.detour_param[2][1]
                rospy.loginfo(f'2, straight, speed={speed.data}, duration={duration}')
                if (rospy.Time.now() - avoid_start_time).to_sec() > duration:
                    avoid_phase = 3
                    avoid_start_time = rospy.Time.now()

            elif avoid_phase == 3:
                steer.data = -600
                speed.data = self.detour_param[3][0]
                duration = self.detour_param[3][1]
                rospy.loginfo(f'3, turn right, speed={speed.data}, duration={duration}')
                if (rospy.Time.now() - avoid_start_time).to_sec() > duration:
                    avoid_phase = 4
                    avoid_start_time = rospy.Time.now()

            elif avoid_phase == 4:
                steer.data = 600
                speed.data = self.detour_param[4][0]
                duration = self.detour_param[4][1]
                rospy.loginfo(f'4, align heading, speed={speed.data}, duration={duration}')
                if (rospy.Time.now() - avoid_start_time).to_sec() > duration:
                    avoid_phase = 5
                    avoid_start_time = rospy.Time.now()

            elif avoid_phase == 5:
                steer.data = 0
                speed.data = self.detour_param[5][0]
                duration = self.detour_param[5][1]
                rospy.loginfo(f'5, mini str, speed={speed.data}, duration={duration}')
                if (rospy.Time.now() - avoid_start_time).to_sec() > duration:
                    avoid_phase = 0
                    break

            # Publish steering and speed commands
            self.pub_lat.publish(steer)
            self.pub_lon.publish(speed)
            rate.sleep()

        rospy.logerr('NodeMissionDriver: resume (obs detouring)')
        self.resume_lane_following()
    
    def traffic_light_detect(self, rate):
        while not rospy.is_shutdown():   
            result = self.yolo(self.img, verbose=False)
            boxes = result[0].boxes
            num_boxes = len(boxes)
            if num_boxes !=0 and boxes.cls[0]==0:
                found=True
                rospy.loginfo(f'NodeMissionDriver: traffic light detected... {self.tl_detected_count}')
            else:
                found=False
            self.is_tl_detected, self.tl_detected_count = self.decision_maker(found, self.tl_detected_count)

            if self.is_tl_detected:
                rospy.loginfo('NodeMissionDriver: Traffic light detected')
                rospy.logerr('NodeMissionDriver: resume (crosswalk)')
                self.resume_lane_following()
                break
            
            self.pub_yolo_img.publish(self.bridge.cv2_to_imgmsg(result[0].plot(),encoding='bgr8'))
            rate.sleep()

    def cb_obs(self, msg:Bool):
        if self.donot_detect_obs:
            return
        self.is_obs_detected, self.obs_detected_count = self.decision_maker(msg.data, self.obs_detected_count, self.th_obs)
        if self.is_obs_detected and self.obs_detected_count == self.th_obs:
            rospy.loginfo('NodeMissionDriver: Obstacle detected')

    def cb_crosswalk(self, msg:Bool):
        if self.donot_detect_crosswalk:
            return
        self.is_crosswalk_detected, self.crosswalk_detected_count = self.decision_maker(msg.data, self.crosswalk_detected_count,2)
        # if self.is_crosswalk_detected:
        #     rospy.loginfo('NodeMissionDriver: Crosswalk detected')
            
    def cb_cam(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def decision_maker(self, msg, counter, threshold=5):
        ret = False
        if msg:
            counter += 1
            if counter >= threshold:
                ret = True
        else:
            counter = 0
        return ret, counter

    def stop_lane_following(self):
        self.pub_control_enable.publish(Bool(False))
        self.pub_lat.publish(Int16(0))
        self.pub_lon.publish(Int16(0))

    def resume_lane_following(self):
        self.pub_control_enable.publish(Bool(True))

    def lane_follower_set_mode(self):
        time.sleep(1)
        self.pub.publish(Int16(1))
        time.sleep(1)
        self.pub_control_enable.publish(Bool(True))
        

if __name__ == '__main__':
    rospy.init_node('mission_driver')

    weight_fn='weights/yololight_n_240813.pt'
    cam_topic='usb_cam_light/image_raw/compressed'

    detour_mode = rospy.get_param('~mode', 1)
    rospy.loginfo(f'NodeMissionDriver: Detour param set to mode {detour_mode}')
    nmd = NodeMissionDriver(weight_fn=weight_fn, cam_topic=cam_topic, detour_mode=detour_mode)
    nmd.main_loop()