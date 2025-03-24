#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from std_msgs.msg import Int16, Bool
from rocketdan.msg import LaneInfo

from lateral_controller import LateralController

import rospy
import rospkg
import numpy as np
import yaml
import os

class LaneFollower():
    def __init__(self, lateral_controller: LateralController, xtarget=320, delta_max=20, latt_cmd_max=600, control_enable=True):
        # xtarget: center pixel point in the image frame
        # delta_max: actual maximum wheel angle in degree
        # latt_cmd_max: maximum command input for the weel angle in Arduino driver

        rospy.init_node('lane_follower')

        rospy.Subscriber('lane/lane_info', LaneInfo, self.cb_lane_info, queue_size=1)
        rospy.Subscriber('lane/xtarget', Int16, self.cb_xtarget, queue_size=1)
        rospy.Subscriber('lane/control_enable', Bool, self.cb_control_enable, queue_size=1)
        rospy.Subscriber('lane/set_mode', Int16, self.cb_mode, queue_size=10)
        
        self.pub_lat = rospy.Publisher('latt_cmd', Int16, queue_size=1)
        self.pub_lon = rospy.Publisher('longi_cmd', Int16, queue_size=1)

        self.lc = lateral_controller

        self.xtarget = xtarget
        self.from_range = [-delta_max, delta_max]
        self.to_range = [-latt_cmd_max, latt_cmd_max]

        self.set_mode(0) # velocity profile setting according to mission mode
        self.control_enable = control_enable
            
    def set_mode(self, mode):
        # Meaning: if abs(angle) < profile[0]: vx = profile[1]
        if mode == 0: # fastest driving mode
            self.lst_vx_profile = [[30, 255], 
                                   [40, 290], 
                                   [60, 280], 
                                   [1000, 180]]
        elif mode == 1: # for mission driving
            self.lst_vx_profile = [[30, 255], 
                                   [40, 290], 
                                   [60, 280], 
                                   [1000, 180]]


    def main_loop(self, loop_hz = 30):
        rospy.loginfo('NodeLaneFollower: Waiting Topics--------------')
        rospy.wait_for_message('lane/lane_info', LaneInfo)
        rospy.loginfo('NodeLaneFollower: Lane detected --------------')
        rospy.loginfo(f'NodeLaneFollower: run the main thread at {loop_hz}Hz')
        rate = rospy.Rate(loop_hz)

        while not rospy.is_shutdown():
            if not self.control_enable:
                rate.sleep()
                continue

            vx = self.velocity_profile(self.angle)
            delta = self.lc(self.xtarget, self.xpos, self.angle, vx)
            latt_cmd = int(np.interp(delta, self.from_range, self.to_range))
            rospy.loginfo(f'xpos:{int(self.xpos):4d}, angle:{int(self.angle):4d}, lon: {int(vx):4d}, lat: {int(latt_cmd):4d}')
            self.pub_lat.publish(Int16(latt_cmd))
            self.pub_lon.publish(Int16(vx))
            rate.sleep()
    
    def velocity_profile(self, angle):
        angle = abs(90 - angle)
        for profile in self.lst_vx_profile:
            if angle < profile[0]:
                return profile[1]
        return self.lst_vx_profile[-1][1]

    def cb_lane_info(self, msg: LaneInfo) -> None:
        self.xpos = msg.xpos
        self.angle = msg.angle
    def cb_xtarget(self, msg: Int16) -> None:
        self.xtarget = msg.data
    def cb_control_enable(self, msg: Bool) -> None:
        self.control_enable = msg.data
        if self.control_enable:            
            rospy.logerr('NodeLaneFollower: Control has been enabled.')
        else:
            rospy.logerr('NodeLaneFollower: Control has been disabled.')
    def cb_mode(self, msg: Int16) -> None:
        self.set_mode(msg.data)

if __name__ == '__main__':
    def load_points(fn):
        if os.path.exists(fn):
            with open(fn) as f:
                saved_dict = yaml.load(f, Loader=yaml.FullLoader)
            return saved_dict.get(4)['x']

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rocketdan')
    perspective_fn = os.path.join(pkg_path, 'lanedetect/perspective_points.yaml')
    xtarget = load_points(perspective_fn)
            
    lc_stanely = LateralController(k_stanely=3)
    lc_pid = LateralController(pid_angle=LateralController.get_pid_controller(kp=0.01, ki=0.0, kd=0.0),
                               pid_xpos=LateralController.get_pid_controller(kp=0.3, ki=0.05, kd=0.02))
    
    lf = LaneFollower(lateral_controller=lc_pid, xtarget=xtarget, control_enable=True)
    # lf = LaneFollower(lateral_controller=lc_stanely, xtarget=xtarget)
    lf.main_loop()