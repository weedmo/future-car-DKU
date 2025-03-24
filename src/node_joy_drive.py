#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
import rospy
import numpy as np

manual_angle, manual_speed = 0, 0

def cb_joy(msg: Joy) -> None:
    global manual_angle
    global manual_speed
    manual_speed = int((msg.axes[1]*245+5) if msg.axes[1] > 0 else (msg.axes[1]*245-5))
    manual_angle = int(msg.axes[3]*(600))

if __name__ == '__main__':
    rospy.init_node('joy_drive')
    rospy.Subscriber('/joy', Joy, cb_joy, queue_size=1)
    pub_lat = rospy.Publisher('latt_cmd', Int16, queue_size=1)
    pub_lon = rospy.Publisher('longi_cmd', Int16, queue_size=1)
    msg_lat = Int16()
    msg_lon = Int16()

    loop_hz = 50
    rate = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        rospy.loginfo(f'{manual_angle}, {manual_speed}')
        msg_lat.data = manual_angle
        msg_lon.data = int(manual_speed)
        pub_lat.publish(msg_lat)
        pub_lon.publish(msg_lon)
        rate.sleep()