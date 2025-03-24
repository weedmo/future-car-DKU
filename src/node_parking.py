#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

class NodeParking():
    def __init__(self, mode):
        self.init_params(mode=mode)

        self.steer_pub = rospy.Publisher('latt_cmd', Int16, queue_size=10)
        self.speed_pub = rospy.Publisher('longi_cmd', Int16, queue_size=10)
        self.scan_sub = rospy.Subscriber('/roi_2d', LaserScan, self.scanCallback)

        self.no_point_start_time = None  # 포인트가 감지된 시작 시간
        self.stop_start_time = None  # 주차 로직 중 멈추기 시작한 시간
        self.parking_phase = 0  # 주차 단계 (0: 초기 상태, 1: 멈춤, 2: 전진, 3: 후진, 4: 복귀, 5: 직진 복귀)
        self.parking_complete = False  # 주차 완료 여부
        self.initiate_parking = False  # 주차 로직 실행 여부

        self.msg_steer = Int16()
        self.msg_speed = Int16()

    def init_params(self, mode):
        if mode == 1: # 왼쪽에서 주차
            self.normal_speed = 210   # 기본 전진 속도
                           # steer, speed, duration
            self.command = [[    0,     0,        1],  # 1초 멈춤
                            [  500,   110,        5],  # 주차 로직 시작: 왼쪽으로 조향 및 전진
                            [ -600,   -80,      3.4],  # 후진하면서 반대 조향
                            [    0,  -100,        5],  # 똑바로 후진
                            [    0,     0,        3],  # 후진 완료 후 멈춤
                            [    0,   100,        5],  # 똑바로 전진
                            [ -600,   100,        3],  # 원래 위치로 복귀                             
                            [    0,   200,       15]]  # 탈출 직진

        else: # 오른쪽에서 주차
            self.normal_speed = 210   # 기본 전진 속도
                           # steer, speed, duration
            self.command = [[    0,     0,        1],  # 1초 멈춤
                            [  500,   110,        5],  # 주차 로직 시작: 왼쪽으로 조향 및 전진
                            [ -600,   -80,      3.4],  # 후진하면서 반대 조향
                            [    0,  -100,      4.5],  # 똑바로 후진
                            [    0,     0,        3],  # 후진 완료 후 멈춤
                            [    0,   100,      4.5],  # 똑바로 전진
                            [ -600,   100,        3],  # 원래 위치로 복귀                             
                            [    0,   200,       15]]  # 탈출 직진

    def move(self, steer:int, speed:int):
        self.msg_steer.data = steer
        self.msg_speed.data = speed
        self.steer_pub.publish(self.msg_steer)
        self.speed_pub.publish(self.msg_speed)

    def do_parking(self):        
        if self.parking_phase==9:            
            self.move(0,0)
            rospy.loginfo('NodeParking: Parking completed')
            return
        
        steer=self.command[self.parking_phase-1][0]
        speed=self.command[self.parking_phase-1][1]
        self.move(steer, speed)
        duration=self.command[self.parking_phase-1][2]
        if (rospy.Time.now() - self.stop_start_time).to_sec() > duration:
            self.parking_phase += 1
            self.stop_start_time = rospy.Time.now()
            return
        rospy.loginfo(f'NodeParking: Phase {self.parking_phase}. steer={steer}, speed={speed}, duration={duration:0.2f}, remain {duration-(rospy.Time.now() - self.stop_start_time).to_sec():0.2f}')

    def init_motor(self):
        rate = rospy.Rate(10)
        phase = 0
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if phase==0:
                if (rospy.Time.now()-prev_time).to_sec() > 1:
                    phase=1
                    prev_time=rospy.Time.now()
                self.move(-600, 0)
            else:
                if (rospy.Time.now()-prev_time).to_sec() > 1:
                    return
                self.move(0, 0)
            rate.sleep()
            
    def main_loop(self, loop_hz):
        rospy.loginfo('NodeParking: Waiting -------------------')
        rospy.wait_for_message("/roi_2d", LaserScan)
        rospy.loginfo("NodeParking: roi_2d Ready --------------")

        rate = rospy.Rate(loop_hz)
        rospy.loginfo('NodeParking: initializing motor')
        self.init_motor()
        rospy.loginfo('NodeParking: go forward')
        # self.initiate_parking=True
        # self.parking_phase=1
        # self.stop_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.initiate_parking:
                self.do_parking()
            else:
                self.move(0, self.normal_speed)
            rate.sleep()

    def scanCallback(self, scan: LaserScan):
        # LiDAR 데이터에서 전방의 장애물 거리 확인
        valid_ranges = [r for r in scan.ranges if r > scan.range_min and r < scan.range_max]

        if not self.initiate_parking:
            current_time = rospy.Time.now()
           
            if valid_ranges:
                if self.no_point_start_time is None:
                    # 유효한 포인트가 처음 감지된 시간 기록
                    self.no_point_start_time = current_time
                    rospy.loginfo("Valid points detected")
            else:
                if self.no_point_start_time is not None:
                    # 유효한 포인트가 사라진 경우
                    time_since_detection = (current_time - self.no_point_start_time).to_sec()
                    rospy.loginfo(f"Time since detection: {time_since_detection}")
                   
                    if 0.3 < time_since_detection < 3.0:
                        self.initiate_parking = True
                        self.parking_phase = 1
                        self.stop_start_time = current_time
                        rospy.loginfo("Parking initiated")
                    else:
                        rospy.loginfo("Detection time out of range")
                   
                    self.no_point_start_time = None  # 타이머 리셋



if __name__ == '__main__':
    rospy.init_node('NodeParking')
    
    mode = rospy.get_param('~mode', 1)
    rospy.loginfo(f'NodeParking: mode {mode}')
    np = NodeParking(mode)

    np.main_loop(10)