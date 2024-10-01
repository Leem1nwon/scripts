#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd

class PathMaker:
    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2  # 속도 제어 유형 설정

        self.drive_started = False
        self.current_section = 1
        self.section_start_time = None
        self.section_duration = {
            1: rospy.Duration(8),  # 첫 번째 구간 10초 동안 주행
            2: rospy.Duration(12),  # 두 번째 구간 10초 동안 주행
            3: rospy.Duration(10),   # 세 번째 구간 10초 동안 주행
            4: rospy.Duration(1)
        }

    def drive(self):
        if not self.drive_started:
            # 주행 시작 시간을 현재 시간으로 설정
            self.section_start_time = rospy.Time.now()
            self.drive_started = True

        if self.drive_started and self.current_section <= 3:
            current_time = rospy.Time.now()
            if current_time - self.section_start_time < self.section_duration[self.current_section]:
                if self.current_section == 1:
                    self.ctrl_cmd_msg.velocity = 5.0  # 구간 1의 속도 설정
                    self.ctrl_cmd_msg.steering = 0
                elif self.current_section == 2:
                    self.ctrl_cmd_msg.velocity = 5.0   # 구간 2의 속도 설정
                    self.ctrl_cmd_msg.steering = 0.03
                elif self.current_section == 3:
                    self.ctrl_cmd_msg.velocity = 5.0   # 구간 3의 속도 설정
                    self.ctrl_cmd_msg.steering = -0.008
                elif self.current_section == 4:
                    self.ctrl_cmd_msg.velocity = 5.0   # 구간 3의 속도 설정
                    self.ctrl_cmd_msg.steering = 0    
            else:
                self.section_start_time = current_time
                self.current_section += 1
                if self.current_section > 3:
                    self.ctrl_cmd_msg.velocity = 0  # 마지막 구간 이후 속도를 0으로 설정
                    self.drive_started = False  # 주행 완료 후 다시 시작하지 않도록 설정

            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 제어 명령 발행

if __name__ == '__main__':
    try:
        path_maker = PathMaker()
        rate = rospy.Rate(40)  # 40Hz에서 동작
        while not rospy.is_shutdown():
            path_maker.drive()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
