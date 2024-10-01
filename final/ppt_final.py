#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int16, String
from morai_msgs.msg import CtrlCmd
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from config import config
import time

class pure_pursuit:
    config = config

    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("waypoints", Int16, self.index_callback)
        rospy.Subscriber("ped_result", String, self.pedestrian_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.prev_velocity = 0

        self.is_path = False
        self.is_odom = False
        self.is_index = False
        self.is_pedestrian_detected = False
        self.ignore_ped_detection_time = 2  # 보행자 인식 무시 시간 (초)
        self.last_pedestrian_time = time.time()  # 마지막 보행자 감지 시간


        self.forward_point = Point()
        self.current_position = Point()
        self.waypoints = Int16()
        self.is_look_forward_point = False
        self.vehicle_length = 4.635
        self.lfd = 7
        self.prev_velocity = 0

        self.steering_buffer_size1 = 2
        self.steering_buffer_size2 = 1
        self.steering_buffer = []

        rate = rospy.Rate(40)  # 40hz
        while not rospy.is_shutdown():
            if self.is_pedestrian_detected:
                self.ctrl_cmd_msg.velocity = 0  # 보행자 감지 시 속도를 0으로 설정
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                time.sleep(3.5)  # 3.5초간 대기
                self.last_pedestrian_time = time.time()  # 재출발 시간 갱신
                self.is_pedestrian_detected = False  # 보행자 감지 상태 초기화
            elif (self.is_path == True) and (self.is_odom == True):
                if not self.is_look_forward_point:
                    # Set the initial waypoint when it's not set yet
                    if self.path.poses:
                        self.waypointspoint.data = 0
                        self.is_look_forward_point = True
                vehicle_position = self.current_position
                self.is_look_forward_point = False
                translation = [vehicle_position.x, vehicle_position.y]
                current_index = self.waypoints.data
                print("current waypoint : ", current_index)

                t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]]) 
                det_t = np.array([
                    [t[0][0], t[1][0], - (t[0][0] * translation[0] + t[1][0] * translation[1])],
                    [t[0][1], t[1][1], - (t[0][1] * translation[0] + t[1][1] * translation[1])],
                    [0, 0, 1]])
                
                for i, waypoint in enumerate(self.path.poses):
                    path_point = waypoint.pose.position

                    # Transform the global path point to the vehicle's local coordinate
                    global_path_point = [path_point.x, path_point.y, 1]
                    local_path_point = det_t.dot(global_path_point)

                    #print('real point :',current_index)
                    #print(local_path_point)

                    if local_path_point[0] > 0:
                        dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                        if i == 0 or dis >= self.lfd:
                            self.forward_point = path_point
                            self.is_look_forward_point = True
                            break

                theta = atan2(local_path_point[1], local_path_point[0])
                
                if self.is_look_forward_point:
                    self.adjust_velocity_and_steering(current_index, theta)

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    @staticmethod
    def find_index_of_config(index):
        left, right = 0, len(pure_pursuit.config) - 1
        while left <= right:
            mid = (left + right) // 2
            start, end, _ = pure_pursuit.config[mid]
            if start <= index < end:
                return mid
            elif index < start:
                right = mid - 1
            else:
                left = mid + 1
        return None

    def adjust_velocity_and_steering(self, current_index, theta):
        idx = self.find_index_of_config(current_index)
        config_data = config[idx][2]

        steering_angle_factor = abs(self.ctrl_cmd_msg.steering) / (pi / 4)
        speed_reduction = config_data['base_speed'] * steering_angle_factor * config_data['speed_reduction_factor']
        self.lfd = config_data['lfd_base'] + (self.ctrl_cmd_msg.velocity - 7) / 5
        self.ctrl_cmd_msg.velocity = config_data['base_speed'] - speed_reduction
        raw_steering_angle = atan2((2 * self.vehicle_length * sin(theta)), self.lfd) / config_data['raw_steering_divider']
                
        print('Self.lfd : ', self.lfd)
        print("theta : ", theta)
        print("velocity : ", self.ctrl_cmd_msg.velocity)
        print("delta : ", raw_steering_angle)

        if len(self.steering_buffer) < self.steering_buffer_size2:
            self.steering_buffer.append(raw_steering_angle)
        else:
            self.steering_buffer.pop(0)
            self.steering_buffer.append(raw_steering_angle)

        moving_average_steering = sum(self.steering_buffer) / len(self.steering_buffer)
        weighted_steering = moving_average_steering * (1 - config_data['weight_factor'] * abs(moving_average_steering))
        
        if abs(weighted_steering) < config_data['steering_threshold']:
            self.ctrl_cmd_msg.steering = 0
        else:
            self.ctrl_cmd_msg.steering = weighted_steering
        
        print(self.ctrl_cmd_msg.steering, self.ctrl_cmd_msg.velocity)
        
    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def index_callback(self, msg):
        self.is_index = True
        self.waypoints = msg
    def pedestrian_callback(self, msg):
        current_time = time.time()
        if 385 <= self.waypoints.data <= 400 or 187 <= self.waypoints.data <= 192: 
            self.is_pedestrian_detected = False
            return  # 이 구간에서는 어떤 경우에도 감지 상태를 변경하지 않음
        if msg.data == "ped" and (current_time - self.last_pedestrian_time > self.ignore_ped_detection_time):
            self.is_pedestrian_detected = True
if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass