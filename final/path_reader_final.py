#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathReader:
    def __init__(self, package_name):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path(package_name)

    def read_json(self, file_name):
        full_file_name = self.file_path + "/" + file_name
        with open(full_file_name, 'r') as openFile:
            data = json.load(openFile)  # JSON 파일 읽기
            
        out_path = Path()
        out_path.header.frame_id = '/map'
        
        # JSON 내의 포인트 배열 파싱
        for segment in data:
            for point in segment['points']:
                read_pose = PoseStamped()
                read_pose.pose.position.x = point[0]  # x 좌표
                read_pose.pose.position.y = point[1]  # y 좌표
                read_pose.pose.position.z = 0.0  # z 좌표는 사용하지 않음
                read_pose.pose.orientation.x = 0
                read_pose.pose.orientation.y = 0
                read_pose.pose.orientation.z = 0
                read_pose.pose.orientation.w = 1
                out_path.poses.append(read_pose)

        return out_path

if __name__ == '__main__':
    try:
        p_r = PathReader("morai_msgs")
        global_path = p_r.read_json("link.json")  # JSON 파일 이름 확인 필요
        rospy.init_node("path_reader_final", anonymous=True)
        path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            path_pub.publish(global_path)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass