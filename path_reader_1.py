#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

class pathReader:
    def __init__(self,morai_msgs):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(morai_msgs)

    def read_txt(self,file_name):
        full_file_name = self.file_path+"/"+file_name
        openFile = open(full_file_name,'r')
        out_path = Path()
        out_path.header.frame_id = '/map'
        #파일 한줄 --> waypoint 한개
        line=openFile.readlines()
        for i in line:
            tmp = i.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.z = 0.0
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        openFile.close()
        return out_path
    
if __name__ == '__main__':
    try:
        p_r=pathReader("morai_msgs")
        global_path = p_r.read_txt("ego_path.txt")
        rospy.init_node("path_reader_1", anonymous=True)
        path_pub = rospy.Publisher('/global_path',Path,queue_size=1)

        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            start_time = time.time()
            path_pub.publish(global_path)
            rate.sleep()
            print("delay : " , time.time() - start_time)

    except rospy.ROSInterruptException:
        pass
