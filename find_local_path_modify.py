#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from nav_msgs.msg import Path
import tf
from math import sqrt
from geometry_msgs.msg import PoseStamped
from path_reader_1 import pathReader
from morai_msgs.msg import GPSMessage
from gpsimu_parser import GPSIMUParser
from std_msgs.msg import Int16
import yaw_deg
import utm
import time


# global_path와 turtle의 status_msg 이용해 현재 waypoint와 local_path 생성
def find_local_path(ref_path, status_msg, index):
    out_path = Path()
    x_pre = status_msg.longitude
    y_pre = status_msg.latitude
    utm_coord = utm.from_latlon(y_pre, x_pre)
    current_x = utm_coord[0]
    current_y = utm_coord[1]
    current_waypoint = index
    min_dls = float('inf')
 
    if current_waypoint + 10 > len(ref_path.poses):
        last_local_waypoint = len(ref_path.poses)
    else:
        last_local_waypoint = current_waypoint + 10

    # Find currently targeted waypoint by using previous index recursively
    for i in range(current_waypoint, last_local_waypoint):
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dls = sqrt(pow(dx, 2) + pow(dy, 2))
        if dls < min_dls:
            min_dls = dls
            current_waypoint = i    


    out_path.header.frame_id = 'map'
    for i in range(current_waypoint, last_local_waypoint):
        tmp_pose = PoseStamped()
        tmp_pose.pose.position.x = ref_path.poses[i].pose.position.x - 402300 # Subtract East Offset
        tmp_pose.pose.position.y = ref_path.poses[i].pose.position.y - 4132900 # Subtract North Offset
        tmp_pose.pose.position.z = ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x = imu
        tmp_pose.pose.orientation.y = 0
        tmp_pose.pose.orientation.z = 0
        tmp_pose.pose.orientation.w = 1
        out_path.poses.append(tmp_pose)

    return out_path, current_waypoint


class ego_listener():
    def __init__(self):
        rospy.Subscriber("/gps", GPSMessage, self.statusCB)
        self.status_msg = GPSMessage()

    def statusCB(self, data):  ##turtle status subscriber
        self.status_msg = data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.longitude, self.status_msg.latitude, 0),
                         tf.transformations.quaternion_from_euler(0, 0, yaw_deg.yaw_deg),
                         rospy.Time.now(),
                         "ego",
                         "map")


if __name__ == '__main__':
    try: 
        rospy.init_node('local_path_finder', anonymous=True)
        path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        current_index_pub = rospy.Publisher('/current_waypoint', Int16, queue_size=1)
        el = ego_listener()
        # 전역경로로드
        p_r = pathReader("morai_msgs")
        global_path = p_r.read_txt("ego_path.txt")
        current_waypoint = 0
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            # 지역경로생성
            start_time = time.time()
            local_path, current_waypoint = find_local_path(global_path, el.status_msg, current_waypoint)
            index_msg = Int16()
            index_msg.data = current_waypoint
            local_path_pub.publish(local_path)
            current_index_pub.publish(index_msg)
            path_pub.publish(global_path)
            rate.sleep()
            print("delay : " , time.time() - start_time)

    except rospy.ROSInterruptException:
        pass
