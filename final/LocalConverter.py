import rospy
import numpy as np
from math import cos, sin
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class localConverter():
    def __init__(self):
        rospy.init_node('local_converter', anonymous=True)
        self.center_lane_sub = rospy.Subscriber('/waypoints', Path, self.center_lane_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.local_center_lane_pub = rospy.Publisher('/local_center_lane', Path, queue_size=1)
        
        self.center_lane = Path()

        self.current_position = Point()
        self.vehicle_yaw = 0
        self.is_center_lane = False
        self.is_odom = False
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            if self.is_center_lane and self.is_odom:
                self.center_lane = self.convert_lane_to_local(self.center_lane, self.current_position)
                self.center_lane.header.frame_id = 'odom'
                
                self.local_center_lane_pub.publish(self.center_lane)
                rate.sleep()
            else:
                pass
                #print("can't subscribe '/left_lane' or '/right_lane' topic... \n    please check your EgoLanePredictor.py connection")

    def center_lane_callback(self, msgs):
        self.center_lane = msgs
        self.is_center_lane = True

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        
    def convert_lane_to_local(self, lane, vehicle_position):
        new_lane = Path()
        new_lane.header.frame_id = 'odom'
        translation = [vehicle_position.x, vehicle_position.y]

        t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]])
        det_t = np.array([
                    [t[0][0], t[1][0], - (t[0][0] * translation[0] + t[1][0] * translation[1])],
                    [t[0][1], t[1][1], - (t[0][1] * translation[0] + t[1][1] * translation[1])],
                    [0, 0, 1]])
        
        #print(det_t)
        print(self.vehicle_yaw)

        for idx, point in enumerate(lane.poses):
            x = point.pose.position.x
            y = point.pose.position.y
            point_matrix = np.array([[x], [y], [1]])
            local_point = det_t.dot(point_matrix)
            # print(local_point[0][0], local_point[1][0], idx)
            lane.poses[idx].pose.position.x = local_point[0][0]
            lane.poses[idx].pose.position.y = local_point[1][0]
            lane.poses[idx].pose.position.z = 0
            lane.poses[idx].pose.orientation.x = 0
            lane.poses[idx].pose.orientation.y = 0
            lane.poses[idx].pose.orientation.z = 0
            lane.poses[idx].pose.orientation.w = 1
            new_lane.poses.append(lane.poses[idx])
        
        return new_lane
        
if __name__ == '__main__':
    try:
        local_converter = localConverter()
    except rospy.ROSInterruptException:
        pass