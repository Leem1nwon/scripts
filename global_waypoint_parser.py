import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from path_reader_1 import pathReader
from scipy.spatial import KDTree
import numpy as np
import time
LIMIT_DISTANCE = 0.3

class WaypointParser():
    def __init__(self):
        rospy.init_node('waypoint_parser', anonymous = True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=1) 

        p_r = pathReader("morai_msgs")
        global_waypoints = p_r.read_txt("ego_path.txt")
        rate = rospy.Rate(40)

        self.is_odom = False
        self.current_x = 0
        self.current_y = 0

        while not rospy.is_shutdown():
            self.waypoints = Path()
            if self.is_odom:
                # get closest waypoint index using KDTree
                index = self.get_closest_index_kdtree(global_waypoints, self.ego_loc)

                # publish 10 waypoints from the closest waypoint
                # if index is higher than len(global_waypoints) - 10, publish the rest of the waypoints
                if index + 25 > len(global_waypoints.poses):
                    last_local_waypoint = len(global_waypoints.poses)
                else:
                    last_local_waypoint = index + 25
                
                for i in range(index, last_local_waypoint):
                    tmp_pose = PoseStamped()
                    tmp_pose.pose.position.x = global_waypoints.poses[i].pose.position.x - 402300
                    tmp_pose.pose.position.y = global_waypoints.poses[i].pose.position.y - 4132900
                    tmp_pose.pose.position.z = global_waypoints.poses[i].pose.position.z
                    tmp_pose.pose.orientation.x = 0
                    tmp_pose.pose.orientation.y = 0
                    tmp_pose.pose.orientation.z = 0
                    tmp_pose.pose.orientation.w = 1
                    self.waypoints.poses.append(tmp_pose)
                
                print("current waypoint : ", index)
                print(len(self.waypoints.poses))

                self.waypoints_pub.publish(self.waypoints)
                rate.sleep()
                
                
            else:
                print("can't subscribe '/odom' topic... \n    please check your gpsimu_parser.py connection")

    def odom_callback(self, odom_msg):
        self.is_odom = True
        self.ego_loc = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]

    def get_closest_index_kdtree(self, waypoints, ego_odom):
        waypoint_coords = np.array([(point.pose.position.x - 402300, point.pose.position.y - 4132900) for point in waypoints.poses])
        
        tree = KDTree(waypoint_coords)
        dist, index = tree.query([ego_odom[0], ego_odom[1]])

        # prevent targeting backward waypoint
        if dist > LIMIT_DISTANCE and index < len(waypoints.poses) - 1:
            return index
        else:
            return index + 1
        


if __name__ == '__main__':
    try:
        GPS_IMU_parser = WaypointParser()
    except rospy.ROSInterruptException:
        pass