import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree
import numpy as np
import json

LIMIT_DISTANCE = 0.2

class WaypointParser():
    def __init__(self):
        rospy.init_node('waypoint_parser', anonymous = True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=1)
        # p_r = pathReader("morai_msgs")
        # global_waypoints = p_r.read_txt("ego_path.txt")
        with open('link.json', 'r') as file:
            self.link_data = json.load(file)
        

        rate = rospy.Rate(40)

        self.is_odom = False
        self.current_x = 0
        self.current_y = 0
        self.current_link_index = 0
        self.link_waypoints = np.array(self.link_data[self.current_link_index]['points'])[:, :2]
        self.next_link_waypoints = np.array(self.link_data[self.current_link_index + 1]['points'])[:, :2]

        while not rospy.is_shutdown():
            self.waypoints = Path()
            if self.is_odom:
                #print(self.link_waypoints)
                # get closest waypoint index using KDTree
                index = self.get_closest_index_kdtree(self.link_waypoints, self.ego_loc)
                #print("len : ", len(self.link_waypoints))
                #print("len - index : ", len(self.link_waypoints) - index)
                # publish 25 waypoints(12.5m) from the closest waypoint
                # if index is higher than len(global_waypoints) - 25, publish the rest of the waypoints
                if index + 35 > len(self.link_waypoints):
                    if len(self.link_waypoints) - index < 4: # distance between current waypoint and next link waypoint is less than 2m
                        self.update_next_link()
                        continue

                    for i in range(index, len(self.link_waypoints)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.link_waypoints[i][0]
                        tmp_pose.pose.position.y = self.link_waypoints[i][1]
                        tmp_pose.pose.position.z = 0
                        tmp_pose.pose.orientation.x = 0
                        tmp_pose.pose.orientation.y = 0
                        tmp_pose.pose.orientation.z = 0
                        tmp_pose.pose.orientation.w = 1
                        self.waypoints.poses.append(tmp_pose)

                    if len(self.next_link_waypoints) < 35:
                        if(len(self.link_waypoints) - index + len(self.next_link_waypoints) < 20):
                            self.update_next_link()
                            continue
                        for i in range(0, len(self.next_link_waypoints)):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = self.next_link_waypoints[i][0]
                            tmp_pose.pose.position.y = self.next_link_waypoints[i][1]
                            tmp_pose.pose.position.z = 0
                            tmp_pose.pose.orientation.x = 0
                            tmp_pose.pose.orientation.y = 0
                            tmp_pose.pose.orientation.z = 0
                            tmp_pose.pose.orientation.w = 1
                            self.waypoints.poses.append(tmp_pose)
                    else:
                        for i in range(0, 35 - (len(self.link_waypoints) - index)):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = self.next_link_waypoints[i][0]
                            tmp_pose.pose.position.y = self.next_link_waypoints[i][1]
                            tmp_pose.pose.position.z = 0
                            tmp_pose.pose.orientation.x = 0
                            tmp_pose.pose.orientation.y = 0
                            tmp_pose.pose.orientation.z = 0
                            tmp_pose.pose.orientation.w = 1
                            self.waypoints.poses.append(tmp_pose)
                
                else:
                    for i in range(index, index + 35):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.link_waypoints[i][0]
                        tmp_pose.pose.position.y = self.link_waypoints[i][1]
                        tmp_pose.pose.position.z = 0
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
                print("Maybe GPS BLACKOUT SECTION")

    # update next link when the vehicle is close to the end of the current link
    def update_next_link(self):
        if self.current_link_index == len(self.link_data) - 1:
            self.link_waypoints = np.array(self.link_data[self.current_link_index]['points'])[:, :2]
            self.next_link_waypoints = self.link_waypoints
        else:
            self.current_link_index += 1
            self.link_waypoints = np.array(self.link_data[self.current_link_index]['points'])[:, :2]
            self.next_link_waypoints = np.array(self.link_data[self.current_link_index + 1]['points'])[:, :2]
        
        

    def odom_callback(self, odom_msg):
        self.is_odom = True
        self.ego_loc = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]

    def get_closest_index_kdtree(self, waypoints, ego_odom):
        tree = KDTree(waypoints)
        dist, index = tree.query([ego_odom[0], ego_odom[1]])

        # prevent targeting backward waypoint
        if dist > LIMIT_DISTANCE and index < len(waypoints) - 1:
            return index
        else:
            return index + 1
        


if __name__ == '__main__':
    try:
        GPS_IMU_parser = WaypointParser()
    except rospy.ROSInterruptException:
        pass