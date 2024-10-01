import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time


LANE_OFFSET = 1.5

class EgoLanePredictor():
    def __init__(self):
        rospy.init_node('ego_lane_predictor', anonymous=True)
        self.local_wp_sub = rospy.Subscriber('/local_center_lane', Path, self.waypoints_callback)

        self.left_lane_pub = rospy.Publisher('/left_lane', Path, queue_size=1)
        self.right_lane_pub = rospy.Publisher('/right_lane', Path, queue_size=1)
        self.curvature_pub = rospy.Publisher('/curvature', Float32, queue_size=1)
        rate = rospy.Rate(40)
        #initialization
        self.is_waypoints = False
        self.curvature = 0

        while not rospy.is_shutdown():
            if self.is_waypoints:
                self.predict_ego_lane(self.waypoints)
                self.left_lane.header.frame_id = 'odom'
                self.right_lane.header.frame_id = 'odom'

                self.left_lane_pub.publish(self.left_lane)
                self.right_lane_pub.publish(self.right_lane)
                self.curvature_pub.publish(self.curvature)
                print("ego lane predicted")

                rate.sleep()
            else:
                print("can't subscribe '/waypoints' topic... \n    please check your global_waypoint_parser.py connection")
     
    # bring waypoints from global_waypoint_parser.py
    # waypoints consist of 10 waypoints
    # the funciton predict_ego_lane should predict the ego lane using np.polyfit
    # and publish the predicted ego lane to '/ego_lane' topic
    # the predicted ego lane should be a Path message
    # the Path message should be published at 40Hz
    def predict_ego_lane(self, waypoints):
        waypoints = self.waypoints
        x = np.array([waypoint.pose.position.x for waypoint in waypoints.poses])
        y = np.array([waypoint.pose.position.y for waypoint in waypoints.poses])

        # polyfit
        z = np.polyfit(x, y, 2)
        p = np.poly1d(z)

        first_derivative = p.deriv()
        second_derivative = first_derivative.deriv()
        curvature = abs(second_derivative(x[10])) / (1 + first_derivative(x[10])**2)**1.5
        self.curvature = curvature
        print(curvature)

        # initialize
        left_lane = Path()
        right_lane = Path()
        
        # y_new is about the center line of the road
        # but we are interested in the ego lane which consists of two lanes
        # so we need to predict the left and right ego lanes
        # the left ego lane is LANE_OFFSET meters left from the center line
        # the right ego lane is LANE_OFFSET meters right from the center line
        # and we have to consider the slope of the center line
        # the slope of the center line is the derivative of the polyfit function
        # and the slope of the left and right ego lanes are the slope of the center line plus 90 degrees
        # the left and right ego lanes are the lines that are parallel to the center line
        # so we can get the left and right ego lanes by using the slope and the offset

        # Ego lane
        for i in range(len(x)):
            pose_left = PoseStamped()
            pose_right = PoseStamped() 
            p_derivative = p.deriv()
            slope = p_derivative(x[i])

            # if slope is 0, the slope of the left ego lane is 0.0001
            if slope == 0:
                slope = 0.0001
            
            orthogonal_slope = -1 / slope
            orthogonal_angle = math.atan(orthogonal_slope)
            if orthogonal_angle < 0:
                orthogonal_angle += math.pi
            #print(orthogonal_angle)
            
            pose_left.pose.position.x = x[i]
            pose_left.pose.position.y = y[i] + LANE_OFFSET * math.sin(orthogonal_angle)
            pose_right.pose.position.x = x[i]
            pose_right.pose.position.y = y[i] - LANE_OFFSET * math.sin(orthogonal_angle)
            
            # print(pose_left.pose.position.x, pose_left.pose.position.y, pose_right.pose.position.x, pose_right.pose.position.y)
            left_lane.poses.append(pose_left)
            right_lane.poses.append(pose_right)

        self.left_lane = left_lane
        self.right_lane = right_lane

    def waypoints_callback(self, msgs):
        self.waypoints = msgs
        self.is_waypoints = True
     

if __name__ == '__main__':
    try:
        EgoLanePredictor()
    except rospy.ROSInterruptException:
        pass