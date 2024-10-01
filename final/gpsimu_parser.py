#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import numpy as np
from math import cos, sin
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
#from tf import euler_from_quaternion

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        # initialization
        self.xy_buffer = None
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        self.last_time = rospy.get_time()
        

        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        # Kalman Filter setting
        self.state = np.zeros(4)[:, None] # x, y, velocity_x, velocity_y
        self.P = np.eye(4) * 0.1
        self.process_noise = np.diag([0, 0, 0, 0]) #
        self.covariance = np.eye(4) * 0.1
        self.measurement_noise = np.diag([1e-4, 1e-4, 1e-4, 1e-4])
        self.dt = 0.025 # 40Hz
        self.theta = 0
        self.vehicle_yaw = 0

        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_imu and self.is_gps:
                self.convertLL2UTM()
                #print("x : " , self.state[0], "\n y : ", self.state[1])
                self.odom_pub.publish(self.odom_msg)
                print(f"odom_msg is now being published at '/odom' topic!\n")
                print('-----------------[ odom_msg ]---------------------')
                print(self.odom_msg.pose)

            if not self.is_gps and self.is_imu:
                print("GPS BLACKOUT SECTION. Start estimating position using IMU data...")
                self.estimate_position_using_imuData()
                print(self.state)
            if not self.is_gps and not self.is_imu:
                print("NO GPS, NO IMU. Waiting for GPS and IMU data...")
            
            self.is_gps = self.is_imu = False
            rate.sleep()

    def estimate_position_using_imuData(self):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        acceleration = (self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.linear.y)
        angular_velocity = self.odom_msg.twist.twist.angular.z
        self.theta += angular_velocity * dt
        # _, _, self.vehicle_yaw = euler_from_quaternion([self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w])
        self.predict(acceleration, dt)
        self.update(self.state)

        self.odom_msg.pose.pose.position.x = self.state[0]
        self.odom_msg.pose.pose.position.y = self.state[1]
        self.odom_msg.pose.pose.position.z = 0.0


    def predict(self, acceleration, dt):
        A = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        B = np.array([[0.5 * acceleration[0] * dt**2],
                      [0.5 * acceleration[1] * dt**2],
                      [acceleration[0] * dt],
                      [acceleration[1] * dt]])
        
        print("Linear acceleration : ", acceleration)
        self.state = np.dot(A, self.state) + B
        print("FIrst shape : ", np.shape(self.state))
        self.covariance = np.dot(np.dot(A, self.covariance), A.T) + self.process_noise
        print("Second shape : ", np.shape(self.covariance))

        # self.odom_msg.pose.pose.position.x = self.state[0]
        # self.odom_msg.pose.pose.position.y = self.state[1]
        # self.odom_msg.pose.pose.position.z = 0.0

    def update(self, measurement):
        H = np.eye(4)
        S = np.dot(np.dot(H, self.covariance), H.T) + self.measurement_noise
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(S))
        self.state += np.dot(K, (measurement - np.dot(H, self.state)))
        I = np.eye(4)
        self.covariance = np.dot((I - np.dot(K, H)), self.covariance)
        
        

    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        if self.lat == 0 and self.lon == 0:
            self.is_gps = False
        else:
            self.is_gps=True

    def convertLL2UTM(self):
        xy_zone = self.proj_UTM(self.lon, self.lat)

        if self.lon == 0 and self.lat == 0:
            self.x = 0.0
            self.y = 0.0
        else:
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o

        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.
        self.state[0] = self.x
        self.state[1] = self.y

    def imu_callback(self, data):
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
            self.odom_msg.twist.twist.linear.x = data.linear_acceleration.x
            self.odom_msg.twist.twist.linear.y = data.linear_acceleration.y
            self.odom_msg.twist.twist.angular.z = data.angular_velocity.z

        self.is_imu=True

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
