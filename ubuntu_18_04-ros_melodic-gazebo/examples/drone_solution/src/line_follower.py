#!/usr/bin/env python
# coding: utf-8

import threading

import time
from math import sin, cos
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from hector_uav_msgs.srv import EnableMotors

import cv2
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
import tf2_ros
import tf.transformations as ttt

PLANING_HORIZON = 50
TIME_LIFTOFF = 3

RING_AVOIDANCE_TIME = 5 # [SECONDS]
DEFAULT_ALTITUDE = 3 # [METERS]

V_MAX = 2.05
W_MAX = 0.35

Kp_z = 0.5

Kp_y =  0.015
Kd_y =  0.000045
Ki_y =  0.0000825

Kp_w =  0.01555
Kd_w =  0.000095
Ki_w =  0.000165


class SimpleMover():

    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        self.cv_bridge = CvBridge()

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("cam_1/camera/image", Image, self.camera_callback)
        rospy.Subscriber("cam_2/camera/image", Image, self.camera_rings_callback)
        
        rospy.Subscriber('/ground_truth/state', Odometry, self.obom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.z_des = DEFAULT_ALTITUDE

        self.drone_state = [0] * 6  # position vector
        self.e_y = 0
        self.e_omega_z = 0
        self.rate = rospy.Rate(30)

        self.omega_error = 0
        self.y_error = 0

        self.input_kp = 0.015
        self.input_kd = 0.000045

        self.image_1 = []
        self.image_2 = []

        self.state = "free_flight"
        self.red_ring_detected = False
        self.blue_ring_detected = False
        self.time_start_up = 0
        self.avoidance_time = 0
        self.e_x_blue, self.e_y_blue = 0, 0

    def obom_callback(self, msg):
        """ Pose of a robot extraction"""
        try:
            transform = self.tfBuffer.lookup_transform('world', 'base_stabilized', rospy.Time(0)).transform
            x, y, z = transform.translation.x, transform.translation.y, transform.translation.z
            quat = transform.rotation
            r, p, y = ttt.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            self.drone_state = [x, y, z, r, p, y]
        except:
            print('Cant get transform.')

    def camera_callback(self, msg):
        """ Computer vision stuff"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(cv_image, (160, 0), (160, 240), (0, 123, 0), 1)
        cv2.line(cv_image, (0, 120), (320, 120), (0, 123, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[msg.height / 2] >= 10)
        if  (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point
            
            cv2.circle(cv_image, (top_line_point, 10), 5, (0,0,255), 1)
            cv2.circle(cv_image, (mid_line_point, int(msg.height/2)), 5, (0,0,255), 1)
            cv2.line(cv_image, (mid_line_point, int(msg.height/2)), (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy

            cv2.circle(cv_image, (cy, int(msg.height/2)), 7, (0,255,0), 1)
            cv2.line(cv_image, (160, 120), (cy, int(msg.height/2)), (0, 255, 0), 3)

        self.image_1 = cv_image

    def ring_detector(self, image, lower, upper, color):
       color_mask = cv2.inRange(image, lower, upper)
       color_contours, _ = cv2.findContours(color_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
       if color_contours:
           max_len_c = 0
           c = color_contours[0]
           for i in range(0, len(color_contours)):
               if len(color_contours[i]) > max_len_c:
                   c = color_contours[i]
                   max_len_c = len(color_contours[i])
           self.color_distance = max_len_c
           M = cv2.moments(c)
           if M['m00'] != 0:
               cx = int(M['m10']/M['m00'])
               cy = int(M['m01']/M['m00'])
           else:
               cx = 0
               cy = 0
           (x1,y1), color_r = cv2.minEnclosingCircle(c)
           if color_r > 10:
               image = cv2.circle(image, (cx, cy), radius=5, color=color, thickness=-1)
               cv2.drawContours(color_r, c, -1, (0,255,0), 1)
               color_r = cv2.circle(color_r, (int(x1), int(y1)), radius=int(color_r), color=color, thickness=4)       
               return image, (x1,y1), color_r[0]
       return image, (0,0), 0

    def camera_rings_callback(self, msg):
       # """ Computer vision stuff for Rings"""
       try:
           cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
       except CvBridgeError as e:
           rospy.logerr("CvBridge Error: {0}".format(e))
 
       # red
       lower = np.uint8([0, 0, 90])
       upper = np.uint8([30, 30, 120])
       cv_image, red_pose, red_radius  = self.ring_detector(cv_image, lower, upper, (0,0,255))
 
       # blue
       lower = np.uint8([40, 20, 20])
       upper = np.uint8([80, 50, 50])
       cv_image, blue_pose, blue_radius = self.ring_detector(cv_image, lower, upper, (255,0,0))
 
       # print(red_radius, blue_radius)
 
       if 50 < red_radius < 70 or 50 < blue_radius < 80:
           if red_radius > blue_radius:
               self.blue_ring_detected = False
               self.red_ring_detected = True
           else:
               self.red_ring_detected = False
               self.blue_ring_detected = True
              
               # offset in ring xy-plane to fly through center of a ring
               # error = <center of image> - <center of ring>
               self.e_x_blue = 160 - blue_pose[0]
               self.e_y_blue = 120 - blue_pose[1]
       else:
           self.blue_ring_detected = False
           self.red_ring_detected = False
 
       # save results
       self.image_2 = cv_image

    def show_image(self, img, title='Camera 1'):
        cv2.imshow(title, img)
        cv2.waitKey(3)

    def enable_motors(self):
        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ", e)

    def read_data(self):
        while not rospy.is_shutdown():
            self.input_kp, self.input_kd = [float(s) for s in raw_input().split()]

    def fsm_update(self):
        if self.red_ring_detected:
            self.state = "drone_up"
            self.time_start_up = rospy.get_time()
        elif self.blue_ring_detected:
            self.state = "drone_blue_ring"
            self.time_start_up = rospy.get_time()
        elif RING_AVOIDANCE_TIME <  self.avoidance_time:
            self.state = "drone_down"
        else:
            self.state = "free_flight"

    def spin(self):
        self.enable_motors()
        
        # Initialisations
        altitude_prev = 0
        y_error_prev = 0
        omega_error_prev = 0

        alpha = self.drone_state[5]

        time_start = rospy.get_time()
        time_prev = time_start

        prev_input_kp = 0
        prev_input_kd = 0

        while not rospy.is_shutdown():

            self.fsm_update()

            if self.state == "drone_up":
                self.z_des = 5
            elif self.state == "drone_down":
                self.z_des = DEFAULT_ALTITUDE
            elif self.state == "drone_blue_ring":
                self.z_des += 0.001 * self.e_y_blue
                print(self.e_y_blue)
                pass
            elif self.state == "free_flight":
                pass
            else:
                rospy.logerr("Error: state name error!")
     
            self.avoidance_time = rospy.get_time() - self.time_start_up

            print(self.state, self.z_des)

            if len(self.image_1) > 0 and len(self.image_2) > 0:
                self.show_image(self.image_1, title='Line')
                self.show_image(self.image_2, title='Rings')

            if (self.input_kp != prev_input_kp) | (self.input_kd != prev_input_kd):
                print(self.input_kp)
                print(self.input_kd)
                prev_input_kp = self.input_kp
                prev_input_kd = self.input_kd

            try:
                # Time stuff
                t = rospy.get_time() - time_start
                dt = t - time_prev
                time_prev = t
                if dt == 0:
                    dt = 1 / 30.

                # TODO: Write here altitude controller
                # Here!
                error_z = self.z_des - self.drone_state[2]
                error_z_prev = self.z_des - altitude_prev

                u_z = 4 * error_z + 0.09 * (error_z - error_z_prev) / dt 

                altitude_prev = self.drone_state[2]

                # TODO: Steering control
                # Here!
                u_omega_z = -0.005 * self.omega_error - 0.001 * (self.omega_error - omega_error_prev) / dt
                omega_error_prev = self.omega_error

                # TODO: Offset control
                # Here!
                u_y = self.input_kp * self.y_error - self.input_kd * (self.y_error - y_error_prev) / dt

                if (error_z < 0.2):
                    if (abs(u_y) < 0.1):
                        u_x = 3
                    else:
                        u_x = 0.3
                else:
                    u_x = 0.1

                y_error_prev = self.y_error

                twist_msg = Twist()
                twist_msg.linear.x = u_x
                twist_msg.linear.y = u_y
                twist_msg.linear.z = u_z
                twist_msg.angular.z = u_omega_z
                self.cmd_vel_pub.publish(twist_msg)

            except KeyboardInterrupt:
                break

            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__=="__main__":
    simple_mover = SimpleMover()
    threading._start_new_thread(simple_mover.read_data, ())
    simple_mover.spin()
