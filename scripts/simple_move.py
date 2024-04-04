#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
import time
from math import sin, cos

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import Int16  # For error/angle plot publishing

from hector_uav_msgs.srv import EnableMotors

from cv_bridge import CvBridge, CvBridgeError


class SimpleMover:

    def __init__(self):
        rospy.init_node('simple_mover', anonymous=True)

        if rospy.has_param('/profi2021_bachelor_solution/altitude_desired'):
            self.altitude_desired = rospy.get_param(
                '/profi2021_bachelor_solution/altitude_desired')  # in solution.launch
        else:
            rospy.logerr("Failed to get param '/profi2021_bachelor_solution/altitude_desired'")

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(30)
        self.pub_error = rospy.Publisher('error', Int16, queue_size=10)
        self.pub_angle = rospy.Publisher('angle', Int16, queue_size=10)
        rospy.Subscriber("cam_1/camera/image", Image, self.folow_the_line)
        self.cv_bridge = CvBridge()
        self.Kp = 0.15
        self.Ki = 0
        self.kd = 1
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.Kp_ang = 0.04
        self.Ki_ang = 0
        self.kd_ang = -0.01
        self.integral_ang = 0
        self.derivative_ang = 0
        self.last_ang = 0
        self.error = []
        self.angle = []
        self.velocity = 0.8

        rospy.on_shutdown(self.shutdown)

    @staticmethod
    def enable_motors():

        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: " + str(response))
            print(e)

    @staticmethod
    def camera_zoom(cv_image, scale):
        height, width, _ = cv_image.shape
        # prepare the crop
        center_x, center_y = int(height / 2), int(width / 2)
        radius_x, radius_y = int(scale * height / 100), int(scale * width / 100)

        min_x, max_x = center_x - radius_x, center_x + radius_x
        min_y, max_y = center_y - radius_y, center_y + radius_y

        cv_image = cv_image[min_x:max_x, min_y:max_y]
        cv_image = cv2.resize(cv_image, (width, height))

        return cv_image

    def take_off(self):

        self.enable_motors()

        start_time = time.time()
        end_time = start_time + 3
        twist_msg = Twist()
        twist_msg.linear.z = self.altitude_desired

        while (time.time() < end_time) and (not rospy.is_shutdown()):
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

    def folow_the_line(self, msg):

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        if int(self.altitude_desired) >= 5:
            cv_image = self.camera_zoom(cv_image, scale=20)
            mask = cv2.inRange(cv_image, (20, 20, 20), (130, 130, 130))
        elif int(self.altitude_desired) <= 2.4:
            cv_image = self.camera_zoom(cv_image, scale=20)
            mask = cv2.inRange(cv_image, (0, 0, 0), (30, 30, 30))
        elif 2.4 < int(self.altitude_desired) <= 3.5:
            cv_image = self.camera_zoom(cv_image, scale=150)
            mask = cv2.inRange(cv_image, (20, 20, 20), (130, 130, 130))
        else:
            cv_image = self.camera_zoom(cv_image, scale=35)
            mask = cv2.inRange(cv_image, (20, 20, 20), (130, 130, 130))
        # cv_image = cv2.add(cv_image, np.array([-50.0]))

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=5)
        mask = cv2.dilate(mask, kernel, iterations=9)
        _, contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk.sort(key=cv2.minAreaRect)

        if len(contours_blk) > 0 and cv2.contourArea(contours_blk[0]) > 500:
            if int(self.altitude_desired) > 2.4:
                line_left_side = cv2.minAreaRect(contours_blk[0])
                line_right_side = cv2.minAreaRect(contours_blk[-1])
                (x_left, y_left), (w_left, h_left), angle_left = line_left_side
                (x_right, y_right), (w_right, h_right), angle_right = line_right_side
                x_min, y_min, w_min, h_min, angle = (x_left + x_right) / 2, (y_left + y_right) / 2, (
                        w_left + w_right) / 2, (h_left + h_right) / 2, (angle_left + angle_right) / 2
            else:
                line = cv2.minAreaRect(contours_blk[0])
                (x_min, y_min), (w_min, h_min), angle = line

            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle

            set_point = cv_image.shape[1] / 2
            error = int(x_min - set_point)
            self.error.append(error)
            self.angle.append(angle)
            normal_error = float(error) / set_point

            self.integral = float(self.integral + normal_error)
            self.derivative = normal_error - self.last_error
            self.last_error = normal_error

            error_corr = -1 * (
                    self.Kp * normal_error + self.Ki * self.integral + self.kd * self.derivative)  # PID controler

            angle = int(angle)

            self.integral_ang = float(self.integral_ang + angle)
            self.derivative_ang = angle - self.last_ang
            self.last_ang = angle

            ang_corr = -1 * (
                    self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler
            if int(self.altitude_desired) > 2.4:
                box_left = cv2.boxPoints(line_left_side)
                box_left = np.int0(box_left)

                cv2.drawContours(cv_image, [box_left], 0, (0, 0, 255), 3)

                box_right = cv2.boxPoints(line_right_side)
                box_right = np.int0(box_right)

                cv2.drawContours(cv_image, [box_right], 0, (0, 0, 255), 3)

            else:
                box = cv2.boxPoints(line)
                box = np.int0(box)

                cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 3)

            cv2.putText(cv_image, "Angle: " + str(angle), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

            cv2.putText(cv_image, "Error: " + str(error), (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                        cv2.LINE_AA)
            cv2.line(cv_image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

            twist = Twist()
            twist.linear.x = self.velocity
            twist.linear.y = error_corr
            twist.angular.z = ang_corr
            self.cmd_vel_pub.publish(twist)

            ang = Int16()
            ang.data = angle
            self.pub_angle.publish(ang)

            err = Int16()
            err.data = error
            self.pub_error.publish(err)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1) & 0xFF

    def start(self):
        self.take_off()
        while not rospy.is_shutdown():
            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    mover = SimpleMover()
    mover.start()
