#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()

g_left_pub = None
g_right_pub = None

def right_callback(msg):
    global g_right_pub
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        if gray[0,0] > 128:
            res = False
        else:
            res = True
        right_msg = Bool(res)
        g_right_pub.publish(right_msg)
    except CvBridgeError, e:
        print(e)


def left_callback(msg):
    global g_left_pub
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        if gray[0,0] > 128:
            res = False
        else:
            res = True
        left_msg = Bool(res)
        g_left_pub.publish(left_msg)
    except CvBridgeError, e:
        print(e)

def main():
    global g_left_pub
    global g_right_pub
    rospy.init_node('image_listener')
    g_left_pub = rospy.Publisher('line_sensor_left', Bool, queue_size=5)
    g_right_pub = rospy.Publisher('line_sensor_right', Bool, queue_size=5)
    rospy.Subscriber("ls_left/ls_left", Image, left_callback)
    rospy.Subscriber("ls_right/ls_right", Image, right_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
