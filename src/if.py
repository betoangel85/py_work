#!/usr/bin/env python

import roslib; roslib.load_manifest('py_work')

import numpy as	np
import rospy
import sys
import cv2
from sensor_msgs.msg import	Image, CameraInfo
from cv_bridge import CvBridge,	CvBridgeError
from collections import deque

colorLower = (24, 100, 100)
colorUpper = (44, 255, 255)

class cvBridgeDemo():

    def	__init__(self):
        self.node_name = 'cv_bridge'
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge	= CvBridge()
        self.depth_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.depth_callback)
        rospy.loginfo('nodos listos')

    def	depth_callback(self, ros_image):
        try:
            depth_image	= self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError, e:
            print e

        depth_array = np.array(depth_image,	dtype=np.float32)
        
        hsv  = cv2.cvtColor(depth_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=4)
        mask = cv2.dilate(mask, None, iterations=4)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                cv2.circle(depth_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(depth_image, center, 5, (0, 0, 255), -1)

        cv2.imshow(self.node_name, depth_image)
        cv2.imshow('Mascara', hsv)

        cc = 'x'
        self.keystroke = cv2.waitKey(5)
        if 32 <= self.keystroke	and	self.keystroke < 128:
            cc = chr(self.keystroke).lower()
        if cc == 'q':
            rospy.signal_shutdown('Presiona q para salir.')

    def	cleanup(self):
        rospy.loginfo('Cerrando Nodos')
        cv2.destroyAllWindows()

def	main(args):
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Cerrando Nodos')
        cv2.DestroyAllWindows()

if __name__	== '__main__':
    main(sys.argv)