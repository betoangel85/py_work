#!/usr/bin/env python

import roslib; roslib.load_manifest('py_work')

import rospy
import sys
import cv2
from sensor_msgs.msg import	Image, CameraInfo
from cv_bridge import CvBridge,	CvBridgeError
import numpy as	np

class cvBridgeDemo():

    def	__init__(self):
        self.node_name = 'cv_bridge'
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge	= CvBridge()

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw',	Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.depth_callback)
        rospy.loginfo('nodos listos')

    def	image_callback(self, ros_image):
        try:
            frame =	self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError, e:
            print e

        frame =	np.array(frame,	dtype=np.uint8)
        (display_image,gray) =	self.process_image(frame)

        cv2.imshow(self.node_name, display_image)

        cc = 'x'
        self.keystroke = cv2.waitKey(5)
        if 32 <= self.keystroke	and	self.keystroke < 128:
            cc = chr(self.keystroke).lower()
        if cc == 'q':
            rospy.signal_shutdown('Presiona q para salir.')

    def	depth_callback(self, ros_image):
        try:
            depth_image	= self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        except CvBridgeError, e:
            print e

        depth_array	= np.array(depth_image,	dtype=np.float32)
        cv2.normalize(depth_array, depth_array,	0, 1, cv2.NORM_MINMAX)
        depth_display_image	= self.process_depth_image(depth_array)
        cv2.imshow('Imagen RAW', depth_display_image)

    def	process_image(self, frame):
        grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        grey = cv2.blur(grey, (7, 7))
        edges =	cv2.Canny(grey,	15.0, 30.0)
        return edges, grey

    def	process_depth_image(self, frame):
        return frame

    def	cleanup(self):
        print('Cerrando Nodo')
        cv2.destroyAllWindows()	  

def	main(args):		  
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print('Cerrando Nodo')
        cv2.DestroyAllWindows()

if __name__	== '__main__':
    main(sys.argv)