#!/usr/bin/env python

import rospy
import cv
import cv2
import numpy as np
import math 
import thread

import ERRbotVision
import ERRbotMap
import ERRbotPath

from geometry_msgs.msg import Twist, Vector3
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3

class ERRbotMain:

	def __init__(self,descriptor):
		rospy.Subscriber("scan", LaserScan, self.scan_received, queue_size=1)
		self.pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.camera_listener = rospy.Subscriber("camera/image_raw", Image, self.capture)
		self.bridge = CvBridge()
		self.new_img = None

	def capture(self,msg):
		# IMAGE FROM NEATO 
		#useful link for image types http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		3self.new_img = cv_image
		if self.new_img.shape[0] == 480:
			self.image_stream = True
		else:
			self.image_stream = False

	def arbiter(self)

if __name__ == '__main__':
	rospy.init_node('capture', anonymous=True)
	n = ERRbotMain
	while not(rospy.is_shutdown()):
		
		if n.image_stream == False:
			print 'nope'
		else:
			self.arbiter