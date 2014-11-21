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

		try:
			#for image capture 
			self.camera_listener = rospy.Subscriber("camera/image_raw", Image, self.capture)
			self.bridge = CvBridge()
			#make image something useful
		except AttributeError:
			print "ERROR!"
			pass	

	def capture(self,msg):
		# IMAGE FROM NEATO 
		#useful link for image types http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		self.new_img = cv_image
		if self.new_img.shape[0] == 480:
			self.image_stream = True
		else:
			self.image_stream = False

	def arbiter(self,location,is_object,what_object,next_move):
		linear,angular=next_move
		if is_object > .8:
			#its an object!! Map it and move on!
			#add to map if it is not already there (push location to where we are making the map)
			self.new_object = location,what_object
			#follow path (next_move)
			pub.publish(Twist(linear=Vector3(x=linear),angular=Vector3(z=angular)))
		elif is_object > .5:
			#Is it an object? Check it and ignore the path
			#turn towards potential object
			#ignore path
			print 'hello'
		else:
			print 'else'
			#Theres probably no object. Move on.
			#follow path (next_move)
			#pub.publish(Twist(linear=Vector3(x=linear),angular=Vector3(z=angular)))

if __name__ == '__main__':
	rospy.init_node('capture', anonymous=True)
	n = ERRbotMain
	n.capture = False
	while not(rospy.is_shutdown()):
		if n.capture == False:
			print 'nope. no image.'
		else:
			try:
				location,is_object,what_object = thread.start_new_thread(ERRbotVision.Vison,(img))
				mapping = thread.start_new_thread(ERRbotMap.Map,(n.new_object))
				next_move = thread.start_new_thread(ERRbotPath.Path,(mapping))
			#else:
				#print 'failed threading'
			n.arbiter(location,is_object,what_object,next_move)
		cv2.namedWindow("Image")
		cv2.imshow("Image",frame)