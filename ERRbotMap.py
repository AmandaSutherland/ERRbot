#!/usr/bin/env python

import rospy
import cv
import cv2
import numpy as np
import math 
<<<<<<< HEAD
import thread
=======

>>>>>>> 85e82fb5da3de80043b3f2ad07651df17d9a8ec5
import ERRbotVision
import ERRbotMap
import ERRbotPath

from geometry_msgs.msg import Twist, Vector3
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from std_msg.msg import String

class ERRbotMap:

	def __init__(self,descriptor):

		pub = rospy.Publisher('Map', Int16MultiArray, queue_size = 10)
		rospy.init_node('ERRbotMap', anonymous = True)

	def Map(new_objects):
		"""
		Output is some list of particles or map for the path planning to take in and make its decisions based on.
		Does not actually matter what this is, but the path planning depends on what it is."""

		return 1

		while not rospy.is_shutdown():
			str = ''
			rospy.loginfo(str)
			pub.publish(str)
			r.sleep()

		#return (distance,is_object,what_object)

if __name == '__main__':
	try:
		ERRbotMap.Map()
	except rospy.ROSInterruptException: pass
