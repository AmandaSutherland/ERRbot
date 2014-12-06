#!/usr/bin/env python

import rospy
import cv
import cv2
import numpy as np
import math 

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

	#def __init__(self,descriptor):

	def Map(new_objects):
		"""
		Output is a map of the area, updated to show the objects as they are found. Map takes in the 
		Occupancy Grid from hector_slam and the object position from ERRbotVision. Map is for visualization only."""

		return 1

		while not rospy.is_shutdown():
			# int = map
			# rospy.loginfo(int)
			# pub.publish(int)
			r.sleep()

		#return (distance,is_object,what_object)

if __name == '__main__':
	try:
		ERRbotMap.Map()
	except rospy.ROSInterruptException: pass