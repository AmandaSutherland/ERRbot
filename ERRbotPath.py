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
from std_msg.msg import String

class ERRbotPath:

	def __init__(self,descriptor):

		pub = rospy.Publisher('Path', Int16MultiArray, queue_size = 10)
		rospy.init_node('ERRbotPath', anonymous = True)

	def Path:

		#output is new_move which should be composed of a suggested linear,angular pair of speeds
		
		return 1

		while not rospy.is_shutdown():
			int = linear, anglular
			rospy.loginfo(int)
			pub.publish(int)
			r.sleep()

		#return (distance,is_object,what_object)

if __name == '__main__':
	try:
		ERRbotPath.Path()
	except rospy.ROSInterruptException: pass