#!/usr/bin/env python

import rospy
import cv
import cv2
import math

from geometry_msgs.msg import Twist, Vector3
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from std_msgs.msg import String,Int64

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

markerArray = MarkerArray()


class Main:
	def __init__(self):
		rospy.Subscriber("scan", LaserScan, queue_size=1)
		self.pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.red_sub = rospy.Subscriber('VisionRed', Twist, self.Red, queue_size=1)
		self.green_sub = rospy.Subscriber('VisionGreen', Twist, self.Green, queue_size=1)
		self.blue_sub = rospy.Subscriber('VisionBlue', Twist, self.Blue, queue_size=1)
		self.yellow_sub = rospy.Subscriber('VisionYellow', Twist, self.Yellow, queue_size=1)
		self.markerArray = MarkerArray()
		#xs_red, ys_red = subVRed;
 		#xs_green, ys_green = subVGreen;
		#xs_blue, ys_blue = subVBlue;
		#xs_yellow, ys_yellow = subVYellow;
		#subVRed, subVGreen, subVBlue, subVYellow = ball_types



	def Red(self,data):
		#if ball_types = subVRed:
		marker = Marker()
		marker.header.frame_id = 0
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		#parameters for the marker 
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = data.linear.z
		marker.pose.position.y = data.angular.z
		marker.pose.position.z = 0
		marker.lifetime = rospy.Duration()
		marker.color.a = 1.0

		self.markerArray.markers.append(marker)


	def Green(self,data):
		#if ball_types = subVGreen:
		marker = Marker()
		marker.header.frame_id = 1
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		#parameters for the marker 
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = data.linear.z
		marker.pose.position.y = data.angular.z
		marker.pose.position.z = 0
		marker.lifetime = rospy.Duration()
		marker.color.r = 1.0

		self.markerArray.markers.append(marker)

	def Blue(self,data):
		#if ball_types = subVBlue:
		marker = Marker()
		marker.header.frame_id = 2
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		#parameters for the marker 
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = data.linear.z
		marker.pose.position.y = data.angular.z
		marker.pose.position.z = 0
		marker.lifetime = rospy.Duration()
		marker.color.a = 1.0

		self.markerArray.markers.append(marker)

	def Yellow(self,data):
		#if ball_types = subVYellow:
		marker = Marker()
		marker.header.frame_id = 3
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		#parameters for the marker 
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = data.linear.z
		marker.pose.position.y = data.angular.z
		marker.pose.position.z = 0
		marker.lifetime = rospy.Duration()
		marker.color.a = 1.0

		self.markerArray.markers.append(marker)

if __name__ == '__main__':
	rospy.init_node('visuals', anonymous=True)
	n = Main()
	pub = rospy.Publisher('/visualization_marker_array', MarkerArray)
	while not(rospy.is_shutdown()):
		try:
			pub.publish(n.markerArray)
			#print marker 
		except rospy.ROSInterruptException:
			pass

	
	




