#!/usr/bin/env python

import rospy
import cv
import cv2
import numpy as np
import math 
import random

from geometry_msgs.msg import Twist, Vector3
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from std_msgs.msg import String, Int64, Header
import tf


class ERRbotMain:

	def __init__(self):
		self.listener = tf.TransformListener()
		rospy.Subscriber("scan", LaserScan, queue_size=1)
		#self.vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.chatter_pub=rospy.Publisher('chatter',String,queue_size=10)
		self.move_simple_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
		self.pose_pub = rospy.Publisher("pose",Pose,queue_size=1)#self.vision_pub=rospy.Publisher("vision_pub",Int64,queue_size=10)

		#self.vision = rospy.Subscriber("Vision", Int64, self.vision_callback, queue_size=1)
		self.map = rospy.Subscriber("Map", Int64, queue_size=1)
		#self.path = rospy.Subscriber("Path", Int64, queue_size=1)
		self.wall_follow = rospy.Subscriber("Wall_Follow", Twist, self.wall_follow_callback,queue_size=1)
		
		#subscribe to balls
		#Vector3, Vector3
		#Vector3[1,2,3] represent the angle, Vector3[3] represents the distance

		self.vision_red = rospy.Subscriber("Red", Twist, queue_size=1)
		self.vision_yellow = rospy.Subscriber("Yellow", Twist, queue_size=1)
		self.vision_green = rospy.Subscriber("Green", Twist, queue_size=1)
		self.vision_blue = rospy.Subscriber("Blue", Twist, queue_size=1)

		self.vision_flag = False
		self.wall_follow_flag = False

	def tf_listen(self):
		try:
			(pose,quaternion) = self.listener.lookupTransform('/base_link','/map',rospy.Time())
			self.pose_pub(pose)
			self.thing()
		except:
			pass
		
		#print pose, quaternion


	def go_to_goal(self):
		position = Point()
		orientation = Quaternion()

		position.x = random.randint(0,9)
		position.y = random.randint(0,9)
		position.z = 0
		orientation.x = 0
		orientation.y = 0
		orientation.w = 1
		orientation.z = 0
		send_goal_topic(position,orientation)

	def send_goal_topic(self, pos, orientation_quaternion, frame_id='map'):
		pos.z = 0
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = frame_id
		base_goal = PoseStamped(header,Pose(position=pos, orientation=orientation_quaternion))
		self.running = True
		self.move_simple_base_pub.publish(base_goal)

	def vision_callback(self,data):
		self.vision_data = data
		self.vision_flag = True
		self.vision_pub(self.vision_data)


	def wall_follow_callback(self,data):
		self.wall_folow_data = data
		self.wall_follow_flag = True

	def arbiter(self):
		if self.wall_follow_flag == True: #when data arrives
			pass
			#self.vel_pub.publish(self.wall_follow_data)

		#pub.publishrostopic pub /move_base_simple/goal geetry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'


		#something that talks to dead reckoning

	def thing(self):
		self.chatter_pub.publish('hello')


		# if is_object > .8:
		# 	#its an object!! Map it and move on!
		# 	#add to map if it is not already there (push location to where we are making the map)
		# 	self.new_object = location,what_object
		# 	#follow path (next_move)
		# 	pub.publish(Twist(linear=Vector3(x=linear),angular=Vector3(z=angular)))
		# elif is_object > .5:
		# 	#Is it an object? Check it and ignore the path
		# 	#turn towards potential object
		# 	#ignore path
		# else:
		# 	print 'else'
		# 	#Theres probably no object. Move on.
		# 	#follow path (next_move)
			#pub.publish(Twist(linear=Vector3(x=linear),angular=Vector3(z=angular)))

if __name__ == '__main__':
	rospy.init_node('capture', anonymous=True)
	n = ERRbotMain()
	while not(rospy.is_shutdown()):


		n.arbiter()
		#n.thing()
		#n.go_to_goal()
		n.tf_listen()
