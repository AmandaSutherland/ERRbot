#!/usr/bin/env python
#arbiter

import rospy
import cv
import cv2
import numpy as np
import math 

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan

#from std_msg.msg import String

class ERRbotMain:
	global callback
	def callback(data):
		#testing arbiter
		pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		pub.publish(data)

	def __init__(self):

		#rospy.Subscriber("Vision", 	int32, queue_size=1)
		#rospy.Subscriber("Map", int32, queue_size=1)
		self.sub = rospy.Subscriber("Path", Twist, callback) #calls callback function

		try:
			#for image capture 
			self.camera_listener = rospy.Subscriber("camera/image_raw", Image, self.capture)
			self.bridge = CvBridge()
			#make image something useful
		except AttributeError:
			print "ERROR!"
			pass	

	# def capture(self,msg):
	# 	# IMAGE FROM NEATO 
	# 	#useful link for image types http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
	# 	cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
	# 	self.new_img = cv_image
	# 	if self.new_img.shape[0] == 480:
	# 		self.image_stream = True
	# 	else:
	# 		self.image_stream = False

	def arbiter(self,location=None,is_object=None,what_object=None,next_move=None):
		#linear,angular=next_move
		#remove this when is_object is created
		#pub.publish(Twist(linear=Vector3(x=linear),angular=Vector3(z=angular)))
		self.pub.publish(self.dira)

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
		print 'hi'

if __name__ == '__main__':
	rospy.init_node('main', anonymous=True)
	n = ERRbotMain()
	#n.capture = False
	while not(rospy.is_shutdown()):
		# if n.capture == False:
		# 	print 'nope. no image.'
		# else:
		# 	try:
		# 		#may need to make the arbiter an additional thread. working to resolve this.
		# 		location,is_object,what_object = thread.start_new_thread(ERRbotVision.Vison,(img))
		# 		mapping = thread.start_new_thread(ERRbotMap.Map,(n.new_object))
		# 		next_move = thread.start_new_thread(ERRbotPath.Path,(mapping))
		# 	except:
		# 		print 'failed threading'
		location = 0
		is_object = 0
		what_object = 0
		next_move = 0
		pass
		#n.arbiter(location,is_object,what_object,next_move)
		# cv2.namedWindow("Image")
		# cv2.imshow("Image",frame)