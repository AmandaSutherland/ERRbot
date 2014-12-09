#!/usr/bin/env python
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
###a 'dumb' navigation system

import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import cv
import cv2
import numpy as np
import math 

import ERRbotVision
#import ERRbotMap
import ERRbotPath

first_sight_wall = False
flag_things_in_front_3 = False
flag_things_in_front_5 = False
flag_things_in_front_7 = False
flag_things_in_front_10 = False
diff = 0 
parallel = False
no_valid_angles = False
diffs = []
avoid_these_angles = []

class ERRbotPath:

	#def __init__(self,descriptor):
		#pub = rospy.Publisher('Path', String, queue_size = 10)
		#rospy.init_node('ERRbotPath', anonymous = True)

	def scan_received(msg):
		""" Processes data from the laser scanner, msg is of type sensor_msgs/LaserScan"""
	    """sets diff to -100 if broken """
	    global first_sight_wall
	    global flag_things_in_front_3
	    global flag_things_in_front_5
	    global flag_things_in_front_7
	    global flag_things_in_front_10
	    global diff
	    global parallel
	    global no_valid_angles
	    global diffs
	    global angle_to_avoid
	    global avoid_these_angles

	    less_than_one_ranges = []
	    for i in range(360):
	        if msg.ranges[i] > 0 and msg.ranges[i] < 1:
	            less_than_one_ranges.append((i,msg.ranges[i]))

	    valid_theta1 = []
	    valid_theta2 = []
	    equidistant_angles = []

	    for i in range(45):
	        if msg.ranges[90-i] != 0 and msg.ranges[90+i] != 0:
	            l1 = msg.ranges[90-i]
	            l2 = msg.ranges[90+i]

	            valid_theta1.append(math.atan2(l2,l1))
	            valid_theta2.append(math.atan2(l1,l2))
	            if abs(l2-l1)< .1:
	                equidistant_angles.append(90-i)

	            #print "value of theta1 is %f" % math.atan2(l2,l1)
	            #print "value of l1 is %f" %l1
	            #print "value of l2 is %f" %l2
	            #print "value of theta2 is %f" % math.atan2(l1,l2)
	    diff = sum(valid_theta2)-sum(valid_theta1)
	    diffs.append(diff)
	    #print equidistant_angles

	    if len(equidistant_angles) > 20:
	        parallel = True
	    else:
	        parallel = False

	    if len(valid_theta1) == 0:
	        no_valid_angles = True
	    else:
	        no_valid_angles = False

	    #print valid_theta2
	    #print valid_theta1

	    print "value of diff is %f" % diff

	    if len(less_than_one_ranges) > 10:
	        first_sight_wall = True

	    angles_in_front = []
	    avoid_these_angles = []

	    for i in range(45):
	        if msg.ranges[314 + i] != 0:
	            if msg.ranges[314 + i] < 1:
	                avoid_these_angles.append(314+i)

	    for i in range(45):
	        if msg.ranges[i] != 0:
	            if msg.ranges[i] < 1:
	                avoid_these_angles.append(i)

	    if len(avoid_these_angles) > 0:
	        angle_to_avoid = avoid_these_angles[-1]

	    print avoid_these_angles

	    for i in range(10):
	        if msg.ranges[0+i] != 0 and msg.ranges[0+i] < 1:
	            angles_in_front.append(i)
	        if msg.ranges[359-i] != 0 and msg.ranges[359-i] < 1:
	            angles_in_front.append(360-i)

	    if len(angles_in_front) > 3:
	        flag_things_in_front_3 = True
	        print 'Yes, there are more than 3 angles in front'
	    else:
	        flag_things_in_front_3 = False

	    if len(angles_in_front) > 5:
	        flag_things_in_front_5 = True
	        print 'Yes, there are more than 5 angles in front'
	    else:
	        flag_things_in_front_5 = False

	    if len(angles_in_front) > 7:
	        flag_things_in_front_7 = True
	        print 'Yes, there are more than 7 angles in front'
	    else:
	        flag_things_in_front_7 = False

	    if len(angles_in_front) > 10:
	        flag_things_in_front_10 = True
	        print 'Yes, there are more than 10 angles in front'
	    else:
	        flag_things_in_front_10 = False

	stopped = False

	def find_wall(pub):
	    r = rospy.Rate(10)
	    while not rospy.is_shutdown():
	        velocity_msg = Twist(Vector3(0.05,0.0,0.0),Vector3(0.0,0.0,0.0))
	        pub.publish(velocity_msg)
	        if first_sight_wall == True and no_valid_angles == False:
	            return 'wall_follow'
	        r.sleep()

	def wall_follow(pub):
	    global stopped
	    r = rospy.Rate(10) # 10hz
	    while not rospy.is_shutdown():
	        if first_sight_wall:
	            if no_valid_angles == True:
	                print 'no valid angles'
	                velocity_msg = Twist(Vector3(0.05,0.0,0.0),Vector3(0.0,0.0,0.3))
	            else:
	                if len(diffs) > 1 and diffs[-2] != 0:
	                    print 'pd'
	                    velocity_msg = Twist(Vector3(0.05,0.0,0.0),Vector3(0.0,0.0,0.02*diff-.01*diffs[-1]/diffs[-2]))
	                else:
	                    print 'p'
	                    velocity_msg = Twist(Vector3(0.05,0.0,0.0),Vector3(0.0,0.0,0.02*diff))

	        pub.publish(velocity_msg)
	        if flag_things_in_front_3 and parallel == True:
	            return 'obstacle_avoid'
	        r.sleep()
	 
	def obstacle_avoid(pub):
	    flag_things_in_front_5 = False
	    r = rospy.Rate(10)
	    while not rospy.is_shutdown():
	        if len(avoid_these_angles) > 1:
	            velocity_msg = Twist(Vector3(0.05,0.0,0.0),Vector3(0.0,0.0,-.5*(315-angle_to_avoid)/315))
	        else:
	            velocity_msg = Twist(Vector3(-0.05,0.0,0.0),Vector3(0.0,0.0,0))

	        pub.publish(velocity_msg)
	        if parallel == False and flag_things_in_front_3 != True:
	            print 'not parallel'
	            return 'wall_follow'
	    r.sleep()

		while not rospy.is_shutdown():
			#int = linear, anglular
			#rospy.loginfo(int)
			#pub.publish(int)
			#r.sleep()
	    	#ERRbotPath.Path()
	        rospy.init_node('my_fsm', anonymous=True)
	        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	        sub = rospy.Subscriber('scan', LaserScan, scan_received)
	        state = "find_wall"

	        while not rospy.is_shutdown():
	            if state == 'find_wall':
	                print 'I am now finding a wall'
	                state = find_wall(pub)
	            if state == 'wall_follow':
	                print 'I am now wall following'
	                state = wall_follow(pub)
	            if state == 'obstacle_avoid':
	                print 'I am now obstacle avoiding'
	                state = obstacle_avoid(pub)
	    except rospy.ROSInterruptException: pass

