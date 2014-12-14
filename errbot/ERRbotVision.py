#!/usr/bin/env python

#to run without control of robot
#roslaunch gscam raspi_nodelet.launch host:=192.168.17.___
#to view image
#rosrun image_view image_view image:=/camera/image_raw


import rospy
import cv
import cv2
import numpy as np
import math 
import thread

from geometry_msgs.msg import Twist, Vector3
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from std_msgs.msg import String

class ERRbotVision:

    def __init__(self,descriptor):
        self.camera_listener = rospy.Subscriber("camera/image_raw", Image, self.capture)
        self.bridge = CvBridge()
        self.new_img = Nonec

        pub = rospy.Publisher('Vision', Int16MultiArray, queue_size = 10)
        rospy.init_node('ERRbotVision', anonymous = True)
        r = rospy.Rate(10)

    def capture(self,msg):
        # IMAGE FROM NEATO 
        #useful link for image types http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.new_img = cv_image
        if self.new_img.shape[0] == 480:
            self.image_stream = True
        else:
            self.image_stream = False

    def Vision(img):
        '''
        outputs are distance,is_object,what_object
        distance = distance from the robot of a potenial object
        is_object = probability or "goodness" of the object
        what_object = number/color of the object'''

        img = cv2.medianBlur(img,5)
        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lowerblue = np.array([110,100,100])
        upperblue = np.array([130,255,255])
        bluemask = cv2.inRange(hsv, lowerblue, upperblue)

        lowerred = np.array(0,100,100)
        upperred = np.array([20,255,255])
        redmask = cv2.inRange(hsv, lowerred, upperred)

        loweryellow = np.array([20, 100, 100])
        upperyellow = np.array([30,255,255])
        yellowmask = cv2.inRange(hsv, loweryellow, upperyellow)     

        lowergreen = np.array([110,100,100])
        uppergreen = np.array([130,255,255])
        greenmask = cv2.inRange(hsv, lowergreen, uppergreen)    

        houghCircles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
        houghCircles = np.uint16(np.around(houghCircles))

        what_object = []
        distance = []
        location = []
        is_object = []

        for i in houghCircles[0,:]:

            if bluemask[c[1], c[0]]  > 100:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                bluelocation = (i[0]-280)/30#pixel# - middle pixel / angles Vector3(i[0], i[1],i[2])
                bluedistance = i[2]/30 #some constant to get distance to ball

                what_object.append(1)
                distance.append(bluedistance)
                location.append(bluelocation)
                is_object.append(1)

            if redmask[c[1], c[0]]  > 100:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                redlocation = Vector3(i[0], i[1],i[2])
                reddistance = i[2]/30 #some constant to get distance to ball

                what_object.append(2)
                distance.append(reddistance)
                distance.append(redlocation)
                is_object.append(1)

            if yellowmask[c[1], c[0]]  > 100:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                yellowlocation = Vector3(i[0], i[1],i[2])
                yellowdistance = i[2]/30 #some constant to get distance to ball

                what_object.append(3)
                distance.append(yellowdistance)
                distance.append(yellowlocation)
                is_object.append(1)

            if greenmask[c[1], c[0]]  > 100:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                greenlocation = Vector3(i[0], i[1],i[2])
                greendistance = i[2]/30 #some constant to get distance to ball

                what_object.append(4)
                distance.append(greendistance)
                distance.append(greenlocation)
                is_object.append(1)

            #if circle is in blue mask
                #return blue
                #add color, size, location and probablility it is an object to arrays
            #if circle is in red mask
                #return red
                #add color, size, location and probablility it is an object to arrays
            #if circle is in yellow mask
                #return yellow
                #add color, size, location and probablility it is an object to arrays

        while not rospy.is_shutdown():
            int = location, distance,is_object,what_object
            rospy.loginfo(int)
            pub.publish(int)
            r.sleep()

        #return (distance,is_object,what_object)

if __name__ == '__main__':
    rospy.init_node('capture', anonymous=True)
    n = ERRbotVision
    n.capture = False
    if n.capture == False:
        print 'nope. no image.'
    else:
        ERRbotVision.Vision()
    #except rospy.ROSInterruptException: 
    #    pass