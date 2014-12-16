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
from std_msgs.msg import String, Int64

class ERRbotVision:

    def __init__(self):
        self.camera_listener = rospy.Subscriber("camera/image_raw", Image, self.capture)
        self.bridge = CvBridge()
        self.new_img = None
        self.cimg = None
        self.edges = None

        self.pub = rospy.Publisher('Vision', Int64, queue_size = 10)
        rospy.init_node('ERRbotVision', anonymous = True)
        self.r = rospy.Rate(10)

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

    def Vision(self,img):
        '''
        outputs are distance,is_object,what_object
        distance = distance from the robot of a potenial object
        is_object = probability or "goodness" of the object
        what_object = number/color of the object'''

        print 'Vision is working'

        img = cv2.medianBlur(img,5)
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.cimg = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        print 'messing with image'

        lowerblue = np.array([10,110,110])
        upperblue = np.array([130,255,255])
        bluemask = cv2.inRange(hsv, lowerblue, upperblue)
        print 'bluemask'

        lowerred = np.array([0,100,100])
        upperred = np.array([20,255,255])
        redmask = cv2.inRange(hsv, lowerred, upperred)
        print 'redmask'

        loweryellow = np.array([20, 100, 100])
        upperyellow = np.array([30,255,255])
        yellowmask = cv2.inRange(hsv, loweryellow, upperyellow)
        print 'yellowmask'     

        lowergreen = np.array([110,100,100])
        uppergreen = np.array([130,255,255])
        greenmask = cv2.inRange(hsv, lowergreen, uppergreen)    
        print 'greenmask'

        self.edges = cv2.Canny(img, 100, 150)
        houghCircles = cv2.HoughCircles(self.edges,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=10,param2=22,minRadius=10,maxRadius=50)
        houghCircles = np.uint16(np.around(houghCircles))
        print 'found circles'

        what_object = []
        distance = []
        angle = []
        is_object = []

        for i in houghCircles[0,:]:
            print 'iterating circles'
            print (i)
            # draw the outer circle
            cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,0,0),2)
            # draw the center of the circle
            cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,0),3)

            if bluemask[i[1], i[0]]  > 100:
                # draw the outer circle
                cv2.circle(self.cimg,(i[0],i[1]),i[2],(255,0,0),2)
                # draw the center of the circle
                cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                blueangle = (i[0]-280)/30#pixel# - middle pixel / angles Vector3(i[0], i[1],i[2])
                bluedistance = i[2]/30 #some constant to get distance to ball

                what_object.append(1)
                distance.append(bluedistance)
                angle.append(blueangle)
                is_object.append(1)

            if redmask[i[1], i[0]]  > 100:
                # draw the outer circle
                cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,0,255),2)
                # draw the center of the circle
                cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                redangle = Vector3(i[0], i[1],i[2])
                reddistance = i[2]/30 #some constant to get distance to ball

                what_object.append(2)
                distance.append(reddistance)
                angle.append(redangle)
                is_object.append(1)

            if yellowmask[i[1], i[0]]  > 100:
                # draw the outer circle
                cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,255,255),2)
                # draw the center of the circle
                cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                yellowangle = Vector3(i[0], i[1],i[2])
                yellowdistance = i[2]/30 #some constant to get distance to ball

                what_object.append(3)
                distance.append(yellowdistance)
                angle.append(yellowangle)
                is_object.append(1)

            if greenmask[i[1], i[0]]  > 100:
                # draw the outer circle
                cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                greenangle = Vector3(i[0], i[1],i[2])
                greendistance = i[2]/30 #some constant to get distance to ball

                what_object.append(4)
                distance.append(greendistance)
                angle.append(greenangle)
                is_object.append(1)

            #if circle is in blue mask
                #return blue
                #add color, size, angle and probablility it is an object to arrays
            #if circle is in red mask
                #return red
                #add color, size, angle and probablility it is an object to arrays
            #if circle is in yellow mask
                #return yellow
                #add color, size, angle and probablility it is an object to arrays

        print (what_object)
        print (distance)
        print (angle)

        # while not rospy.is_shutdown():
        #     data = angle, distance,is_object,what_object
        #     #rospy.loginfo(int)
        #     self.pub.publish(data)
        #     self.r.sleep()

        #return (distance,is_object,what_object)

if __name__ == '__main__':
    try:
        #rospy.init_node('capture', anonymous=True)
        n = ERRbotVision()
        #n.capture = False
        cv2.namedWindow('NeatoImage')
        cv2.namedWindow('CirclesImage')
        #cv2.namedWindow('EdgesImage')
        cv2.imshow("NeatoImage",n.new_img)
        while not(rospy.is_shutdown()):
            if n.capture == False:
                print 'nope. no image.'
            else:
                print 'got an image'
                n.Vision(n.new_img)
                frame = np.array(cv2.resize(n.new_img,(n.new_img.shape[1]/2,n.new_img.shape[0]/2)))
                cv2.imshow("NeatoImage",frame)
                cv2.imshow("CirclesImage",n.cimg)
                #cv2.imshow("EdgesImage",n.edges)
                cv2.waitKey(50)
            print 'move on'
    except rospy.ROSInterruptException: 
        pass