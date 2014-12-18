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
        self.what_object = []
        self.distance = []
        self.angle = []

        rospy.init_node('ERRbotVision', anonymous = True)
        self.pub_red = rospy.Publisher('VisionRed', Twist, queue_size = 10)
        self.pub_green = rospy.Publisher('VisionGreen', Twist, queue_size = 10)
        self.pub_yellow = rospy.Publisher('VisionYellow', Twist, queue_size = 10)
        self.pub_blue = rospy.Publisher('VisionBlue', Twist, queue_size = 10)

        
        self.r = rospy.Rate(10)
        # cv2.namedWindow('bluemask')
        # cv2.createTrackbar('H Lower','bluemask', 65,255,self.set_hue_lower)
        # self.hue_lower = 65
        # cv2.createTrackbar('H Upper','bluemask', 110,255,self.set_hue_upper)
        # self.hue_upper = 110
        # cv2.createTrackbar('S Lower','bluemask', 0,255,self.set_s_lower)
        # self.s_lower = 0
        # cv2.createTrackbar('S Upper','bluemask', 255,255,self.set_s_upper)
        # self.s_upper = 255
        # cv2.createTrackbar('V Lower','bluemask', 0,255,self.set_v_lower)
        # self.v_lower = 0
        # cv2.createTrackbar('V Upper','bluemask', 255,255,self.set_v_upper)
        # self.v_upper = 255

        try:
            #for image capture 
            self.camera_listener = rospy.Subscriber("camera/image_raw", Image, self.capture)
            self.bridge = CvBridge()
            #make image something useful
        except AttributeError:
            print "ERROR!"
            pass    

    # def set_hue_lower(self,value):
    #     self.hue_lower = value

    # def set_hue_upper(self,value):
    #     self.hue_upper = value

    # def set_s_lower(self,value):
    #     self.s_lower = value

    # def set_s_upper(self,value):
    #     self.s_upper = value

    # def set_v_lower(self,value):
    #     self.v_lower = value

    # def set_v_upper(self,value):
    #     self.v_upper = value

    def capture(self,msg):
        # IMAGE FROM NEATO 
        #useful link for image types http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.new_img = cv_image
        if self.new_img.shape[0] == 480:
            self.image_stream = True
        else:
            self.image_stream = False

    def trig (self, distance, angle):
        x = math.cos(angle)*distance
        y = math.sin(angle)*distance
        return (x,y)

    def Vision(self,img):
        '''
        outputs are distance,is_object,what_object
        distance = distance from the robot of a potenial object
        is_object = probability or "goodness" of the object
        what_object = number/color of the object'''

        #print 'Vision is working'

        img = cv2.medianBlur(img,5)
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.cimg = cv2.cvtColor(grey,cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #print 'messing with image'

        #lowerred = np.array([self.hue_lower,self.s_lower,self.v_lower])
        #upperred = np.array([self.hue_upper,self.s_upper,self.v_upper])
        lowerred = np.array([0,0,28])
        upperred = np.array([10,255,255])
        redmask = cv2.inRange(hsv, lowerred, upperred)
        #print 'redmask'

        #loweryellow = np.array([self.hue_lower,self.s_lower,self.v_lower])
        #upperyellow = np.array([self.hue_upper,self.s_upper,self.v_upper])
        loweryellow = np.array([25,230,30])
        upperyellow = np.array([30,255,255])
        yellowmask = cv2.inRange(hsv, loweryellow, upperyellow)
        #print 'yellowmask'     

        #lowergreen = np.array([self.hue_lower,self.s_lower,self.v_lower])
        #uppergreen = np.array([self.hue_upper,self.s_upper,self.v_upper])
        lowergreen = np.array([33,230,0])
        uppergreen = np.array([85,255,130])
        greenmask = cv2.inRange(hsv, lowergreen, uppergreen)    
        #print 'greenmask'
        
        #lowerblue = np.array([self.hue_lower,self.s_lower,self.v_lower])
        #upperblue = np.array([self.hue_upper,self.s_upper,self.v_upper])
        lowerblue = np.array([65,0,0])
        upperblue = np.array([110,255,255])
        bluemask = cv2.inRange(hsv, lowerblue, upperblue)
        #cv2.imshow('bluemask',bluemask)
        #print 'greenmask'
        #print (bluemask[1])
        self.what_object = []
        self.distance = []
        self.angle = []
        #is_object = []

        self.edges = cv2.Canny(img, 100, 150)
        houghCircles = cv2.HoughCircles(self.edges,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=10,param2=24,minRadius=10,maxRadius=50)
        if houghCircles != None:
            houghCircles = np.uint16(np.around(houghCircles))
        #print (houghCircles)            

            for i in houghCircles[0,:]:
                #print 'iterating circles'
                #print (i)
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
                    bluedistance = Vector3(0,0,70 - (5*i[2])) #some constant to get distance to ball
                    x,y=trig(bluedistance,blueangle)
                    #print 'blue'
                    #print (i[2])

                    self.pub_blue.publish(Twist(x,y))
                    #self.what_object.append(1)
                    #self.distance.append(bluedistance)
                    #self.angle.append(blueangle)
                    #is_object.append(1)

                if redmask[i[1], i[0]]  > 100:
                    # draw the outer circle
                    cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,0,255),2)
                    # draw the center of the circle
                    cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                    redangle = Vector3(i[0], i[1],i[2])
                    reddistance = Vector3(0,0,i[2]*.27) #some constant to get distance to ball
                    x,y=trig(reddistance,redangle)
                    #print 'red'
                    #print (i[2])

                    self.pub_red.publish(Twist(x,y))
                    #self.what_object.append(2)
                    #self.distance.append(reddistance)
                    #self.angle.append(redangle)
                    #is_object.append(1)

                if yellowmask[i[1], i[0]]  > 100:
                    # draw the outer circle
                    cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,255,255),2)
                    # draw the center of the circle
                    cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                    yellowangle = Vector3(i[0], i[1],i[2])
                    yellowdistance = Vector3(0,0,i[2]*.27) #some constant to get distance to ball
                    x,y=trig(yellowdistance,yellowangle)
                    #print 'yellow'
                    #print (i[2])

                    self.pub_yellow.publish(Twist(x,y))
                    #self.what_object.append(3)
                    #self.distance.append(yellowdistance)
                    #self.angle.append(yellowangle)
                    #is_object.append(1)

                if greenmask[i[1], i[0]]  > 100:
                    # draw the outer circle
                    cv2.circle(self.cimg,(i[0],i[1]),i[2],(0,255,0),2)
                    # draw the center of the circle
                    cv2.circle(self.cimg,(i[0],i[1]),2,(0,0,255),3)
                    greenangle = Vector3(i[0], i[1],i[2])
                    greendistance = Vector3(0,0,i[2]*.27) #some constant to get distance to ball
                    x,y=trig(greendistance,greenangle)
                    #print 'green'
                    #print (i[2])

                    self.pub_green.publish(Twist(x,y))
                    #self.what_object.append(4)
                    #self.distance.append(greendistance)
                    #self.angle.append(greenangle)
                    #self.is_object.append(1)

                #if circle is in blue mask
                    #return blue
                    #add color, size, angle and probablility it is an object to arrays
                #if circle is in red mask
                    #return red
                    #add color, size, angle and probablility it is an object to arrays
                #if circle is in yellow mask
                    #return yellow
                    #add color, size, angle and probablility it is an object to arrays

        #print 'object'
        #print (self.what_object)
        #print 'distance'
        #print (self.distance)
        #print 'angle'
        #print (self.angle)

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
        #cv2.imshow("NeatoImage",n.new_img)
        while not(rospy.is_shutdown()):
            if n.capture == False:
                print 'nope. no image.'
            else:
                #print 'got an image'
                n.Vision(n.new_img)
                frame = np.array(cv2.resize(n.new_img,(n.new_img.shape[1]/2,n.new_img.shape[0]/2)))
                cv2.imshow("NeatoImage",frame)
                cv2.imshow("CirclesImage",n.cimg)
                #cv2.imshow("EdgesImage",n.edges)
                #data = n.what_object, n.distance, n.angle
                #if n.what_object == "RED":
                #n.pub.publish(data)
                
                cv2.waitKey(50)
            #print 'move on'
    except rospy.ROSInterruptException: 
        pass