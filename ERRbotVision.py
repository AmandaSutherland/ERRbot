#!/usr/bin/env python

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

	lowerred = np.array([110,100,100])
	upperred = np.array([130,255,255])
	redmask = cv2.inRange(hsv, lowerred, upperred)

	loweryellow = np.array([110,100,100])
	upperyellow = np.array([130,255,255])
	yellowmask = cv2.inRange(hsv, loweryellow, upperyellow)		

	houghCircles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
	houghCircles = np.uint16(np.around(houghCircles))

	what_object = []
	distance = []
	is_object = []

	for i in houghCircles[0,:]:

		# draw the outer circle
    	cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    	# draw the center of the circle
    	cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

		#if circle is in blue mask
			#return blue
			#add color, size, location and probablility it is an object to arrays
		#if circle is in red mask
			#return red
			#add color, size, location and probablility it is an object to arrays
		#if circle is in yellow mask
			#return yellow
			#add color, size, location and probablility it is an object to arrays

	return (distance,is_object,what_object)