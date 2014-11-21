#!/usr/bin/env python

def Vision(img):
	'''
	outputs are location,is_object,what_object
	location = location in the map of a potenial object
	is_object = probability or "goodness" of the object
	what_object = number/color of the object'''

	img = cv2.medianBlur(img,5)
	cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


	lowerblue = np.array([110,100,100])
	upperblue = np.array([130,255,255])
	bluemask = cv2.inRange(hsv, lowerblue, upperblue)		

	houghCircles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
	houghCircles = np.uint16(np.around(houghCircles))
	for i in houghCircles[0,:]:
    	# draw the outer circle
    	cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    	# draw the center of the circle
    	cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

	return (1,2,3)