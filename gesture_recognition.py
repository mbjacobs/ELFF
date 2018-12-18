#!/usr/bin/python

import cv2
import numpy as np
import math

def countFingers(largestContour):  # -> finished bool, fingerCount: finger count
    #  convexity defect
    hull = cv2.convexHull(largestContour, returnPoints=False)
    if len(hull) > 3:
        defects = cv2.convexityDefects(largestContour, hull)
        if type(defects) != type(None):  # avoid crashing.   (BUG not found)

            fingerCount = 0
            for i in range(defects.shape[0]):  # calculate the angle
                s, e, f, d = defects[i][0]
                start = tuple(largestContour[s][0])
                end = tuple(largestContour[e][0])
                far = tuple(largestContour[f][0])
                a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # cosine theorem
                if angle <= math.pi / 2:  # angle less than 90 degree, treat as fingers
                    fingerCount += 1
            return True, fingerCount
    return False, 0

def removeBackground(frame):
    fg_mask = bgModel.apply(frame, learningRate = 0) # changed learningRate to 0
    kernel = np.ones((3,3),np.uint8)
    fg_mask = cv2.erode(fg_mask,kernel,iterations = 1)
    frame = cv2.bitwise_and(frame,frame,mask = fg_mask)
    return frame

def findLargestContour(contours):
    max_area=0
    largest_contour=-1
    for i in range(len(contours)):
        cont=contours[i]
        area=cv2.contourArea(cont)
        if(area>max_area):
            max_area=area
            largest_contour=i
    if(largest_contour==-1):
        return False, 0
    else:
        largestContour=contours[largest_contour]
        return True, largestContour


#main
camera = cv2.VideoCapture(0)
bCaptureDone = False
bBGCaptured = False
cap_region_x_begin = 0.5
cap_region_y_end = 0.8 
gaussian_ksize = 41 
gaussian_sigma = 0
thresholdLowValue = 60
thresholdMaxValue = 255
bgSubThreshold = 50


while camera.isOpened():
	camReadRV, frame = camera.read()

	#Captured image is filtered, rectangle is drawn...
	if camReadRV is False:
		print ("CAMERA DID NOT CAPTURE")
	else:
		#frame = cv2.bilateralFilter(frame,5,50,100) #smoothing filter (not sure the effect this will have)
		cv2.rectangle(frame,(int(cap_region_x_begin*frame.shape[1]),0),(frame.shape[1],int(cap_region_y_end*frame.shape[0])),(255,0,0),1)
		cv2.imshow("show image",frame)
	if bBGCaptured is True:
		bgRemovedFrame = removeBackground(frame)
		grayFrame = cv2.cvtColor(bgRemovedFrame, cv2.COLOR_BGR2GRAY)
		blurFrame = cv2.GaussianBlur(grayFrame, (gaussian_ksize, gaussian_ksize), gaussian_sigma)
		threshRV, threshFrame = cv2.threshold(blurFrame, thresholdLowValue, thresholdMaxValue, cv2.THRESH_BINARY)
		if threshRV is False:
			print ("cv2.threshold failure")
		_, contours, _ = cv2.findContours(threshFrame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		largestContourRV, largestContour = findLargestContour(contours)
		if largestContourRV is False:
			print("findLargestContour failure")
		calcFingersRV, fingerCount = countFingers(largestContour)
		if calcFingersRV is False:
			print("countFingers failure")
		print("Finger Count: {fingerCount}".format(fingerCount = fingerCount))

	k = cv2.waitKey(10)
	if k == 27:  # press ESC to exit
		break
	elif k == ord('b'):  # press 'b' to capture the background
		bgModel = cv2.createBackgroundSubtractorMOG2(0, bgSubThreshold)
		bBGCaptured = True
		print( '!!!Background Captured!!!')