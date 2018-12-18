#!/usr/bin/python
import cv2
import numpy as np
import copy
import math

camera = cv2.VideoCapture(0)
bg_captured = 0

while(1):
	camReadRV, frame = camera.read()
	oriframe = copy.deepcopy(frame)
	cv2.imshow ('Original Image', frame)

	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.imshow('color changed', frame)

	#frame = cv2.bilateralFilter(frame, 9, 150, 150)
	frame = cv2.GaussianBlur (frame, (5,5), 0)
	cv2.imshow ('Smoothing Filter', frame)

	framecopy = copy.deepcopy(frame)

	if (bg_captured == 1):
		framecopy = bg_model.apply(framecopy, learningRate = 0)
		cv2.imshow ('background model', framecopy)
		#cv2.threshold(frame, 0, 150, cv2.THRESH_BINARY) 
		#cv2.imshow ('thresholding', frame)
		
		_, contours, _ = cv2.findContours(framecopy, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


		length = len(contours)
		maxArea = -1
		if length > 0:
			for i in range(length):  # find the biggest contour (according to area)
				temp = contours[i]
				area = cv2.contourArea(temp)
				if area > maxArea:
					maxArea = area
					ci = i

			res = contours[ci]
			hull = cv2.convexHull(res)

			cv2.drawContours(oriframe, [hull], -1, (255, 255, 0), 3)
			cv2.drawContours(oriframe, contours, -1, (255, 255, 0), 3)
			cv2.imshow('contours', oriframe)

	interrupt=cv2.waitKey(10)
	
	# Quit by pressing 'q'
	if  interrupt & 0xFF == ord('q'):
		break
	elif interrupt & 0xFF == ord('b'):
		bg_model = cv2.createBackgroundSubtractorMOG2(10, 100, True)
		bg_captured = 1
		

camera.release()
