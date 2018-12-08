#!/usr/bin/python
import cv2
import numpy as np
import math

camera = cv2.VideoCapture(0)
bgModel = cv2.createBackgroundSubtractorMOG2()
bg_captured = 0

while(1):
	camReadRV, frame = camera.read()
	cv2.imshow ('Original Image', frame)

	cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.imshow('color changed', frame)

	#frame = cv2.bilateralFilter(frame, 9, 150, 150)
	frame = cv2.GaussianBlur (frame, (5,5), 0)
	cv2.imshow ('Smoothing Filter', frame)

	if (bg_captured == 1):
		fg_mask = bg_model.apply(frame, learningRate = 0)
		cv2.imshow ('background model', fg_mask)
		cv2.threshold(fg_mask, 100, 255, cv2.THRESH_BINARY) 
		cv2.imshow ('thresholding', fg_mask)


	interrupt=cv2.waitKey(10)
	
	# Quit by pressing 'q'
	if  interrupt & 0xFF == ord('q'):
		break
	elif interrupt & 0xFF == ord('b'):
		bg_model = cv2.createBackgroundSubtractorMOG2(0,16, False)
		bg_captured = 1
		

camera.release()
cv2.destroyWindow()