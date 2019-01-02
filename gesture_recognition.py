#!/usr/bin/python

import sys
import cv2 as cv
import numpy as np
import math

class cvGestures():
    def __init__(self):
        self.bCaptureDone = False
        self.bBGCaptured = False
        self.gaussian_ksize = 41 
        self.gaussian_sigma = 0
        self.thresholdLowValue = 60
        self.thresholdMaxValue = 255
        self.bgSubThreshold = 50

    def countFingers(largestContour):  # -> finished bool, counted fingers
        #  convexity defect
        hull = cv.convexHull(largestContour, returnPoints=False)
        if len(hull) > 3:
            defects = cv.convexityDefects(largestContour, hull)
            if type(defects) != type(None):  

                fingerCount = 0
                for i in range(defects.shape[0]):  # calculate the angle
                    startPoint, endPoint, farthestPoint, distanceApprox = defects[i][0]
                    start = tuple(largestContour[startPoint][0])
                    end = tuple(largestContour[endPoint][0])
                    far = tuple(largestContour[farthestPoint][0])
                    a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                    b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                    c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                    angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # cosine theorem
                    if angle <= math.pi / 2:  # angle less than 90 degree, treat as fingers
                        fingerCount += 1
                return True, fingerCount
        return False, 0

    def removeBackground(frame): # -> background subtracted frame
        fg_mask = bgModel.apply(frame, learningRate = 0) # learning rate defines how often the background reinitialized
        kernel = np.ones((3,3),np.uint8)
        fg_mask = cv.erode(fg_mask,kernel,iterations = 1)
        frame = cv.bitwise_and(frame,frame,mask = fg_mask)
        cv.imshow("bg image",frame) # debug
        return frame

    def findLargestContour(contours): # -> found largest bool, largest contour OR 0
        max_area=0
        largest_contour=-1
        for i in range(len(contours)):
            cont=contours[i]
            area=cv.contourArea(cont)
            if(area>max_area):
                max_area=area
                largest_contour=i
        if(largest_contour==-1):
            return False, 0
        else:
            largestContour=contours[largest_contour]
            return True, largestContour
    
def main(args):
    cvGesture = cvGestures()
    camera = cv.VideoCapture(0)
    while camera.isOpened():
        camReadRV, frame = camera.read()

        if camReadRV is False:
            print ("CAMERA DID NOT CAPTURE")
        else:
            cv.imshow("show image",frame)
        if cvGesture.bBGCaptured is True:
            bgRemovedFrame = removeBackground(frame)
            grayFrame = cv.cvtColor(bgRemovedFrame, cv.COLOR_BGR2GRAY)
            blurFrame = cv.GaussianBlur(grayFrame, (cvGesture.gaussian_ksize, cvGesture.gaussian_ksize), cvGesture.gaussian_sigma)
            threshRV, threshFrame = cv.threshold(blurFrame, cvGesture.thresholdLowValue, cvGesture.thresholdMaxValue, cv.THRESH_BINARY)
            if threshRV is False:
                print ("cv2.threshold failure")
            _, contours, _ = cv.findContours(threshFrame, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            largestContourRV, largestContour = findLargestContour(contours)
            if largestContourRV is False:
                print("findLargestContour failure")
            else:
                calcFingersRV, fingerCount = countFingers(largestContour)
                if calcFingersRV is False:
                    print("countFingers failure")
                print("Finger Count: {fingerCount}".format(fingerCount = fingerCount))

        k = cv.waitKey(10)
        if k == 27:  # press ESC to exit
            break
        elif k == ord('b'):  # press 'b' to capture the background
            bgModel = cv.createBackgroundSubtractorMOG2(0, cvGesture.bgSubThreshold)
            bBGCaptured = True
            print( '!!!Background Captured!!!')

if __name__ == '__main__':
    main(sys.argv)


