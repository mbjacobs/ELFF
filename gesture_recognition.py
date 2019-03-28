#!/usr/bin/python

import sys
import getopt
import cv2 as cv
import numpy as np
import math
from enum import Enum
import rospy 
from std_msgs.msg import String

class Gesture(Enum):
    TOGGLESTART = 1
    REVERSE = 2
    LEFT = 3
    RIGHT = 4
    SCOOP = 5

class DIRECTION(Enum):
    NORTH = 1
    EAST = 2
    SOUTH = 3
    WEST = 4

class cvGestures():
    def __init__(self):
        self.gaussian_ksize = 41 
        self.gaussian_sigma = 0
        self.thresholdLowValue = 30 #60
        self.thresholdMaxValue = 255
        self.bgSubThreshold = 50
        self.gestureNumberOfFingersScoop = 4
        self.gestureNumberOfFingersToggleStart = 5
        self.gestureNumberOfFingersReverseLeftRight = 1
        self.percentThresholdToQualifyGesture = 0.9
        self.timeIncrement = 5
        self.ScoopCounter = 0
        self.ToggleCounter = 0
        self.ReverseCounter = 0
        self.LeftCounter = 0
        self.RightCounter = 0

    def findContourCenter(self, contour):
        moments = cv.moments(contour)
        if (0 != moments["m00"]):
            centerX = int(moments["m10"] / moments["m00"])
            centerY = int(moments["m01"] / moments["m00"])
            return True, centerX, centerY
        return False, 0, 0

    def findExtremePoints(self, contour):
        extremeWest = tuple(contour[contour[:, :, 0].argmin()][0])
        extremeEast = tuple(contour[contour[:, :, 0].argmax()][0])
        extremeNorth = tuple(contour[contour[:, :, 1].argmin()][0])
        extremeSouth = tuple(contour[contour[:, :, 1].argmax()][0])
        return extremeWest, extremeEast, extremeNorth, extremeSouth

    def compareContourCenterWithExtremes(self, contour):
        findCenterSuccess, centerX, centerY = self.findContourCenter(contour)
        if (findCenterSuccess):
            extremeWest, extremeEast, extremeNorth, extremeSouth = self.findExtremePoints(contour)
            westFromCenter = abs (centerX - extremeWest[0])
            eastFromCenter = abs (centerX - extremeEast[0])
            northFromCenter = abs (centerY - extremeNorth[1])
            southFromCenter = abs (centerY - extremeSouth[1])

            maxDifference = max (westFromCenter, eastFromCenter, northFromCenter, southFromCenter)
            if maxDifference == westFromCenter:
                direction = DIRECTION.WEST
            elif maxDifference == eastFromCenter:
                direction = DIRECTION.EAST
            elif maxDifference == northFromCenter:
                direction = DIRECTION.NORTH
            elif maxDifference == southFromCenter:
                direction = DIRECTION.SOUTH
            else:
                direction = None
        else:
            direction = None
        return direction

    def printCounters(self):
        print("ScoopCounter:")
        print(self.ScoopCounter)
        print("ToggleCounter:")
        print(self.ToggleCounter)
        print("ReverseCounter:")
        print(self.ReverseCounter)
        print("LeftCounter:")
        print(self.LeftCounter)
        print("RightCounter:")
        print(self.RightCounter)

    def resetCounters(self):
        self.ScoopCounter = 0
        self.ToggleCounter = 0
        self.ReverseCounter = 0
        self.LeftCounter = 0
        self.RightCounter = 0

    def evaluateGestureOverTime (self):
        if self.ScoopCounter >= (self.timeIncrement * self.percentThresholdToQualifyGesture):
            gesture = Gesture.SCOOP
        elif self.ToggleCounter >= (self.timeIncrement * self.percentThresholdToQualifyGesture):
            gesture = Gesture.TOGGLESTART
        elif self.ReverseCounter >= (self.timeIncrement * self.percentThresholdToQualifyGesture):
            gesture = Gesture.REVERSE
        elif self.LeftCounter >= (self.timeIncrement * self.percentThresholdToQualifyGesture):
            gesture = Gesture.LEFT
        elif self.RightCounter >= (self.timeIncrement * self.percentThresholdToQualifyGesture):
            gesture = Gesture.RIGHT
        else:
            gesture = None
        return gesture

    def countGesture(self, gesture):
        if gesture is Gesture.TOGGLESTART:
            self.ToggleCounter += 1
        elif gesture is Gesture.SCOOP:
            self.ScoopCounter += 1
        elif gesture is Gesture.REVERSE:
            self.ReverseCounter += 1
        elif gesture is Gesture.LEFT:
            self.LeftCounter += 1
        elif gesture is Gesture.RIGHT:
            self.RightCounter += 1

    def identifyGesture(self, numFingers, direction):
        if (numFingers >= self.gestureNumberOfFingersToggleStart):
            gesture = Gesture.TOGGLESTART
        elif (numFingers == self.gestureNumberOfFingersScoop):
            gesture = Gesture.SCOOP
        elif (numFingers == self.gestureNumberOfFingersReverseLeftRight):
            if direction == DIRECTION.WEST:
                gesture = Gesture.RIGHT
            elif direction == DIRECTION.EAST:
                gesture = Gesture.LEFT
            elif direction == DIRECTION.NORTH: 
                gesture = Gesture.REVERSE
            else:
                gesture = None
        else:
            gesture = None
        return gesture

    def countFingers(self, largestContour):  # -> finished bool, counted fingers
        #  convexity defect
        hull = cv.convexHull(largestContour, returnPoints=False)
        if len(hull) > 3:
            defects = cv.convexityDefects(largestContour, hull)
            if type(defects) != type(None):  

                fingerCount = 1
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

    def captureBackground(self): # -> background model
        bgModel = cv.createBackgroundSubtractorMOG2(0, self.bgSubThreshold)
        return bgModel

    def removeBackground(self, bgModel, frame): # -> background subtracted frame
        fg_mask = bgModel.apply(frame, learningRate = 0) # learning rate defines how often the background reinitialized
        kernel = np.ones((3,3),np.uint8)
        fg_mask = cv.erode(fg_mask,kernel,iterations = 1)
        frame = cv.bitwise_and(frame,frame,mask = fg_mask)
        return frame

    def findLargestContour(self, contours): # -> found largest bool, largest contour OR 0
        if len(contours) > 0:
            return True, max(contours, key=cv.contourArea)
        else:
            return False, 0

    def publishGesture(self, gesture):
        rospy.init_node('topic_command_publisher', anonymous=True)
        pub = rospy.Publisher('motioncommand', String)
        if gesture == Gesture.TOGGLESTART:
            command = "TOGGLESTART"
        elif gesture == Gesture.REVERSE:
            command = "REVERSE"
        elif gesture == Gesture.LEFT:
            command = "LEFT"
        elif gesture == Gesture.RIGHT:
            command = "RIGHT"
        elif gesture == Gesture.SCOOP:
            command = "SCOOP"
        else:
            command = "NONE"
        rospy.loginfo(command)
        pub.publish(command)


def main(argv):
    cvGesture = cvGestures()
    camera = cv.VideoCapture(0)
    bBGCaptured = False
    counter = 0
    iteration = 0

    while camera.isOpened():
        camReadRV, frame = camera.read()

        if camReadRV is False:
            print ("CAMERA DID NOT CAPTURE")
        else:
            cv.imshow("show image",frame)
        if bBGCaptured is True:
            bgRemovedFrame = cvGesture.removeBackground(bgModel, frame)
            cv.imshow("bg image", bgRemovedFrame) # debug
            grayFrame = cv.cvtColor(bgRemovedFrame, cv.COLOR_BGR2GRAY)
            blurFrame = cv.GaussianBlur(grayFrame, (cvGesture.gaussian_ksize, cvGesture.gaussian_ksize), cvGesture.gaussian_sigma)
            threshRV, threshFrame = cv.threshold(blurFrame, cvGesture.thresholdLowValue, cvGesture.thresholdMaxValue, cv.THRESH_BINARY)
            cv.imshow("thresholded image", threshFrame) # debug
            _, contours, _ = cv.findContours(threshFrame, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            largestContourRV = False
            largestContourRV, largestContour = cvGesture.findLargestContour(contours)
            iteration += 1 # debug
            if largestContourRV is True:
                calcFingersRV, fingerCount = cvGesture.countFingers(largestContour)
                direction = cvGesture.compareContourCenterWithExtremes(largestContour)
                if (counter <= cvGesture.timeIncrement):
                    counter += 1
                    cvGesture.countGesture(cvGesture.identifyGesture(fingerCount, direction))
                else:
                    print ("ITERATION")# debug
                    print (iteration)# debug
                    evaluatedGesture = cvGesture.evaluateGestureOverTime()
                    if evaluatedGesture is not None:
                        #cvGesture.publishGesture (evaluatedGesture)
                        print("####################")# debug
                        print("EVALULATED GESTURE:")# debug
                        print("####################")# debug
                        print (evaluatedGesture)# debug
                        print("####################")# debug
                        print("####################")# debug
                    counter = 0
                    cvGesture.resetCounters()
        keyPress = cv.waitKey(10)
        if keyPress == 27:  # press ESC to exit
            break
        if keyPress == ord('c'): # press 'c' to print counts
            cvGesture.printCounters()
        if keyPress == ord('f'): # press 'f' to print current finger count
            print(fingerCount)
        elif keyPress == ord('b'):  # press 'b' to capture the background
            bgModel = cvGesture.captureBackground()
            bBGCaptured = True
            print( '!!!Background Captured!!!')

if __name__ == '__main__':
    main(sys.argv)
