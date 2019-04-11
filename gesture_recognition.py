#!/usr/bin/python

import sys
import getopt
import cv2 as cv
import numpy as np
import math
from enum import Enum
import rospy 
from std_msgs.msg import String

class Command(Enum):
    START = 1
    REVERSE = 2
    LEFT = 3
    RIGHT = 4
    SCOOP = 5
    STOP = 6

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
        self.timeIncrement = 20

        self.ScoopCounter = 0
        self.ToggleCounter = 0
        self.ReverseCounter = 0
        self.LeftCounter = 0
        self.RightCounter = 0
        self.CommandTuple = (  [Command.START, "START", self.ToggleCounter],
                               [Command.REVERSE, "REVERSE", self.ReverseCounter],
                               [Command.LEFT, "LEFT", self.LeftCounter],
                               [Command.RIGHT, "RIGHT", self.RightCounter],
                               [Command.SCOOP, "SCOOP", self.ScoopCounter],
                               [Command.STOP, "STOP", self.ToggleCounter]
                            )
        self.CmdTupleCommand = 0
        self.CmdTupleString = 1
        self.CmdTupleCounter = 2                               

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

    def findLargestContour(self, contours): # -> found largest bool, largest contour OR 0
        if len(contours) > 0:
            return True, max(contours, key=cv.contourArea)
        else:
            return False, 0

    def evaluateGestureOverTime (self):
        gesture = None
        for i in self.CommandTuple:
            if i[self.CmdTupleCounter] >= (self.timeIncrement * self.percentThresholdToQualifyGesture):
                gesture = i
                ############FIX HERE????
        return gesture

    def countGesture(self, gesture):
        if gesture != None:
            for i in self.CommandTuple:
                if gesture == i[self.CmdTupleCommand]:
                    i[self.CmdTupleCounter] += 1
            
    def identifyGesture(self, numFingers, direction):
        if (numFingers >= self.gestureNumberOfFingersToggleStart):
            gesture = Command.START
        elif (numFingers == self.gestureNumberOfFingersScoop):
            gesture = Command.SCOOP
        elif (numFingers == self.gestureNumberOfFingersReverseLeftRight):
            if direction == DIRECTION.WEST:
                gesture = Command.RIGHT
            elif direction == DIRECTION.EAST:
                gesture = Command.LEFT
            elif direction == DIRECTION.NORTH: 
                gesture = Command.REVERSE
            else:
                gesture = None
        else:
            gesture = None
        return gesture

    def captureBackground(self): # -> background model
        bgModel = cv.createBackgroundSubtractorMOG2(0, self.bgSubThreshold)
        return bgModel

    def removeBackground(self, bgModel, frame): # -> background subtracted frame
        fg_mask = bgModel.apply(frame, learningRate = 0) # learning rate defines how often the background reinitialized
        kernel = np.ones((3,3),np.uint8)
        fg_mask = cv.erode(fg_mask,kernel,iterations = 1)
        frame = cv.bitwise_and(frame,frame,mask = fg_mask)
        return frame

    def publishGesture(self, gesture):
        rospy.init_node('topic_command_publisher', anonymous=True)
        pub = rospy.Publisher('motioncommand', String)
        command = gesture[1]
        rospy.loginfo(command)
        pub.publish(command)

    def resetCounters(self):
        for i in self.CommandTuple:
            i[self.CmdTupleCounter] = 0
  
    def printCounters(self):
        print("ToggleCounter:")
        print(self.CommandTuple[0][2])
        print("ReverseCounter:")
        print(self.CommandTuple[1][2])
        print("LeftCounter:")
        print(self.CommandTuple[2][2])
        print("RightCounter:")
        print(self.CommandTuple[3][2])
        print("ScoopCounter:")
        print(self.CommandTuple[4][2])

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
                    print("####################")# debug
                    print ("ITERATION")# debug
                    print (iteration)# debug
                    evaluatedGesture = cvGesture.evaluateGestureOverTime()
                    if evaluatedGesture is not None:
                        #cvGesture.publishGesture (evaluatedGesture)
                        print("EVALULATED GESTURE:")# debug
                        print (evaluatedGesture[1])# debug
                    counter = 0
                    cvGesture.printCounters()
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
