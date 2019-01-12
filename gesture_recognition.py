#!/usr/bin/python

import sys
import getopt
import cv2 as cv
import numpy as np
import math

class cvGestures():
    def __init__(self):
        #self.bCaptureDone = False
        self.gaussian_ksize = 41 
        self.gaussian_sigma = 0
        self.thresholdLowValue = 60
        self.thresholdMaxValue = 255
        self.bgSubThreshold = 50

    def countFingers(self, largestContour):  # -> finished bool, counted fingers
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

    def captureBackground(self): # -> background model
        bgModel = cv.createBackgroundSubtractorMOG2(0, self.bgSubThreshold)
        return bgModel

    def removeBackground(self, bgModel, frame): # -> background subtracted frame
        fg_mask = bgModel.apply(frame, learningRate = 0) # learning rate defines how often the background reinitialized
        kernel = np.ones((3,3),np.uint8)
        fg_mask = cv.erode(fg_mask,kernel,iterations = 1)
        frame = cv.bitwise_and(frame,frame,mask = fg_mask)
        return frame

    def captureColor(self, frame): # -> color histogram
        hsvframe = cv.cvtColor (frame, cv.COLOR_BGR2HSV)
        rows, cols, _ = frame.shape
        #roi = np.zeros([rows / 2, cols / 2], dtype = "uint8") # not sure if needed
        roi = hsvframe[int(rows * 0.25) : int(rows * 0.75), int(cols * 0.25) : int(cols * 0.75)] 
        color_hist = cv.calcHist([roi], [0,1], None, [180, 256], [0, 180, 0, 256])
        cv.normalize(color_hist, color_hist, 0, 255, cv.NORM_MINMAX)
        return color_hist

    def applyColorSegmentation(self, roihst, frame): # -> color segmented frame
        
        ######try 2
        print("applyColorSegmentation")
        # converted = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # skinMask = cv.inRange (converted, hsv[0], hsv[1])
        # kernel = cv.getStructuringElement (cv.MORPH_ELLIPSE, (11,11))
        # skinMask = cv.erode (skinMask, kernel, iterations = 2)
        # skinMask = cv.dilate (skinMask, kernel, iterations = 2)
        # skinMask = cv.GaussianBlur (skinMask, (3, 3), 0)
        # skin = cv.bitwise_and (frame, frame, mask = skinMask)
        # return skin
        ######


        ####try 1
        hsvframe = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        dst = cv.calcBackProject([hsvframe], [0,1], roihst, [0,180,0,256], 1)
        disc = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
        cv.filter2D(dst, -1, disc, dst)
        retVal, thresh = cv.threshold(dst, 50, 255, 0)
        thresh = cv.merge((thresh, thresh, thresh))
        #cv.GaussianBlur(dst, (3,3),0,dst)
        res = cv.bitwise_and(frame, thresh)
        red = np.vstack((frame, thresh, res))
        return res
        ######

    def findLargestContour(self, contours): # -> found largest bool, largest contour OR 0
        max_area = 0
        largest_contour = -1
        for i in range(len(contours)):
            cont = contours[i]
            area = cv.contourArea(cont)
            if area > max_area:
                max_area = area
                largest_contour = i

        if largest_contour is -1:
            return False, 0
        else:
            largestContour = contours[largest_contour]
            return True, largestContour

def main(argv):
    cvGesture = cvGestures()
    camera = cv.VideoCapture(0)
    commandlineOptions = "-c"
    bBGCaptured = False
    bColorCaptured = False
    bColorSegment = False

    if len(sys.argv) > 1:
        if sys.argv[1] in commandlineOptions:
            bColorSegment = True

    if bColorSegment is True:
        while camera.isOpened():
            camReadRV, frame = camera.read()

            if camReadRV is False:
                print ("CAMERA DID NOT CAPTURE")
            else:
                cv.imshow("show image",frame)
            if bColorCaptured is True:
                colorSegmentedFrame = cvGesture.applyColorSegmentation(colorHst, frame)
                cv.imshow("color img", colorSegmentedFrame)
                grayFrame = cv.cvtColor(colorSegmentedFrame, cv.COLOR_BGR2GRAY)
                blurFrame = cv.GaussianBlur(grayFrame, (cvGesture.gaussian_ksize, cvGesture.gaussian_ksize), cvGesture.gaussian_sigma)
                threshRV, threshFrame = cv.threshold(blurFrame, cvGesture.thresholdLowValue, cvGesture.thresholdMaxValue, cv.THRESH_BINARY)
                cv.imshow("thresholded image", threshFrame) # debug
                if threshRV is False:
                    print ("cv2.threshold failure")
                _, contours, _ = cv.findContours(threshFrame, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                largestContourRV, largestContour = cvGesture.findLargestContour(contours)
                if largestContourRV is False:
                    print("findLargestContour failure")
                else:
                    calcFingersRV, fingerCount = cvGesture.countFingers(largestContour)
                    if calcFingersRV is False:
                        print("countFingers failure")
                    #print("Finger Count: {fingerCount}".format(fingerCount = fingerCount))

            keyPress = cv.waitKey(10)
            if keyPress == 27:  # press ESC to exit
                break
            elif keyPress == ord('b'):  # press 'b' to capture the color
                colorHst = cvGesture.captureColor(frame)
                bColorCaptured = True
                print( '!!!Color Histogram Captured!!!')
    else:
        print ("hey")
        # while camera.isOpened():
        #     camReadRV, frame = camera.read()

        #     if camReadRV is False:
        #         print ("CAMERA DID NOT CAPTURE")
        #     else:
        #         cv.imshow("show image",frame)
        #     if bBGCaptured is True:
        #         bgRemovedFrame = cvGesture.removeBackground(bgModel, frame)
        #         cv.imshow("bg image", bgRemovedFrame) # debug
        #         grayFrame = cv.cvtColor(bgRemovedFrame, cv.COLOR_BGR2GRAY)
        #         blurFrame = cv.GaussianBlur(grayFrame, (cvGesture.gaussian_ksize, cvGesture.gaussian_ksize), cvGesture.gaussian_sigma)
        #         threshRV, threshFrame = cv.threshold(blurFrame, cvGesture.thresholdLowValue, cvGesture.thresholdMaxValue, cv.THRESH_BINARY)
        #         cv.imshow("thresholded image", threshFrame) # debug
        #         if threshRV is False:
        #             print ("cv2.threshold failure")
        #         _, contours, _ = cv.findContours(threshFrame, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #         largestContourRV, largestContour = cvGesture.findLargestContour(contours)
        #         if largestContourRV is False:
        #             print("findLargestContour failure")
        #         else:
        #             calcFingersRV, fingerCount = cvGesture.countFingers(largestContour)
        #             if calcFingersRV is False:
        #                 print("countFingers failure")
        #             print("Finger Count: {fingerCount}".format(fingerCount = fingerCount))

        #     keyPress = cv.waitKey(10)
        #     if keyPress == 27:  # press ESC to exit
        #         break
        #     elif keyPress == ord('b'):  # press 'b' to capture the background
        #         bgModel = cvGesture.captureBackground()
        #         bBGCaptured = True
        #         print( '!!!Background Captured!!!')

if __name__ == '__main__':
    main(sys.argv)
