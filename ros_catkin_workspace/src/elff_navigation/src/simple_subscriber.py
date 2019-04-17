#!/usr/bin/env python
import RPi.GPIO as GPIO
import video_dir
import car_dir
import motor
import rospy
from std_msgs.msg import String  #MJ012019: Don't know for sure if I need this, might be included in other packages...

from socket import *
from time import ctime          # Import necessary modules

ctrl_cmd = ['forward', 'backward', 'left', 'right', 'stop', 'read cpu_temp', 'home', 'distance', 'x+', 'x-', 'y+', 'y-', 'xy_home']

busnum = 1          # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

#==============================================================
# Establish the callback function and listener node for ROS
#==============================================================
def callback (data):
    rospy.loginfo (rospy.get_caller_id () + "Direction is %s", data.data)

class Direction:
    def __init__ (self):
        self.directionMsg = None

    def direction_callback (self, msg):
        #print "This is the message from the publisher."
        #print msg
        self.directionMsg = msg
        print "This is the direction stored in the direction class."
        print self.directionMsg

def mover ():

    sRobotDirection = Direction ()

    rospy.init_node ('mover', anonymous = True)

    rospy.Subscriber ("motioncommand", String, sRobotDirection.direction_callback)

    video_dir.setup(busnum=busnum)
    car_dir.setup(busnum=busnum)
    motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor.
    video_dir.home_x_y()
    car_dir.home()

    #data = sRobotDirection.directionMsg

    rospy.spin ()

if __name__=='__main__':
    mover ()

