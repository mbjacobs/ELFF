#!/usr/bin/env python
import RPi.GPIO as GPIO
import video_dir
import car_dir
import motor
import rospy
from std_msgs.msg import String  

from socket import *
from time import ctime          # Import necessary modules

ctrl_cmd = ['TOGGLESTART', 'REVERSE', 'LEFT', 'RIGHT', 'SCOOP', 'RESET']

busnum = 1          

#==============================================================
# Establish the callback function for ROS
#==============================================================
def callback (data):
    rospy.loginfo (rospy.get_caller_id () + "Callback! Direction is %s", data.data)

class Direction:
    def __init__ (self):
        self.directionMsg = None

    def direction_callback (self, msg):
        self.directionMsg = msg.data
        #print ("directionMsg is...", self.directionMsg) #debug

#def doAction (data, counter):
#Tried to move the nested loop in here but couldn't figure out why
#things weren't working.

def mover ():
    #Init variables for the control loop
    counter = 1
    bDriving = False
    bEndLoops = False

    sRobotDirection = Direction ()

    rospy.init_node ('mover', anonymous = True)

    rospy.Subscriber ("motioncommand", String, sRobotDirection.direction_callback)

    video_dir.setup(busnum=busnum)
    car_dir.setup(busnum=busnum)
    motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor.
    video_dir.home_x_y()
    car_dir.home()
 
    while (bEndLoops == False): # Loop to wait for received commands.
        
        while (bEndLoops == False): # Loop to perform movement controls while input received.

                data = sRobotDirection.directionMsg

                # Analyze the command received and control the car accordingly.
                #doAction (data, counter)
                #counter += 1
                #print counter
                
                if not data:
                    break
                
                if data == ctrl_cmd[0]: #and bDriving == False#:
                        print 'ELFF WILL DRIVE'
                        counter += 1
                        print counter
                       
                        try:
                                spd = 50
                                print "Moving forward with speed!"
                                motor.forwardWithSpeed (spd)
                                #bDriving = True
                        except:
                                print 'Error speed =' + str (spd)

               # elif data == ctrl_cmd[0] and bDriving == True:                            
                #        print 'ELFF WILL STOP'
                 #       motor.ctrl(0); #Stop the car

                elif data == ctrl_cmd[1]:
                        print 'ELFF WILL REVERSE'
                        counter += 1
                        print counter

                        try:
                                spd = 50
                                print "Moving backward with speed!"
                                motor.backwardWithSpeed (spd)
                        except:
                                print 'Error speed =' + str (spd)

                elif data == ctrl_cmd[2]:
                        print 'ELFF WILL GO LEFT'
                        counter += 1
                        car_dir.turn_left()

                elif data == ctrl_cmd[3]:
                        print 'ELFF WILL GO RIGHT'
                        counter += 1
                        car_dir.turn_right()

               # elif data == ctrl_cmd[6]:
                #        print 'recv home cmd'
                 #       #car_dir.home()

                elif data == ctrl_cmd[4]:
                        print 'ELFF WILL SCOOP (BUT FOR NOW STOP MOTORS)'
                        car_dir.home()
                        motor.ctrl(0)
                        
                        bEndLoops = True
                        ###video_dir.move_increase_y()

                            
                elif data == ctrl_cmd[5]: # Used with publisher.py only as a debug method.
                        print 'ELFF WILL RESET POSITION'
                        car_dir.home()
                        motor.ctrl(0)
                        bEndLoops = True

                else:
                        print 'Waiting to receive a command...'
                        
    rospy.spin ()
    #car_dir.home()


if __name__=='__main__':
    mover ()
