#!/usr/bin/env python
import RPi.GPIO as GPIO
import scooper_dir
import car_dir
import motor
import rospy
from std_msgs.msg import String  

from socket import *
from time import ctime          # Import necessary modules

ctrl_cmd = ['START', 'REVERSE', 'LEFT', 'RIGHT', 'SCOOP', 'RESET', 'STOP']

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

    scooper_dir.setup(busnum=busnum)
    car_dir.setup(busnum=busnum)
    
    # Initialize the Raspberry Pi GPIO connected to the DC motor.
    motor.setup(busnum=busnum)     
    
    #video_dir.home_x_y()
    #scooper_dir.servo_test ()
    car_dir.home()
 
    # Loop to wait for received commands.
    while (bEndLoops == False): 
        
        # Loop to perform movement controls while input received.
        while (bEndLoops == False): 

                data = sRobotDirection.directionMsg

                # Analyze the command received and control the car accordingly.
                #doAction (data, counter)
                #counter += 1
                #print counter
                
                if not data:
                    break
                
                if data == ctrl_cmd[0]: 
                        print 'ELFF WILL DRIVE'
                        counter += 1
                        print counter
                       
                        try:
                                spd = 20
                                #print "Moving forward with speed!"
                                motor.forwardWithSpeed (spd)
                        except:
                                print 'Error speed =' + str (spd)

                elif data == ctrl_cmd[1]:
                        print 'ELFF WILL REVERSE'
                        counter += 1
                        print counter

                        try:
                                spd = 50
                                #print "Moving backward with speed!"
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

                elif data == ctrl_cmd[4]:
                        print 'ELFF WILL SCOOP'
                        
                        scooper_dir.home_x_y()
                        scooper_dir.doScoop()
                        scooper_dir.home_x_y()
                            
                # Used with publisher.py only as a debug method.
                elif data == ctrl_cmd[5]: 
                        print 'ELFF WILL RESET POSITION'
                        scooper_dir.home_x_y()
                        car_dir.home()
                        motor.ctrl(0)
                                                
                        bEndLoops = True

                elif data == ctrl_cmd[6]:
                        print 'ELFF WILL STOP'
                        counter += 1
                        print counter
                        try:
                                spd = 0
                                motor.forwardWithSpeed (spd)
                        except:
                                print 'Error speed =' + str (spd)


                else:
                        print 'Waiting to receive a command...'
                        
    rospy.spin ()
    #car_dir.home()


if __name__=='__main__':
    mover ()
