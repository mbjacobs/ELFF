#!/usr/bin/env python
import RPi.GPIO as GPIO
import video_dir
import car_dir
import motor
import rospy
from std_msgs.msg import String  #MJ012019: Don't know for sure if I need this, might be included in other packages...

from socket import *
from time import ctime          # Import necessary modules

ctrl_cmd = ['TOGGLESTART', 'REVERSE', 'LEFT', 'RIGHT', 'SCOOP', 'read cpu_temp', 'home', 'distance', 'x+', 'x-', 'y+', 'y-', 'xy_home']

busnum = 1          # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

#==============================================================
# Establish the callback function and listener node for ROS
#==============================================================
def callback (data):
    rospy.loginfo (rospy.get_caller_id () + "Callback! Direction is %s", data.data)

class Direction:
    def __init__ (self):
        self.directionMsg = None

    def direction_callback (self, msg):
        self.directionMsg = msg.data
        print ("directionMsg is...", self.directionMsg) #debug

def mover ():

    sRobotDirection = Direction ()

    rospy.init_node ('mover', anonymous = True)

    rospy.Subscriber ("motioncommand", String, sRobotDirection.direction_callback)

    video_dir.setup(busnum=busnum)
    car_dir.setup(busnum=busnum)
    motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor.
    video_dir.home_x_y()
    car_dir.home()

    counter = 1

    while (counter != 20): #For debugging purpose to prevent blowing things up with an infinite loop

            while (counter != 20): #See above

                    data = sRobotDirection.directionMsg

                    # Analyze the command received and control the car accordingly.
                    if not data:
                            break
                    if data == ctrl_cmd[0]:
                            print 'ELLF WILL DRIVE'
                            #motor.forward()
                            counter += 1 #Debug
                    elif data == ctrl_cmd[1]:
                            print 'ELFF WILL REVERSE'
                            #motor.backward()
                    elif data == ctrl_cmd[2]:
                            print 'ELFF WILL GO LEFT'
                            #car_dir.turn_left()
                    elif data == ctrl_cmd[3]:
                            print 'ELFF WILL GO RIGHT'
                            #car_dir.turn_right()
                    elif data == ctrl_cmd[6]:
                            print 'recv home cmd'
                            #car_dir.home()
                    elif data == ctrl_cmd[4:
                            print 'ELFF WILL SCOOP'
                            #motor.ctrl(0)
                    elif data == ctrl_cmd[5]:
                            print 'read cpu temp...'
                            #temp = cpu_temp.read()
                            #tcpCliSock.send('[%s] %0.2f' % (ctime(), temp))
                    elif data == ctrl_cmd[8]:
                            print 'recv x+ cmd'
                            #video_dir.move_increase_x()
                    elif data == ctrl_cmd[9]:
                            print 'recv x- cmd'
                            #video_dir.move_decrease_x()
                    elif data == ctrl_cmd[10]:
                            print 'recv y+ cmd'
                            #video_dir.move_increase_y()
                    elif data == ctrl_cmd[11]:
                            print 'recv y- cmd'
                            #video_dir.move_decrease_y()
                    elif data == ctrl_cmd[12]:
                            print 'home_x_y'
                            #video_dir.home_x_y()
                    #elif data[0:5] == 'speed':     # Change the speed
                     #       print data
                      #      numLen = len(data) - len('speed')
                       #     if numLen == 1 or numLen == 2 or numLen == 3:
                        #            tmp = data[-numLen:]
                         #           print 'tmp(str) = %s' % tmp
                          #          spd = int(tmp)
                           #         print 'spd(int) = %d' % spd
                            #        if spd < 24:
                             #               spd = 24
                              #      motor.setSpeed(spd)
                   # elif data[0:5] == 'turn=':	#Turning Angle
                    #        print 'data =', data
                     #       angle = data.split('=')[1]
                      #      try:
                       #             angle = int(angle)
                        #            car_dir.turn(angle)
                         #   except:
                          #          print 'Error: angle =', angle
                    #elif data[0:8] == 'forward=':
                     #       print 'data =', data
                      #      spd = data[8:]
                       #     try:
                        #            spd = int(spd)
                         #           motor.forward(spd)
                          #  except:
                           #         print 'Error speed =', spd
                    #elif data[0:9] == 'backward=':
                    #        print 'data =', data
                    #        spd = data.split('=')[1]
                    #        try:
                    #                spd = int(spd)
                    #                motor.backward(spd)
                    #        except:
                    #                print 'ERROR, speed =', spd

                    else:
                            print 'Command Error! Cannot recognize command: ' + sRobotDirection.directionMsg
    rospy.spin ()

if __name__=='__main__':
    mover ()
