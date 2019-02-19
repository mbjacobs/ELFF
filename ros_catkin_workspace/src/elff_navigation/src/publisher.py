#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
#from Tkinter import *
from socket import *      # Import necessary modules

ctrl_cmd = ['forward', 'backward', 'left', 'right', 'stop', 'read cpu_temp', 'home', 'distance', 'x+', 'x-', 'y+', 'y-', 'xy_home']

#top = Tk()   # Create a top window
#top.title('Sunfounder Raspberry Pi Smart Video Car')

# =============================================================================
# Define publisher node's interface to the rest of ROS
# =============================================================================
def gesture_recognizer ():
    pub = rospy.Publisher ('motioncommand', String, queue_size=10)
    rospy.init_node ('gesture_recognizer', anonymous = True)
    rate = rospy.Rate (10)

    while not rospy.is_shutdown ():
        #motion_str = 'forward'
        #rospy.loginfo (motion_str)
        #pub.publish (motion_str)
        forward_fun (pub)

        rate.sleep ()

# =============================================================================
# The function is to send the command forward to the server, so as to make the
# car move forward.
# =============================================================================
def forward_fun (pub):
	print 'forward'
        motion_str = 'forward'
        rospy.loginfo (motion_str)
        pub.publish (motion_str)

def backward_fun(event):
	print 'backward'
	tcpCliSock.send('backward')

def left_fun(event):
	print 'left'
	tcpCliSock.send('left')

def right_fun(event):
	print 'right'
	tcpCliSock.send('right')

def stop_fun(event):
	print 'stop'
	tcpCliSock.send('stop')

def home_fun(event):
	print 'home'
	tcpCliSock.send('home')

def x_increase(event):
	print 'x+'
	tcpCliSock.send('x+')

def x_decrease(event):
	print 'x-'
	tcpCliSock.send('x-')

def y_increase(event):
	print 'y+'
	tcpCliSock.send('y+')

def y_decrease(event):
	print 'y-'
	tcpCliSock.send('y-')

def xy_home(event):
	print 'xy_home'
	tcpCliSock.send('xy_home')

# =============================================================================
# Exit the GUI program and close the network connection between the client
# and server.
# =============================================================================
def quit_fun(event):
	top.quit()
	tcpCliSock.send('stop')
	tcpCliSock.close()

spd = 50

def changeSpeed(ev=None):
	tmp = 'speed'
	global spd
	spd = speed.get()
	data = tmp + str(spd)  # Change the integers into strings and combine them with the string 'speed'.
	print 'sendData = %s' % data
	tcpCliSock.send(data)  # Send the speed data to the server(Raspberry Pi)

#label = Label(top, text='Speed:', fg='red')  # Create a label
#label.grid(row=6, column=0)                  # Label layout

#speed = Scale(top, from_=0, to=100, orient=HORIZONTAL, command=changeSpeed)  # Create a scale
#speed.set(50)
#speed.grid(row=6, column=1)

if __name__ == '__main__':
        try:
            gesture_recognizer ()
        except rospy.ROSInterruptException:
            pass
