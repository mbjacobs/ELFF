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

# =============================================================================
# Create buttons
# =============================================================================
#Btn0 = Button(top, width=5, text='Forward')
#Btn1 = Button(top, width=5, text='Backward')
#Btn2 = Button(top, width=5, text='Left')
#Btn3 = Button(top, width=5, text='Right')
#Btn4 = Button(top, width=5, text='Quit')
#Btn5 = Button(top, width=5, height=2, text='Home')

# =============================================================================
# Buttons layout
# =============================================================================
#Btn0.grid(row=0,column=1)
#Btn1.grid(row=2,column=1)
#Btn2.grid(row=1,column=0)
#Btn3.grid(row=1,column=2)
#Btn4.grid(row=3,column=2)
#Btn5.grid(row=1,column=1)

# =============================================================================
# Bind the buttons with the corresponding callback function.
# =============================================================================
#Btn0.bind('<ButtonPress-1>', forward_fun)  # When button0 is pressed down, call the function forward_fun().
#Btn1.bind('<ButtonPress-1>', backward_fun)
#Btn2.bind('<ButtonPress-1>', left_fun)
#Btn3.bind('<ButtonPress-1>', right_fun)
#Btn0.bind('<ButtonRelease-1>', stop_fun)   # When button0 is released, call the function stop_fun().
#Btn1.bind('<ButtonRelease-1>', stop_fun)
#Btn2.bind('<ButtonRelease-1>', stop_fun)
#Btn3.bind('<ButtonRelease-1>', stop_fun)
#Btn4.bind('<ButtonRelease-1>', quit_fun)
#Btn5.bind('<ButtonRelease-1>', home_fun)

# =============================================================================
# Create buttons
# =============================================================================
#Btn07 = Button(top, width=5, text='X+', bg='red')
#Btn08 = Button(top, width=5, text='X-', bg='red')
#Btn09 = Button(top, width=5, text='Y-', bg='red')
#Btn10 = Button(top, width=5, text='Y+', bg='red')
#Btn11 = Button(top, width=5, height=2, text='HOME', bg='red')

# =============================================================================
# Buttons layout
# =============================================================================
#Btn07.grid(row=1,column=5)
#Btn08.grid(row=1,column=3)
#Btn09.grid(row=2,column=4)
#Btn10.grid(row=0,column=4)
#Btn11.grid(row=1,column=4)

# =============================================================================
# Bind button events
# =============================================================================
#Btn07.bind('<ButtonPress-1>', x_increase)
#Btn08.bind('<ButtonPress-1>', x_decrease)
#Btn09.bind('<ButtonPress-1>', y_decrease)
#Btn10.bind('<ButtonPress-1>', y_increase)
#Btn11.bind('<ButtonPress-1>', xy_home)
#Btn07.bind('<ButtonRelease-1>', home_fun)
#Btn08.bind('<ButtonRelease-1>', home_fun)
#Btn09.bind('<ButtonRelease-1>', home_fun)
#Btn10.bind('<ButtonRelease-1>', home_fun)
#Btn11.bind('<ButtonRelease-1>', home_fun)

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
