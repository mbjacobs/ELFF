#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
#from Tkinter import *
from socket import *      # Import necessary modules

ctrl_cmd = ['forward', 'backward', 'left', 'right', 'stop', 'read cpu_temp', 'home']

#top = Tk()   # Create a top window
#top.title('Sunfounder Raspberry Pi Smart Video Car')

# =============================================================================
# Define publisher node's interface to the rest of ROS
# =============================================================================
def gesture_recognizer ():
    counter = 0
    
    pub = rospy.Publisher ('motioncommand', String, queue_size=10)
    rospy.init_node ('gesture_recognizer', anonymous = True)
    rate = rospy.Rate (10)

    while not rospy.is_shutdown ():
        #motion_str = 'forward'
        #rospy.loginfo (motion_str)
        #pub.publish (motion_str)
	#stop_fun (pub)

	#while (counter < 2500):
	    #forward_fun (pub)
	    #counter += 1
	    #print counter
       
        #stop_fun (pub)

        #while (counter < 1000):
         #   scoop_fun (pub)
	 #   sleep (0.5)
         #   counter += 1
         #   print counter

        
        #stop_fun (pub) 

	#while (counter < 5000):	
	    #backward_fun (pub)
	    #counter += 1
	    #print counter
        
        #stop_fun (pub)
	
        #while (counter < 7500):
	    #forward_fun (pub)
	    #counter += 1
	    #print counter

        #left_fun (pub)
        #left_fun (pub)

        #while (counter < 10000):
            #forward_fun (pub)
            #counter += 1
            #print counter
	    
        #right_fun (pub)
        #right_fun (pub)

        #while (counter < 12500):
            #forward_fun (pub)
            #counter += 1
            #print counter

	scoop_fun (pub)
        print 'END DEMO'
        
        rate.sleep ()

# =============================================================================
# The function is to send the command forward to the server, so as to make the
# car move forward.
# =============================================================================
def forward_fun (pub):
	print 'TOGGLESTART'
        motion_str = 'TOGGLESTART'
        rospy.loginfo (motion_str)
        pub.publish (motion_str)

def backward_fun(pub):
	print 'REVERSE'
	motion_str = 'REVERSE'
	rospy.loginfo (motion_str)
        pub.publish (motion_str)

def left_fun(pub):
	print 'LEFT'
	motion_str = 'LEFT'
	rospy.loginfo (motion_str)
        pub.publish (motion_str)

def right_fun(pub):
	print 'RIGHT'
	motion_str = 'RIGHT'
	rospy.loginfo (motion_str)
        pub.publish (motion_str)

def stop_fun(pub):
	print 'RESET'
        motion_str = 'RESET'
	pub.publish (motion_str)

def scoop_fun(pub):
    print 'SCOOP'
    motion_str='SCOOP'
    pub.publish (motion_str)

def home_fun(event):
	print 'home'

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
