#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback (data):
    rospy.loginfo (rospy.get_caller_id () + "Do you hear what I hear? %s", data.data) #the string of data printed is "hello world [timestamp]"

def listener ():
    rospy.init_node ('listener', anonymous = True)
    
    rospy.Subscriber ("chatter", String, callback) # Subscribe to the 'chatter' topic of type String
                                                   # When new messages received via 'chatter', invoke
                                                   # the callback function w/ the message as first argument 

    # spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
