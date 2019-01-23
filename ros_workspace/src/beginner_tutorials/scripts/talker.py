#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker ():

    #Defines talker's interface to the rest of ROS
    pub = rospy.Publisher ('chatter', String, queue_size=10) #Declares node publishing to 'chatter' topic via message type String

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node ('talker', anonymous = True) #Tells rosy the unique name of your node, which will be shared with roscore (ROS Master)
    
    rate = rospy.Rate (10) #10hz
    
    while not rospy.is_shutdown (): #Node logic
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo (hello_str) #Writes to screen, the node's log file, and rosout (good for debugging)
        pub.publish (hello_str) #Publishes string to chatter topic
        rate.sleep () #Sleeps long enough for the rate of 10 loops/second rate to be maintained

if __name__ == '__main__':
    try:
        talker ()
    except rospy.ROSInterruptException: #Catches if sleep throws an exception
        pass
