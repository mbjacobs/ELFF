#!/usr/bin/python

import rospy
from std_msgs.msg import String

def callback_print(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('command', String, callback_print)
    rospy.spin()

if __name__ == '__main__':
    listener()
 
