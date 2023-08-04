#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(msg):
	rospy.loginfo("Recevied message: %s", msg.data)

def listener():
	rospy.init_node('listener_node')
	rospy.Subscriber('counter', Int32, callback)
	rospy.spin()

if __name__=='__main__':
	listener()

