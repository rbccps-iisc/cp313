#!/usr/bin/env python

'''
    This example node shows you how you can subscribe to data
'''

import rospy
from nav_msgs.msg import Odometry
from turtlebot_kalman.msg import BeaconMsg
from geometry_msgs.msg import Twist
import numpy as np
import time


beacon_msg_flag = 0
vel_msg_flag  = 0
beacon_msg = None
vel_msg = None

''' ROS Node initiation '''
rospy.init_node('turtlebot_example_node')
rate = rospy.Rate(100)


''' Turtlebot noisy velocity callback '''
def noisy_vel_cb(data):
    global vel_msg
    global vel_msg_flag
    vel_msg = data
    vel_msg_flag = 1
    pass

''' Turtlebot noisy velocity callback '''
def beacon_cb(data):
    global beacon_msg
    global beacon_msg_flag
    beacon_msg = data
    beacon_msg_flag = 1

''' Subscribers '''
rospy.Subscriber('/noisy_vel', Twist, noisy_vel_cb)
rospy.Subscriber('/beacon', BeaconMsg, beacon_cb)


while(not rospy.is_shutdown()):

    if(vel_msg_flag == 1):
        print(vel_msg)
    if(beacon_msg_flag == 1):
        print(beacon_msg)

