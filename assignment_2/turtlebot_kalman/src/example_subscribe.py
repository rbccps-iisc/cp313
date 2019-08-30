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


''' ROS Node initiation '''
rospy.init_node('turtlebot_example_node')
rate = rospy.Rate(100)


''' Turtlebot noisy velocity callback '''
def noisy_vel_cb(data):
    print(data)

''' Subscribers '''
rospy.Subscriber('/noisy_vel', Twist, noisy_vel_cb)


rospy.spin()
