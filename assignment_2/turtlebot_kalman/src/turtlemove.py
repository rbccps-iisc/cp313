#!/usr/bin/env python

'''
    This node subscribes to linear and angular velocty of the turtlebot, adds noise and publishes it back
        on topic /noisy_vel
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time




''' You may tweak these parameters '''
''' Mean of velocity sensor input '''
linear_vel = 0.2
angular_vel = 0.2
''' standard deviation of velocity sensor inputs '''
linear_vel_std = 0.05
angular_vel_std = 0.05



''' ROS Node initiation '''
rospy.init_node('turtlebot_move')
rate = rospy.Rate(100)



''' Subscribers and Publishers '''
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
noisy_vel_pub = rospy.Publisher('/noisy_vel', Twist, queue_size=1)

''' Wait for gazebo to initialize '''
time.sleep(10)
''' Main loop '''
while(not rospy.is_shutdown()):

    twist_msg = Twist()
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel 

    noisy_twist_msg = Twist()
    ''' Add noise to pure measurements '''
    noisy_twist_msg.linear.x = np.random.normal(linear_vel,linear_vel_std)
    noisy_twist_msg.angular.z = np.random.normal(angular_vel,angular_vel_std)

    ''' Publish the control input to vel_pub '''
    vel_pub.publish(twist_msg)
    ''' Publish the noisy sensor readings to input to noisy_vel_pub '''
    noisy_vel_pub.publish(noisy_twist_msg)

    rate.sleep()
