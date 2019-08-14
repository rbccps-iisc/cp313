#!/usr/bin/env python

# Imported standard rospy module
import rospy
# Imported ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Global variables
pos = []

# ROS Node initiation
rospy.init_node('tbot_test')
# ROS Node rate
rate = rospy.Rate(100)

# ROS subscriber callbacks
def odom_cb(data):
	global pos
	pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z] 

# ROS Subscribers and Publishers, to be declared only once, at the beginning
rospy.Subscriber('/odom', Odometry, odom_cb)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

#Main loop
while(not rospy.is_shutdown()):
	# Global variables to be used in the main logic
	global pos
 
	# Main logic here
	# ...

	# Create an object of the message to be transmitted
	twist_msg = Twist()
	# Fill the corresponding fields. Use 'rosmsg show <message>' or 'rostopic info <topic_name>' for more info
	twist_msg.linear.x = .2
	twist_msg.angular.z = 1
	# Publish the message on the corresponding publisher for the topic
	vel_pub.publish(twist_msg)

	print (pos)

	# Sleep to keep the node running at a consistent rate.
	rate.sleep()
