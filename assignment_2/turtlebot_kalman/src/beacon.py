#!/usr/bin/env python


'''
    This node subscribes to gazebo's position messages for the turtle bot and the beacon.
    It additionally computes rho, the radial distance between the beacon and turtlebot and 
        phi, the angle between turtlebot heading and line joining turtlebot and beacon, 
        publishing it under /beacon topic.
'''



import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from turtlebot_kalman.msg import BeaconMsg
import math
import numpy as np



''' you may tweak these parameters '''
''' Beacon detection thresholds in meters'''
beacon_threshold = 3
radial_std = 0.1
angular_std = 0.1




''' Global variables '''
''' Heading of the turtlebot '''
yaw = 0.
''' Angle between turtlebot heading and line joining turtlebot and beacon '''
phi  = 0.
''' Radial distance between turtlebot and beacon '''
distance = 0.


''' Gazebo simulation messages '''
turtlebot_pos = Point()
turtlebot_rot = Point()
beacon_pos = Point()



''' Gazebo messages callback for beacon and turtlebot positions'''
def models_cb(data):
    global turtlebot_pos, beacon_pos
    beacon_idx = data.name.index('unit_cylinder')
    turtle_idx = data.name.index('turtlebot3_burger')
    turtlebot_pos = data.pose[turtle_idx].position
    beacon_pos = data.pose[beacon_idx].position
    ''' You can print the beacon_pos if needed '''


''' Turtlebot odometry callback '''
def odom_cb(data):
    global yaw, phi
    orientation_q = data.pose.pose.orientation
    orientation_list = (orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    ''' phi is '''
    phi = -yaw + np.arctan2(-turtlebot_pos.y + beacon_pos.y, -turtlebot_pos.x + beacon_pos.x)


rospy.init_node('beacon')
rate = rospy.Rate(100)
''' Subscribe to gazebo models and turtlebot odom '''
rospy.Subscriber('/gazebo/model_states', ModelStates, models_cb)
rospy.Subscriber('/odom', Odometry, odom_cb)
''' beacon signal publisher '''
beacon_pub = rospy.Publisher('beacon', BeaconMsg, queue_size=3)

print(" Beacon is at position (" + str(beacon_pos.x) + ", " + str(beacon_pos.y) )

''' Main loop '''
while not rospy.is_shutdown():
    ''' Compute distance between beacon and turtlebot '''
    distance = math.sqrt(math.pow(turtlebot_pos.x - beacon_pos.x,
                                  2) + math.pow(turtlebot_pos.y - beacon_pos.y, 2))
    ''' If turtlebot within beacon threshold '''
    if(distance < beacon_threshold):
        beaconMsg = BeaconMsg()
        beaconMsg.header.stamp = rospy.Time.now()
        ''' Adding noise to pure measurements '''
        beaconMsg.beacon_rho = np.random.normal(distance, radial_std)
        beaconMsg.beacon_phi = np.random.normal(phi, angular_std)
        ''' Publish rho and phi '''
        beacon_pub.publish(beaconMsg)

        rate.sleep()
