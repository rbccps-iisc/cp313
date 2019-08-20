import rospy
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import numpy as np


class Turtle:

    def __init__(self):
        self.ax = self.ay = self.az = 0.  # Accelerometer accelerations
        self.yaw = 0.  # 2D rotation angle wrt turtle's front
        self.wx = self.wy = self.wz = 0  # Angular velocities about the axes
        self.groundTruth = [0., 0.]  # Wheel Odometry Measurement
        self.measurement = [0., 0.]  # Lidar  Odometry Measurement
        self.stateSet = False
        # self.timeFreeze = np.empty((1,7), dtype=np.float64) # Store timeseries into array
        pass

    def setState(self, state):
        self.ax = state.linear_acceleration.x
        self.ay = state.linear_acceleration.y
        self.az = state.linear_acceleration.z
        self.wx = state.angular_velocity.x
        self.wy = state.angular_velocity.y
        self.wz = state.angular_velocity.z
        quaternion = [state.orientation.x, state.orientation.y, state.orientation.z,
                      state.orientation.w]  # 3 Axis Compass gives quaternion output
        self.yaw = euler_from_quaternion(quaternion)[2] # Convert to Euler angle, and get yaw value

        # You could get store the time series data into an array
        # self.timeFreeze = np.concatenate((self.timeFreeze,np.array([[self.ax, self.ay,
        #    self.az, self.wx, self.wy, self.wz, self.yaw ]])))

    def updateMeasurement(self, state):
        self.measurement = [state.pose.pose.position.y,
                            -state.pose.pose.position.x]  # Axis is inverted by 90 degrees

    def updateGroundTruth(self, state):
        self.groundTruth = [state.pose.pose.position.x,
                            state.pose.pose.position.y]


def imu_subscriber(data):
    turtle.setState(data)


def odom_rf2o_subscriber(data):
    turtle.updateMeasurement(data)


def odom_subscriber(data):
    turtle.updateGroundTruth(data)


def main():
    global turtle

    rospy.init_node("turtle")
    rospy.Subscriber("/imu", Imu, imu_subscriber)
    rospy.Subscriber("/odom_rf2o", Odometry, odom_rf2o_subscriber)
    rospy.Subscriber("/odom", Odometry, odom_subscriber)
    rospy.spin()


if __name__ == '__main__':
    turtle = Turtle()
    main()
