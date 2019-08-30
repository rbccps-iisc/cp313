# Lab Assignment 2: Use the extended kalman filter to estimate the position of the Turtlebot from noisy sensor input and noisy beacon measrements
### Due Aug 27, 2019
### Assignment Link : https://github.com/rbccps-iisc/cp313/tree/master/assignment_2

Note: Please use python2 for this exercise.

## Introduction
The folder turtlebot_kalman contains a ros package (rospkg), which, when launched, opens gazebo with the turtlebot and a Radio-Frequency Beacon initialized.
The turtlebot initially moves in fixed circle and occassionally approaches the beacon. 
The turtle bot has a velocity estimating sensor (wheel encoder) as well as an angular velocity sensor, which compute its velocity and angular velocity to an extent (with noise). The actual trajectory of the turtlebot (ground truth) is known.
You are to estimate the position of the turtlebot as it executes this trajectory, based on the velocity and angular velocity inputs. Your prediction of the trajectory based only on velocity and angular velocity sensors will be approximate, and will drift. This can be corrected occasionally based on the measurements from the beacon. The beacon gives you 
1. rho, the radial distance between the beacon and the turtlebot, and
2. phi, the angle between turtlebot heading and line joining turtlebot and beacon. The angle is positive when measured in an anti-clockwise direction.
The beacon gives you the above measurements only when the robot is within a distance of R from it. These measurements also have noise associated with them.



## Assignment
1. Formulate the motion model of the turtlebot (Newtonian motion model). The model must convert sensor inputs [in its local frame] (vx, w) to global position state variables (x,y,theta). You need to associate noise with the sensor inputs. You can assume that the noise covariance matrix is diagonal.
2. Formulate the measuement model of the beacon. The model must convert measurement of the turtlebot w.r.t the beacon (rho, phi) to the state variable of the turtlebot (x,y,theta). You need to associate noise with the measurements. You can assume that the noise covariance matrix is diagonal.
3. Use an extended kalman filter to fuse these measurements and provide a better estimate of the state variable (x,y,theta).
4. Compare it with ground truth (rostopic "/odom")


## Assumptions 
1. Assignment 1 is completed
2. You are able to control the turtlebot in gazebo from a ROS python code
3. You have a catkin_ws (workspace) in /home/<user-name>/catkin_ws

##  Installation instructions
1. From this folder, copy the folder turtlebot_kalman to the src folder of your catkin workspace
```
cp -r turtlebot_kalman ~/catkin_ws/src/
```
2. catkin_make the package using the command 
```
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps turtlebot_kalman
catkin_make --only-pkg-with-deps turtlebot_kalman install
source devel/setup.bash
```
3. You should now be able to run this package by 
```
roslaunch turtlebot_kalman turtlebot_beacon.launch
```

## Instructions
1. Inputs -
	a. (vx, w) - turtlebot local frame (linear velocity, counter-clockwise angular velocity). Subscribe to /noisy_vel.
	b. Ground truth for global position and angle (x,y,Q). Q is a rotationary quaternion. You may use an inbuilt function (see src/beacon.py) to convert this to euler angle. Alternatively, 
		since this is only used for plotting, you may plot it using rviz.  Subscribe to /odom. 
	c. Beacon is at position (2,2) and provides all angles w.r.t x axis



### Running the ROSBag file 
ROSBag is ROS's way of recording and storing data in a compressed form. You can create a ROSBag of sensor/actuator message recordings while 
executing your robot's manoeuvres and collect all sensor/actuator messages recordings. Replaying this bag will be similar to running the real robot, and multiple iterations 
of trial and error coding can be made.
(Remember to run roscore on a seperate terminal.)
First create a virtual static transformation, this will be helpful in plotting the data
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 0 odom base_footprint 10
```
Now, run the ROSBag, and pause it.
```
rosbag play ./data/tbot_lidar_loop.bag
```
Subscribe for the data using a simple ROS subscriber. Use code contained in ./code :
The code is well commented.

## Assignment
1. Plot live \/odom and \/odom_rf2o on rViz. (Instructions for plotting data on rViz can be found online and is part of the assignment.)
2. Use the utilities provided by ROS to obtain a time freeze (complete data) from the ROSBag and plot the same using matplotlib.
3. Considering the above, perform dead reckoning, i.e., 2-D position and pose ![equation](https://latex.codecogs.com/png.latex?2D%20%5C%20%28x%2C%20y%2C%20%5Ctheta%29) estimation, using just the IMU data. Compare with wheel odometry (use matplotlib).
4. More tasks will be added later.

## Submission Format
1. Folder name should be assignment_1_\<your_name\> and should contain folders /code /figures and report.pdf.
2. Put your code in \/code and the figures should be saved in \/figures.
3. Make a simple one-page report and store the pdf as report.pdf.
4. Submissions should be made on Piazza. Send the zip file of the assignment folder.

