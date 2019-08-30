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
The beacon gives you the above measurements only when the robot is within a distance of rho=R from it. These measurements also have noise associated with them.
![Alt text](./docs/turtle.png?raw=true "Gazebo showing the simulated turtlebot and the beacon (cylinder)")


## Assignment
1. Formulate the motion model of the turtlebot (Newtonian motion model). The model must convert sensor inputs [in its local frame] (vx, w) (front velocity in x direction of turtle bot, and angular velocity to make it turn(+ve is anticlockwise))  to global position state variables (x,y,theta). You need to associate noise with the sensor inputs. You can assume that the noise covariance matrix is diagonal. The code ./turtlebot_kalman/src/turtlemove.py shows the standard deviations that have been set.
2. Formulate the measuement model of the beacon. The model must convert measurement of the turtlebot w.r.t the beacon (rho, phi) to the state variable of the turtlebot (x,y,theta). You need to associate noise with the measurements. You can assume that the noise covariance matrix is diagonal. The code ./turtlebot_kalman/src/beacon.py shows the standard deviations that have been set.
3. Use an extended kalman filter to fuse these measurements and provide a better estimate of the state variable (x,y,theta).
4. Compare it with ground truth (rostopic "/odom"), plot the odometry on rViz and the velocities on rqt-plot.


## Assumptions 
1. Assignment 1 is completed
2. You are able to control the turtlebot in gazebo from a ROS python code
3. You have a catkin_ws (workspace) in /home/"user-name"/catkin_ws

##  Installation instructions
1. Install filterpy, which is the library you will use to perform kalman filtering
```
sudo pip install filterpy
```
More information on how to use this library can be found here
https://filterpy.readthedocs.io/en/latest/
Examples can be found here
https://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/table_of_contents.ipynb

2. From this folder, copy the folder turtlebot_kalman to the src folder of your catkin workspace
```
cp -r turtlebot_kalman ~/catkin_ws/src/
```

3. catkin_make the package using the command 
```
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps turtlebot_kalman
catkin_make --only-pkg-with-deps turtlebot_kalman install
source devel/setup.bash
```
4. You should now be able to run this package by 
```
roslaunch turtlebot_kalman turtlebot_beacon.launch
```
Gazebo will now open and show you the turtlebot and a beacon (cylinder) as show
![Alt text](./docs/gaz.png?raw=true "Gazebo showing the simulated turtlebot and the beacon (cylinder)")

5. You may need to download rqt-plot separately.
```
sudo apt-get install ros-kinetic-rqt*
```
or
```
sudo apt-get install ros-melodic-rqt*
```
depending on your distribution.
Using this, you can plot the velocities, positions etc.
```
rqt_plot /noisy_vel
```
![Alt text](./docs/vel.png?raw=true "Gazebo showing the simulated turtlebot and the beacon (cylinder)")


## Instructions
1. Inputs -
	a. (vx, w) - turtlebot local frame (linear velocity, counter-clockwise angular velocity). Subscribe to /noisy_vel.
	b. Ground truth for global position and angle (x,y,Q). Q is a rotationary quaternion. You may use an inbuilt function (see src/beacon.py) to convert this to euler angle. Alternatively, 
		since this is only used for plotting, you may plot it using rviz.  Subscribe to /odom. 
	c. Beacon is at position (2,2) and provides all angles w.r.t x axis
2. Simulation - 
The simulation is mainly run by two scripts
    * ./turtlebot_kalman/src/beacon.py - Controls the RF signals to send to the turtlebot when it reaches close to a threshold. The radial thrshold and the noise variances can be set here
    * ./turtlebot_kalman/src/turtlemove.py - Controls the movement of the turtlebot, allowing it to go in a circular path. The sensor input variances (velocity) can be set here
A launch file is provided in ./turtlebot_kalman/launch which will run the above two scripts for you. 
3. Custom Message - 
A custom message for beacon is created for you in ./turtlebot_kalman/msg/BeaconMsg.msg which is a datastructure holding radius and angle as floats. You must necessarily perform step 3. of the installation instruction once and
source source devel/setup.bash in every terminal where you would be running programs from.




## Submission Format
1. Folder name should be assignment_1_\<your_name\> and should contain folders /code /figures and report.pdf.
2. Put your code in \/code and the figures should be saved in \/figures.
3. Make a simple one-page report and store the pdf as report.pdf.
4. Submissions should be made on Piazza by mailing to the TA and Instructor. Send the zip file of the assignment folder.

