# Use the given ROSBag file for a turtlebot executing a loop to obtain its 2D odometry

Note: Please use python2 for this exercise. You will additionally
Link: https://github.com/rbccps-iisc/cp313/tree/master/assignment_1


## Instructions
The ROSBag (in ./data) contains the following essential ROS topic sensor streams which also can be
found by running the command rostopic list:
1. /odom Wheel odometry ![equation](https://latex.codecogs.com/gif.latex?2D%20%5C%20%28x%2C%20y%2C%20%5Ctheta%29)
2. /odom_rf2o Lidar odometry ![equation](https://latex.codecogs.com/gif.latex?2D%20%5C%20%28x%2C%20y%2C%20%5Ctheta%29)
3. /imu 9DOF imu data ![equation](https://latex.codecogs.com/gif.latex?%28a_x%2C%20a_y%2C%20a_z%2C%20%5Cdot%7B%5Ctheta_x%7D%2C%20%5Cdot%7B%5Ctheta_y%7D%2C%20%5Cdot%7B%5Ctheta_z%7D%2C%20%5Ctheta_x%2C%20%5Ctheta_y%2C%20%5Ctheta_z%29)

You can perform rostopic echo <topic_name> to display the sensor output
Wheel odometry is the most accurate amongst these for the scenario in which the data was
generated. Lidar odometry tends to be noisy and the values it produces are not very reliable.
Subscribing to the IMU topic provides no odometry.

### Running the ROSBag file 
ROSBag is ROS's way of recording and storing data in a compressed form. You can create a ROSBag of sensor/actuator message recordings while 
executing your robot's manoeuvres and collect all sensor/actuator messages recordings. Replaying this bag will be similar to running the real robot, and multiple iterations 
of trial and error coding can be made.
(Remember to run roscore on a seperate terminal.)
First create a virtual static transformation, this will be helpful in plotting the data
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 0 odom base_footprint 10
```
Now, run the ROSBag, which was explained in the previous assignment, and pause it
```
rosbag play ./data/tbot_lidar_loop.bag
```
Subscribe for the data using a simple ROS subscriber. Use code contained in ./code :
The code is well commented.

## Assignment
1. Plot live /odom and /odom_rf2o on rViz. (Instructions for plotting data on rViz can be found online and is part of the assignment.)
2. Use the utilities provided by ROS to obtain a time freeze (complete data) from the ROSBag and plot the same using matplotlib.
3. Considering the above, perform dead reckoning, i.e., 2-D position and pose (x,y,$\theta$) estimation, using just the IMU data. Compare with IMU (use matplotlib).

## Submission Format
1. Folder name should be assignment_1_<your_name> and should contain folders /code /figures and report.pdf.
2. Put your code in /code and the figures should be saved in /figures.
3. Make a simple one-page report and store the pdf as report.pdf.
4. Submissions should be made on Piazza. Send the zip file of the assignment folder.

