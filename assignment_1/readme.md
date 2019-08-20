# Use the given ROSBag file for a turtlebot executing a loop to obtain its 2D odometry

Note: Please use python2 for this exercise. You will additionally
Link: https://github.com/rbccps-iisc/cp313/tree/master/assignment_1


## Instruction
The ROSBag (contained in ./data) contains the following essential ROS topic sensor streams which also can be
found by running the command rostopic list:
1. /odom Wheel odometry (2D (x,y,𝜃))
2. /odom_rf2o Lidar odometry (2D (x,y,𝜃))
3. /imu 9DOF imu data (ax,ay,az,𝜃𝑥̇ ,𝜃𝑦̇ ,𝜃𝑧̇ ,𝜃𝑥,𝜃𝑦,𝜃𝑧)

You can perform rostopic echo <topic_name> to display the sensor output
Wheel odometry is the most accurate amongst these for the scenario in which the data was
generated. Lidar odometry tends to be noisy and the values it produces are not very reliable.
Subscribing to the IMU topic provides no odometry.
First, run the ROSBag, which was explained in the previous assignment, and pause it
(remember to run roscore as well).
Subscribe for the data using a simple ROS subscriber using code contained in ./code :
The code is well commented

## Assignment
1. Plot live /odom and /odom_rf2o on rViz (Instructions for plotting data on rViz can be found online and is part of the assignment)
2. Use the utilities provided by ROS to obtain a time freeze (complete data) from the ROSBag and plot the same using matplotlib
3. Considering the above, perform dead reckoning, i.e., 2-D position and pose (x,y,𝜃) estimation, using just the IMU data. Compare with IMU (use matplotlib)

## Submission Format
1. Folder name should be assignment_1_<your_name> and should contain folders /code /figures and report.pdf
2. Put your code in /code and the figures should be saved in /figures 
3. Make a simple 1 page report and store the pdf as report.pdf
4. Submissions should be made on Piazza. Send the zip file of the assignment folder

