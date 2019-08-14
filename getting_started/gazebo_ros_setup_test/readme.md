Ensure that ros and gazebo are installed.
This code controls a simulated turtlebot on gazebo.

cd to the catkin_ws you created, ( you should see devel, build and src directories here)
Assuming that your catkin_ws is ~/catkin_ws

Source the ws environment variables
` source devel/setup.bash `

Ensure TURTLEBOT3_MODEL is initialized to "burger"
` export TURTLEBOT3_MODEL=${TB3_MODEL}`


Run the gazebo simulator with the turtlebot initialized
`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

Now on a seperate terminal you may run this python script
` python tbot_nav.py `


