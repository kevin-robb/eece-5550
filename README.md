# eece-5550
Code for EECE-5550 Mobile Robotics at NEU.

Bulk of simulation was cloned from other repositories using [these instructions](https://github.com/tkelestemur/turtlebot3_mr). I've included these directories in the .gitignore to exclude them from remote storage.

The gazebo simulation can be launched with
`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
or
`roslaunch turtlebot3_mr turtlebot3_lab2.launch`

The sensor visualization tool Rviz can be opened with
`rosrun rviz rviz -d config/rviz.rviz`

We can command the turtlebot to drive in a 1.5m radius circle CCW with the terminal command
`rostopic pub /cmd_vel geometry_msgs/Twist -r 10 '[0.2, 0, 0]' '[0, 0, 0.1333]'`
