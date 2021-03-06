# eece-5550
Code for EECE-5550 Mobile Robotics at NEU.

Bulk of simulation was cloned from other repositories using [these instructions](https://github.com/tkelestemur/turtlebot3_mr). I've used git submodules in place of the `git clone` commands.

The gazebo simulation can be launched with
`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
or
`roslaunch turtlebot3_mr turtlebot3_lab2.launch`

The sensor visualization tool Rviz can be opened by first changing the working directory to `src/turtlebot3_mr` and then running 
`rosrun rviz rviz -d config/rviz.rviz`

We can run `roslaunch turtlebot3_mr apriltag_gazebo.launch` to publish the marker's pose to the `/tf` and `/tag_detections` topics.

We can command the turtlebot to drive in a 1.5m radius circle CCW with the terminal command
`rostopic pub /cmd_vel geometry_msgs/Twist -r 10 '[0.2, 0, 0]' '[0, 0, 0.1333]'`

My code for lab 2 can be executed by first launching the environments above and then calling `rosrun mobile_robotics control_node.py`, or everything can be launched together with `roslaunch mobile_robotics lab2.launch`.
