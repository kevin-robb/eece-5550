#!/bin/bash
source mr_ws/devel/setup.bash
cd mr_ws
# launch gazebo for lab 2
roslaunch turtlebot3_mr turtlebot3_lab2.launch &> /dev/null &
cd src/turtlebot3_mr
sleep 3s
# after gazebo starts, launch rviz
rosrun rviz rviz -d config/rviz.rviz &> /dev/null &
cd ../..
sleep 2s
# init apriltag
roslaunch turtlebot3_mr apriltag_gazebo.launch &> /dev/null &

# tell user to run control node
echo "USE 'rosrun control control_node.py' WHEN READY."
