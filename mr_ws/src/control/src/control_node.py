#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.linalg import logm, inv
from math import sin, cos, acos, asin, atan2
from time import time, sleep

control_pub = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
# Poses as homogenous matrices:
cur_pose = np.array([[1,0,0],[0,1,0],[0,0,1]])
goal_pose = None
# Transformation parameters:
tag_to_goal = 0.12 # distance to goal pose from apriltag when projected to x-y-plane
base_to_cam = 0.07 # distance between origin of base and camera frames when projected to x-y-plane

# get a pose from the detected tag
def tag_detect(tag_msg):
    global goal_pose
    if goal_pose is not None:
        # only get it once
        return
    try:
        tag_pose = tag_msg.detections[0].pose.pose.pose
        print(tag_pose)
        # use this to make goal pose in robot base frame
        q0 = tag_pose.orientation.w
        q1 = tag_pose.orientation.x
        q2 = tag_pose.orientation.y
        q3 = tag_pose.orientation.z
        theta = asin(2*(q0*q2-q3*q1)) # equivalent of yaw in robot base frame
        goal_pose = np.array([[cos(theta), -sin(theta), tag_pose.position.z-tag_to_goal+base_to_cam], [sin(theta), cos(theta), tag_pose.position.y], [0,0,1]])
        print(goal_pose)

        # now that we have the goal pose, go to it
        go_to_goal(goal_pose,5)
    except:
        # tag not detected
        return

# calculate a trajectory to drive to the goal pose
def calculate_trajectory(goal_pose, T):
    # calculate \dot\Omega using inverse kinematics
    dotOmega = logm(inv(cur_pose) @ goal_pose) / T
    # extract necessary speed commands
    cmd = Twist()
    cmd.linear.x = dotOmega[0][2]
    cmd.angular.z = dotOmega[1][0]
    control_pub.publish(cmd)

# drive to the goal pose for a certain amount of time, then halt
def go_to_goal(goal_pose, T):
    print("goal pose:\n",goal_pose)
    start_time = time()
    calculate_trajectory(goal_pose, T)
    while (time()-start_time < T):
        sleep(0.05)
    cmd = Twist() # send blank command of zeros
    control_pub.publish(cmd)


def main():
    global control_pub
    rospy.init_node('control_node')
    # publish the command messages
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subscribers
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,tag_detect,queue_size=1)
    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
