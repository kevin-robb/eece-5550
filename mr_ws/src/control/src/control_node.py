#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.linalg import logm, inv
from math import sin, cos
from tf.transformations import euler_from_quaternion
from time import time, sleep

control_pub = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
# Poses as homogenous matrices:
goal_pose = None

# get a pose from the detected tag
def tag_detect(tag_msg):
    global goal_pose
    if goal_pose is not None:
        # only get it once
        return
    try:
        tag_pose = tag_msg.detections[0].pose.pose.pose
        # transform to robot's coordinate frame & calculate goal pose
        #  - tag's z position corresponds to distance between them (robot x)
        #  - want to be 12cm in front of the tag, directly facing it
        goal_pose = make_affine(theta=0,x=tag_pose.position.z-0.12,y=0)

        # now that we have the goal pose, go to it
        go_to_goal(goal_pose,5)
    except:
        # tag not detected
        return

# extract yaw from quaternion in pose
def yaw_from_pose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    theta = euler_from_quaternion(quaternion)[2] # (roll,pitch,yaw)
    return theta

# create a homogenous affine matrix from a pose msg or components
def make_affine(pose=None,theta=None,x=None,y=None):
    if pose is None:
        return np.array([[cos(theta), -sin(theta), x], [sin(theta), cos(theta), y], [0,0,1]])
    else:
        return make_affine(theta=yaw_from_pose(pose), x=pose.position.x, y=pose.position.y)

# calculate a trajectory to drive to the goal pose
def calculate_trajectory(cur_pose,goal_pose, T):
    # calculate \dot\Omega using inverse kinematics
    dotOmega = logm(inv(cur_pose) @ goal_pose) / T
    # extract necessary speed commands
    cmd = Twist()
    cmd.linear.x = dotOmega[0][2] * (1.25/0.7) # tuning param
    cmd.angular.z = dotOmega[1][0]
    control_pub.publish(cmd)

# drive to the goal pose for a certain amount of time, then halt
def go_to_goal(goal_pose, T):
    # starting pose is origin but offset from camera's frame slightly
    # NOTE there should be a -x offset due to this, but I couldn't find what it should be,
    #   so the robot ends slightly further from the apriltag than it should
    cur_pose = make_affine(theta=0,x=0,y=0)
    print("initial pose:\n",cur_pose)
    print("goal pose\n",goal_pose)
    start_time = time()
    calculate_trajectory(cur_pose,goal_pose, T)
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
