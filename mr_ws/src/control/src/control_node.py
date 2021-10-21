#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.linalg import expm, logm, inv
from math import sin, cos
from tf.transformations import euler_from_quaternion

control_pub = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
T = 3 # total time to reach goal pose (s)
# Poses as homogenous matrices:
cur_pose = None
goal_pose = None

# drive in 1.5m radius circle, as in lab 2, problem 2
def drive_in_circle():
    cmd = Twist()
    cmd.linear.x = 0.2 #dx
    cmd.angular.z = 0.1333 #dtheta
    control_pub.publish(cmd)

# get robot's current pose from Odom
def get_robot_position(msg):
    global cur_pose
    cur_pose = make_affine(pose=msg.pose.pose)

# get a pose from the detected tag
def tag_detect(tag_msg):
    global goal_pose
    if goal_pose is not None:
        # only get it once
        return
    try:
        tag_pose = tag_msg.detections[0].pose.pose.pose
        # calculate goal pose:
        #  - 12cm in front of the tag (+z)
        #  - robot x-axis directly facing it (anti-parallel with its z-axis)
        # transform to robot's coordinate frame
        #  - tag's z position corresponds to distance between them (robot x)
        goal_pose = make_affine(theta=0,x=tag_pose.position.z-0.12,y=0)
    except:
        # tag not detected, keep using last measurement
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

# create a homogenous affine matrix from a pose msg
def make_affine(pose=None,theta=None,x=None,y=None):
    if pose is None:
        return np.array([[cos(theta), -sin(theta), x], [sin(theta), cos(theta), y], [0,0,1]])
    else:
        return make_affine(theta=yaw_from_pose(pose), x=pose.position.x, y=pose.position.y)

# calculate a trajectory to drive to the goal pose
def calculate_trajectory(goal_pose):
    # calculate \dot\Omega using inverse kinematics
    dotOmega = logm(inv(cur_pose) @ goal_pose) / T
    # extract necessary speed commands
    cmd = Twist()
    cmd.linear.x = dotOmega[0][2]
    cmd.angular.z = dotOmega[0][1]
    control_pub.publish(cmd)

# action on every timestep
def timer_callback(event):
    # check how close we are to the desired pose
    if True: #TODO
        # publish velocity cmds to follow the trajectory,
        calculate_trajectory(goal_pose)
    else:
        # halt when it arrives at the target pose.
        cmd = Twist() # send blank command of zeros
        control_pub.publish(cmd)


def main():
    global control_pub, cur_pose
    rospy.init_node('control_node')
    # set current pose to identity
    # cur_pose = make_affine(theta=0,x=0,y=0)

    # publish the command messages
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subscribers
    rospy.Subscriber('/odom', Odometry, get_robot_position, queue_size=1)
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,tag_detect,queue_size=1)
    # init a 10Hz timer to call timer_callback()
    rospy.Timer(rospy.Duration(0.1), timer_callback)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
