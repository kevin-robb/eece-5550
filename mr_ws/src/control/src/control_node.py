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
cur_pose = np.eye(3)
goal_pose = None

# get a pose from the detected tag
def tag_detect(tag_msg):
    try:
        tag_pose = tag_msg.detections[0].pose.pose.pose
        calculate_goal_pose(tag_pose)
        go_to_goal(4)
    except:
        # tag not detected, so need to rely on time to stop
        go_to_goal(5)
        return

# find expression for goal pose in robot's base frame
def calculate_goal_pose(tag_pose):
    global goal_pose
    T_CB = np.array([[1,0,0,0.07],
                    [0,-1,0,0],
                    [0,0,-1,0.2],
                    [0,0,0,1]])
    T_AC = make_transformation_matrix(tag_pose)
    T_GA = np.array([[0,-1,0,0],
                    [0,0,1,0],
                    [-1,0,0,0.12],
                    [0,0,0,1]])
    # gp = T_GA @ T_AC @ T_CB
    gp = np.inv(T_CB) @ np.inv(T_AC) @ np.inv(T_GA)
    # remove z dimension
    goal_pose = np.array([[gp[0][0],gp[0][1],gp[0][2]],
                        [gp[1][0],gp[1][1],gp[1][2]],
                        [gp[2][0],gp[2][1],gp[2][2]]])


# create a homogenous affine matrix from a pose
def make_transformation_matrix(pose):
    # use the quaternion to construct the rotation part
    q0 = pose.orientation.w
    q1 = pose.orientation.x
    q2 = pose.orientation.y
    q3 = pose.orientation.z
    
    return np.array([[2*(q0*q0+q1*q1)-1,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2),pose.position.x],
                    [2*(q1*q2+q0*q3),2*(q0*q0+q2*q2)-1,2*(q2*q3-q0*q1),pose.position.y],
                    [2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),2*(q0*q0+q3*q3)-1,pose.position.z],
                    [0,0,0,1]])
                            

# calculate a trajectory to drive to the goal pose
def calculate_trajectory(T):
    cur_pose = np.eye(3)
    # calculate \dot\Omega using inverse kinematics
    dotOmega = (1/T)*logm(inv(cur_pose) @ goal_pose)
    # dotOmega = (1/T)*logm(np.matmul(inv(cur_pose),goal_pose))
    print("dotOmega:\n",dotOmega)
    # extract necessary speed commands
    cmd = Twist()
    cmd.linear.x = dotOmega[0][2]
    cmd.angular.z = dotOmega[1][0]
    control_pub.publish(cmd)

# drive to the goal pose for a certain amount of time, then halt
def go_to_goal(T):
    print("initial pose:\n",cur_pose)
    print("goal pose:\n",goal_pose)
    start_time = time()
    calculate_trajectory(T)
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
