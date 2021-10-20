#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.linalg import expm, logm, inv
from math import sin, cos, acos
from tf.transformations import euler_from_quaternion

control_pub = None
# constants
r = 0.033 # wheel radius
w = 0.16 # chassis width
# poses as homogenous matrices
cur_pose = None
goal_pose = None

def drive_in_circle():
    cmd = Twist()
    cmd.linear.x = 0.2 #dx
    cmd.angular.z = 0.1333 #dtheta
    control_pub.publish(cmd)

# get robot's current pose from Odom
def get_robot_position(msg):
    global cur_pose
    cur_pose = make_affine(msg.pose.pose)

# get a pose from the detected tag
def tag_detect(tag_msg):
    global goal_pose
    tag_pose = tag_msg.detections[0].pose.pose.pose
    # calculate goal pose:
    #  - 12cm in front of the tag (+z)
    #  - robot x-axis directly facing it (anti-parallel with its z-axis)
    # transform to robot's coordinate frame?
    # TODO make goal pose

    

# extract yaw from quaternion in pose
def yaw_from_pose(pose):
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    theta = euler_from_quaternion(quaternion)[2] # (roll,pitch,yaw)
    #theta = 2*acos(pose.orientation.w)
    return theta

# create a homogenous affine matrix from a pose msg
def make_affine(pose):
    return make_affine(yaw_from_pose(pose), pose.position.x, pose.position.y)

def make_affine(theta,x,y):
    # return np.array([[cos(theta), -sin(theta), pose.position.x], [sin(theta), cos(theta), pose.position.y], [0,0,1]])
    return ([cos(theta), -sin(theta), x], [sin(theta), cos(theta), y], [0,0,1])

# calculate a trajectory to drive to the goal pose
def calculate_trajectory(goal_pose):
    # calculate \dot\Omega
    dotOmega = logm(inv(cur_pose) @ goal_pose)
    # extract necessary speed commands
    cmd = Twist()
    cmd.linear.x = dotOmega[0][2]
    cmd.angular.z = dotOmega[0][1]
    control_pub.publish(cmd)
    # dotPhi_r = dotOmega[0][2] / r - dotOmega[0][1] * w/(2*r)
    # dotPhi_l = dotOmega[0][2] / r + dotOmega[0][1] * w/(2*r)
    # # calculate Twist parameters from these
    # dx = (dotPhi_r + dotPhi_l)/2
    # dtheta = (dotPhi_r - dotPhi_l)

# action on every timestep
def timer_callback(event):
    # publish velocity cmds to follow the trajectory,
    calculate_trajectory(goal_pose)
    # and halt when it arrives at the target pose.
    pass


def main():
    global control_pub
    rospy.init_node('control_node')
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
