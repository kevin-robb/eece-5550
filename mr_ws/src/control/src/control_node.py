#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.linalg import expm, logm

control_pub = None
# constants
r = 0.033 # wheel radius
w = 0.16 # chassis width
# goal pose
goal_pose = None

def timer_callback(event):
    # publish velocity cmds to follow the trajectory,
    # and halt when it arrives at the target pose.
    pass

def drive_in_circle():
    cmd = Twist()
    cmd.linear.x = 0.2 #dx
    cmd.angular.z = 0.1333 #dtheta
    control_pub.publish(cmd)

def tag_detect(tag):
    global goal_pose
    # get a pose from the detected tag
    goal_pose = tag.detections[0].pose.pose.pose
    #position = [pose_msg.position.x,pose_msg.position.y,pose_msg.position.z]
    #orientation = pose_msg.orientation #x,y,z,w

def calculate_trajectory(goal_pose):
    # calculate a trajectory to drive to the goal pose
    pass


def main():
    global control_pub
    rospy.init_node('control_node')
    # publish the command messages
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subscribers
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
