#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from scipy.linalg import logm, inv
from time import sleep
# from tf import TransformListener, lookupTransform
import tf
from scipy.spatial.transform import Rotation as R

control_pub = None
tf_listener = None
# Constants:
r = 0.033 # wheel radius (m)
w = 0.16 # chassis width (m)
# Transforms as homogenous matrices
T_CB = None # base->cam
T_AC = None # cam->tag
T_GA = None # tag->goal
# Poses as homogenous matrices:
cur_pose = np.array([[1,0,0],[0,1,0],[0,0,1]])
# goal_pose = None
# flags
done = False

# get the cam->tag transform
def tag_detect(tag_msg):
    global T_AC
    if T_AC is not None:
        # only get it once
        return
    try:
        tag_pose = tag_msg.detections[0].pose.pose.pose
        # use this to make goal pose in robot base frame
        t = [tag_pose.position.x,tag_pose.position.y,tag_pose.position.z]
        q = [tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z]
        # make it into an affine matrix
        r = R.from_quat(q).as_matrix()
        # make affine matrix for transformation
        T_AC = np.array([[r[0][0],r[0][1],r[0][2],t[0]],
                        [r[1][0],r[1][1],r[1][2],t[1]],
                        [r[2][0],r[2][1],r[2][2],t[2]],
                        [0,0,0,1]])
    except:
        # tag not detected
        return

# calculate a trajectory to drive to the goal pose
def calculate_trajectory(goal_pose, T):
    # calculate \dot\Omega using inverse kinematics
    # because cur_pose=I, inv(cur_pose)@goal_pose = goal_pose
    dotOmega = (1/T) * logm(goal_pose)
    # extract necessary speed commands
    cmd = Twist()
    cmd.linear.x = dotOmega[0][2]
    cmd.angular.z = dotOmega[1][0]
    control_pub.publish(cmd)

# drive to the goal pose for a certain amount of time, then halt
def go_to_goal(goal_pose, T):
    print("goal pose:\n",goal_pose)
    calculate_trajectory(goal_pose, T)
    sleep(T)
    cmd = Twist() # send blank command of zeros
    control_pub.publish(cmd)

def get_T_CB():
    global T_CB
    # make sure the listener has time to initialize
    sleep(2.5)
    # get cam relative to base from the service
    (t,q) = tf_listener.lookupTransform('/base_footprint', '/camera_rgb_optical_frame', rospy.Time(0))
    # get equiv rotation matrix from quaternion
    r = R.from_quat(q).as_matrix()
    # make affine matrix for transformation base->cam
    T_CB = np.array([[r[0][0],r[0][1],r[0][2],t[0]],
                    [r[1][0],r[1][1],r[1][2],t[1]],
                    [r[2][0],r[2][1],r[2][2],t[2]],
                    [0,0,0,1]])
def get_T_GA():
    global T_GA
    # we know the goal pose is 0.12 m away along z_tag,
    # and z_tag -> -x_goal,
    #     y_tag -> z_goal,
    #     x_tag -> -y_goal.
    # we can represent this as first a translation, and then a rotation
    T_translation = np.array([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0.12],
                              [0,0,0,1]])
    T_rotation = np.array([[0,0,-1,0],
                           [-1,0,0,0],
                           [0,1,0,0],
                           [0,0,0,1]])
    T_GA = np.matmul(T_rotation, T_translation)

def timer_callback(event):
    global done
    if done: return
    if T_CB is not None and T_AC is not None and T_GA is not None:
        done = True
        # all three transforms have been set
        g = np.matmul(np.matmul(T_GA,T_AC),T_CB)
        # strip out z parts to make it in SE(2) instead of SE(3)
        goal_pose = np.array([[g[0][0],g[0][1],g[0][3]],
                             [g[1][0],g[1][1],g[1][3]],
                             [0,0,1]])
        print_stuff(goal_pose)
        go_to_goal(goal_pose,5)

def print_stuff(goal_pose):
    print("T_CB:\n",T_CB)
    print("T_AC:\n",T_AC)
    print("T_GA:\n",T_GA)
    print("goal_pose:\n",goal_pose)

def main():
    global control_pub, tf_listener
    rospy.init_node('control_node')
    # get the TF from the service
    tf_listener = tf.TransformListener()
    # set static transforms
    get_T_CB()
    get_T_GA()
    # publish the command messages
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subscribers
    rospy.Subscriber("/tag_detections",AprilTagDetectionArray,tag_detect,queue_size=1)
    # make a 10Hz timer
    rospy.Timer(rospy.Duration(0.1), timer_callback)
    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
