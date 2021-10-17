#!/usr/bin/env python

import rospy
from geometry_msgs import Twist, Vector3

control_pub = None

def timer_callback(event):
    # publish velocity cmds to follow the trajectory,
    # and halt when it arrives at the target pose.
    lin = Vector3()
    lin.x = 0.2 #dx
    ang = Vector3()
    ang.z = 0.1333 #dtheta
    cmd = Twist()
    cmd.linear = lin
    cmd.angular = ang
    control_pub.publish(cmd)

def tag_detect(tags):
    # get a pose from the detected tag
    #TODO
    # calculate a trajectory to drive to the pose
    #TODO
    pass


def main():
    global control_pub
    rospy.init_node('control_node')
    # publish the command messages
    control_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # subscribers
    rospy.Subscriber("/tag_detections",AprilTags,tag_detect,queue_size=1)
    # init a 10Hz timer to call timer_callback()
    rospy.Timer(rospy.Duration(0.1), timer_callback)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
