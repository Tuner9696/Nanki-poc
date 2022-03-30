#!/usr/bin/env python
# coding: UTF-8

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    rospy.init_node('thk001_initialpose')

    pose_pub = rospy.Publisher('thk001/initialpose', PoseWithCovarianceStamped, queue_size = 10)

    pose_test = PoseWithCovarianceStamped()
    pose_test.header.frame_id = 'map'
    pose_test.pose.pose.position.x = -10.63
    pose_test.pose.pose.position.y = 11.11
    pose_test.pose.pose.orientation.z = -0.31
    pose_test.pose.pose.orientation.w = 1

    while not rospy.is_shutdown():

        pose_pub.publish(pose_test)

        rospy.sleep(0.2)