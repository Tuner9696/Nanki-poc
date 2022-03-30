#!/usr/bin/env python
# coding: UTF-8

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    rospy.init_node('thk002_initialpose')

    pose_pub = rospy.Publisher('thk002/initialpose', PoseWithCovarianceStamped, queue_size = 10)

    pose_test = PoseWithCovarianceStamped()
    pose_test.header.frame_id = 'map'
    pose_test.pose.pose.position.x = -16.68
    pose_test.pose.pose.position.y = -2.62
    pose_test.pose.pose.orientation.z = 0.96
    pose_test.pose.pose.orientation.w = 0.27


    while not rospy.is_shutdown():

        pose_pub.publish(pose_test)

        rospy.sleep(0.2)