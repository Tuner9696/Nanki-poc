#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Test Program ：ROS Topic Publisher
# command : rosrun temi_trans_control Test_pub_once.py

import time
import rospy
from mission.msg import ManagerCommandRes, JointManagerCommandRes, TransManagerCommandReq, JointManagerCommandReq


class PoseChange:
    def __init__(self):
        self.msg = ManagerCommandRes()
        self.msg2 = JointManagerCommandRes()
        rospy.init_node("test_publisher", anonymous=True)
        self.sub = rospy.Subscriber('/trsmng_req', TransManagerCommandReq, self.move_req_callback)
        self.sub2 = rospy.Subscriber('/jointmng_req', JointManagerCommandReq, self.lifter_req_callback)
        self.pub = rospy.Publisher('/trsmng_res', ManagerCommandRes, queue_size=1)
        self.pub2 = rospy.Publisher('/jointmng_res', JointManagerCommandRes, queue_size=1)

    def move_req_callback(self, msg):
        rospy.loginfo('trsmng_req received!!!!!') 
        time.sleep(7)
        # 移動応答
        self.msg.robot_id = msg.robot_id
        self.msg.result = 1
        self.pub.publish(self.msg)

    def lifter_req_callback(self, msg):
        rospy.loginfo('jointmng_req received!!!!!')
        time.sleep(5)
        # リフター応答
        self.msg2.robot_id = msg.robot_id
        self.msg2.result = 1
        self.pub2.publish(self.msg2)

if __name__ == "__main__":
    try:
        m = PoseChange()
        m = PoseChange()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            connections = m.pub.get_num_connections()
            rate.sleep()

    except rospy.ROSInterruptException, e:
        raise e




