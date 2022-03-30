#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

class PoseChange:
    def __init__(self):
        self.msg1 = JointTrajectoryControllerState()
        self.msg2 = JointTrajectoryControllerState()
        rospy.init_node("test_lifter", anonymous=True)
        self.sub1 = rospy.Subscriber('/thk001/lifter_controller/command', JointTrajectory, self.lifter_req_callback1)
        self.sub2 = rospy.Subscriber('/thk002/lifter_controller/command', JointTrajectory, self.lifter_req_callback2)
        self.pub1 = rospy.Publisher('/thk001/lifter_controller/state', JointTrajectoryControllerState, queue_size=1)
        self.pub2 = rospy.Publisher('/thk002/lifter_controller/state', JointTrajectoryControllerState, queue_size=1)

    def lifter_req_callback1(self, msg):
        rospy.loginfo('joint_ctl_req received!!!!!')
        #print(msg)
        self.msg1.actual.positions = []
        self.msg1.joint_names = ["knee_joint", "ankle_joint"]
        self.msg1.actual.positions.append(msg.points[0].positions[1])
        self.msg1.actual.positions.append(msg.points[0].positions[0])
        #self.msg1.actual.positions.append(v)
        self.pub1.publish(self.msg1)
        time.sleep(1)

    def lifter_req_callback2(self, msg):
        rospy.loginfo('joint_ctl_req received!!!!!')
        #print(msg)
        self.msg2.actual.positions = []
        self.msg2.joint_names = ["knee_joint", "ankle_joint"]
        self.msg2.actual.positions.append(msg.points[0].positions[1])
        self.msg2.actual.positions.append(msg.points[0].positions[0])
        self.pub2.publish(self.msg2)
        time.sleep(1)

if __name__ == "__main__":
    try:
        m = PoseChange()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
        """
        while not rospy.is_shutdown():
            connections = m.pub1.get_num_connections()
            rate.sleep()
        """

    except rospy.ROSInterruptException as e:
        raise e




