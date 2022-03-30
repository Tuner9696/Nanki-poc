#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Test Program ・ｽFROS Topic Publisher
# command : rosrun temi_trans_control Test_pub_once.py

import time
import rospy
from mission.msg import DisplayStatus, GuideDestination, MissionCommand, RobotStaus


class PoseChange:
    def __init__(self):
        self.msg = MissionCommand()
        self.msg2 = GuideDestination()
        rospy.init_node("test_publisher", anonymous=True)
        self.pub = rospy.Publisher('ext_mission', MissionCommand, queue_size=1)
        self.pub2 = rospy.Publisher('/ext_destination', GuideDestination, queue_size=1)

    def run(self):
        self.msg.command = 1
        self.pub.publish(self.msg)

        time.sleep(10)

        self.msg.command = 2
        #self.pub.publish(self.msg)

        self.msg2.destination_no = 5
        #self.pub2.publish(self.msg2)

if __name__ == "__main__":
    try:
        m = PoseChange()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            connections = m.pub.get_num_connections()
            rospy.loginfo('Connections: %d', connections)
            if connections > 0:
                m.run()
                rospy.loginfo('Published')
                break
            rate.sleep()

    except rospy.ROSInterruptException as e:
        raise e




