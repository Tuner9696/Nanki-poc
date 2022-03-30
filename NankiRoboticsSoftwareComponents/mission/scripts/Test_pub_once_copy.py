#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from mission.msg import TransManagerCommandReq, JointManagerCommandReq

class PoseChange:
    def __init__(self):
        self.msg = TransManagerCommandReq()
        self.msg_l = JointManagerCommandReq()
        rospy.init_node("test_publisher", anonymous=True)
        self.pub = rospy.Publisher('/trsmng_req', TransManagerCommandReq, queue_size=1)
        self.pub_l = rospy.Publisher('/jointmng_req', JointManagerCommandReq, queue_size=1)

    def run(self):
        self.msg.robot_id = 1
        self.msg.trans_kind = 1
        self.msg.goal.trs_flg = 1
        self.msg.goal.point_info.transform.translation.x =  -0.458177439061 
        self.msg.goal.point_info.transform.translation.y = -1.54988048016
        self.msg.goal.point_info.transform.translation.z = 0.0
        self.msg.goal.point_info.transform.rotation.ori_p = 0.0
        self.msg.goal.point_info.transform.rotation.ori_r = 0.0
        self.msg.goal.point_info.transform.rotation.ori_y = 0.0
        #self.pub.publish(self.msg)
        self.msg_l.robot_id = 1
        self.msg_l.lifter_control_kind = 1
        self.pub_l.publish(self.msg_l)

        time.sleep(10)

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

    except rospy.ROSInterruptException, e:
        raise e




