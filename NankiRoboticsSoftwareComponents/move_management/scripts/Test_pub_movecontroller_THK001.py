#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
from move_management.msg import TransRes, TransReq, LifterControlRes, LifterControlReq

class PoseChange:
    def __init__(self):
        self.err_flag = True
        self.msg = TransRes()
        self.msg_l = LifterControlRes()
        rospy.init_node("move_lifter_controller", anonymous=True)
        self.sub = rospy.Subscriber('/thk001/trs_req', TransReq, self.move_req_callback)
        self.pub = rospy.Publisher('/thk001/trs_res', TransRes, queue_size=1)
        self.sub_l = rospy.Subscriber('/thk001/joint_ctl_req', LifterControlReq, self.lifter_req_callback)
        self.pub_l = rospy.Publisher('/thk001/lifter_control_res', LifterControlRes, queue_size=1)

    def move_req_callback(self, msg):
        rospy.loginfo('trs_req received!!!!!') 
        print(self.err_flag)
        time.sleep(3)
        #print(msg)
        if self.err_flag == False:
          if len(msg.route_ps_info) > 0:
            if msg.route_ps_info[0].point_info.transform.translation.x == -2 and  msg.route_ps_info[0].point_info.transform.translation.y == -3.5:
                _msg = TransRes()
                _msg.trans_seq = msg.trans_seq
                self.err_flag = True # 1回だけNG応答
                _msg.trs_result = 20   # リルート要求
                _msg.reroute_index = 1
                rospy.loginfo('trs_res NG send!!!!!') 
                self.pub.publish(_msg)
                return
        self.msg.trans_seq = msg.trans_seq
        self.msg.trs_result = 2  # OK
        self.pub.publish(self.msg)

    def lifter_req_callback(self, msg):
        rospy.loginfo('joint_ctl_req received!!!!!')
        #print(msg)
        time.sleep(3)
        if msg.lifter_control_kind == 1:
            self.msg_l.lifter_position = 1
        else:
            self.msg_l.lifter_position = 2
        self.msg_l.lifter_control_result = 1
        self.pub_l.publish(self.msg_l)

if __name__ == "__main__":
    try:
        m = PoseChange()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            connections = m.pub.get_num_connections()
            rate.sleep()

    except rospy.ROSInterruptException as e:
        raise e




