#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from move_control.msg import TransReq, RouteSP, TransformStampedEuler

list_translation_x = [1, 2, 3]
list_translation_y = [1, 2, 3]
list_rotation_yaw = [0.785, 1.57, 3.14]


def main():
    # ノードを初期化する。
    rospy.init_node("pub_trs_req")

    # 送信者を作成する。
    pub = rospy.Publisher("/thk001/trs_req", TransReq, queue_size=10)
    
    count = 1#シーケンス用
    msg = TransReq()
    msg.trans_kind = 1
    msg.route_ps_num = len(list_translation_x)
    for i in range(3):
      msg.trans_seq = count
      msg_route = RouteSP()
      msg_route.trs_flg  = 1
      msg_route.point_info.transform.translation.x = list_translation_x[i]
      msg_route.point_info.transform.translation.y = list_translation_y[i]
      msg_route.point_info.transform.translation.z = 0.0
      msg_route.point_info.transform.rotation.ori_p = 0.0
      msg_route.point_info.transform.rotation.ori_r = 0.0
      msg_route.point_info.transform.rotation.ori_y = list_rotation_yaw[i]
      msg_route.whl_sp_info.linear.x = 0.2
      msg_route.whl_sp_info.linear.y = 0.0
      msg_route.whl_sp_info.linear.z = 0.0
      msg_route.whl_sp_info.angular.x = 0.0
      msg_route.whl_sp_info.angular.y = 0.0
      msg_route.whl_sp_info.angular.z = 0.2
      msg.route_ps_info.append(msg_route)
      count+=1
    
    rate = rospy.Rate(0.5)
    rate.sleep()
    pub.publish(msg)
    rate.sleep()
    

    rospy.loginfo("Message '{}' published".format(msg))

if __name__ == "__main__":
    main()