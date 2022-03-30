#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from iothub_client.msg import remote_mode
# bool remote_control_mode

from iothub_client.msg import remote_move

def callback(msg):
    rospy.loginfo("Message '{}' recieved".format(msg))


def subscriber():
    # ノードを初期化する。
    rospy.init_node("subscriber")

    # 受信者を作成する。
    rospy.Subscriber("remote_mode", remote_mode, callback)

    # ノードが終了するまで待機する。
    rospy.spin()


if __name__ == "__main__":
    subscriber()