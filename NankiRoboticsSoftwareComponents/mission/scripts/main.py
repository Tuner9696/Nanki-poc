#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal
import time
import datetime
#from concurrent import futures
import rospy
import sys
import time
from threading import Event

from mission_manager import mission_factory

# アプリケーション終了イベント
event_apl_end = Event()

# Temi移動制御プロセスエントリー関数
def main():
  #global event_apl_end

  try:
    # ROSノード作成
    rospy.init_node('mission')
    rospy.loginfo('--- アプリ開始 ---')
    rospy.loginfo('メイン関数開始')

    # load parameters
    #params = rospy.get_param("~", {})
    #mission_params = params.pop("mission", {})
    #pub_topic = mission_params.pop("pub_topic")
    #sub_topic = mission_params.pop("sub_topic")

    # ミッション作成
    rospy.loginfo('ミッション作成')
    guide_mission = mission_factory()

    refresh_rate = rospy.Rate(0.33)

    while not rospy.is_shutdown():
        refresh_rate.sleep()

    #rospy.loginfo('アプリ終了イベント待ち')
    #event_apl_end.wait()

  # ROS系割り込み例外
  except rospy.ROSInterruptException:
    pass

# エントリー関数（main）呼び出し
if __name__ == "__main__":
  main()
