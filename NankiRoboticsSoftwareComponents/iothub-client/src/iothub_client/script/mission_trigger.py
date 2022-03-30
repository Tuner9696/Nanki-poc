#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROSライブラリーインポート
import rospy
# ROSメッセージインポート
from iothub_client.msg import mission_message 

# ミッション開始制御クラス
class MisiionTrigger():
  def __init__(self):
    # Publisherを作成
    self.publisher = rospy.Publisher('/exe_mission', mission_message, queue_size=1)

    # messageの型を作成
    self.message = mission_message()

    # メッセージ送信関数
  def send_msg(self,trigger):
    # messageを送信
    self.message.command = trigger

    # Publish
    self.publisher.publish(self.message)
    # ログ出力
    rospy.loginfo(self.message)
        





