#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROSライブラリーインポート
import rospy
# ROSメッセージインポート
from iothub_client.msg import guide

# ミッション開始制御クラス
class GuideTrigger():
  def __init__(self):
    # Publisherを作成
    self.publisher6 = rospy.Publisher('/ext_destination', guide, queue_size=1)

    # messageの型を作成
    self.message = guide()

    # メッセージ送信関数
  def send_msg(self,point):
    # messageを送信
    self.message.destination_no = int(point)
    # Publish
    self.publisher6.publish(self.message)
    print("GGG")
    # ログ出力
    rospy.loginfo(self.message)
        

