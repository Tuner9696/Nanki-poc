#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import degrees
# ROSライブラリーインポート
import rospy
# ROSメッセージインポート
from iothub_client.msg import remote_mode
# bool remote_control_mode

from iothub_client.msg import remote_move
# float32 goal_position_x
# float32 goal_position_y
# float32 goal_position_degree

# 遠隔制御中継クラス
class RemoteHub():
  def __init__(self):
    # Publisherを作成
    self.publisher1 = rospy.Publisher('/remote_mode', remote_mode, queue_size=1)
    self.publisher2 = rospy.Publisher('/thk001/remote_move', remote_move, queue_size=1)

    # messageの型を作成
    self.remote_mode_message = remote_mode()
    self.remote_move_message = remote_move()

  # 遠隔制御切り替え関数
  def remote_mode(self,trigger):
      self.remote_mode_message.remote_control_mode = trigger
      # Publish
      self.publisher1.publish(self.remote_mode_message)
      rospy.loginfo(self.remote_mode_message)

   # 遠隔制御移動の座標をPublish
  def remote_move(self,x,y,degree):

    self.remote_move_message.goal_position_x = float(x)
    self.remote_move_message.goal_position_y = float(y)
    self.remote_move_message.goal_position_degree = float(degree) 
    # Publish
    self.publisher2.publish(self.remote_move_message)
    rospy.loginfo(self.remote_move_message)




        
        
        

        
        


