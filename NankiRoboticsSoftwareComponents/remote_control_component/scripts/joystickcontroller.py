#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from button_mapping import ButtonMap as bm
from button_mapping import AxesMap as am
from threading import Thread
from remote_control_component.msg import ControllerEvent

"""
コントローラ操作監視コンポーネントのメインクラス
Joyからの操作イベントをフックし
その情報をPublishする
"""
class JoyStickDispatcher():
  # コンストラクタ
  def __init__(self):
    rospy.loginfo('JoyStickDispatcherクラス開始')

    # コントローラ操作フック情報サブスクライバー作成
    rospy.loginfo('コントローラ操作フック情報Publisher作成')
    self.pub_ext_controller = rospy.Publisher('/ext_joycontroller', ControllerEvent, queue_size=10)

  # Joyからの操作イベントコールバック関数
  def on_joydata(self, joy_data):
    # コントローラ操作イベント情報作成
    msg = ControllerEvent()
    msg.button_cross = joy_data.buttons[bm.BUTTON_CROSS]
    msg.button_circle = joy_data.buttons[bm.BUTTON_CIRCLE]
    msg.button_triangle = joy_data.buttons[bm.BUTTON_TRIANGLE]
    msg.button_square = joy_data.buttons[bm.BUTTON_SQUARE]
    msg.button_l1 = joy_data.buttons[bm.BUTTON_L1]
    msg.button_r1 = joy_data.buttons[bm.BUTTON_R1]
    msg.button_l2 = joy_data.buttons[bm.BUTTON_L2]
    msg.button_r2 = joy_data.buttons[bm.BUTTON_R2]
    msg.button_share = joy_data.buttons[bm.BUTTON_SHARE]
    msg.button_option = joy_data.buttons[bm.BUTTON_OPTION]
    msg.button_ps = joy_data.buttons[bm.BUTTON_PS]
    msg.button_l3 = joy_data.buttons[bm.BUTTON_L3]
    msg.button_r3 = joy_data.buttons[bm.BUTTON_R3]

    msg.axes_lstick_angular = joy_data.axes[am.AXES_LSTICK_ANGULAR]
    msg.axes_lstick_fb = joy_data.axes[am.AXES_LSTICK_FB]
    msg.axes_l2 = joy_data.axes[am.AXES_L2]
    msg.axes_rstick_lr = joy_data.axes[am.AXES_RSTICK_LR]
    msg.axes_r2 = joy_data.axes[am.AXES_R2]

    # コントローラ操作イベントPublish
    rospy.loginfo('キー情報Publish')
    self.pub_ext_controller.publish(msg)
    rospy.logdebug('操作イベント情報:{}'.format(msg))

  # コントローラ操作イベントサブスクライバー制御ワーカー
  def sub_worker_joy(self):
    rospy.Subscriber('joy', Joy, self.on_joydata)
    while not rospy.is_shutdown():
      rospy.spin()
