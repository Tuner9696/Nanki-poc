#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from remote_controller_define import RemoteControlDefine
from threading import Thread

"""
ROSアクションにおけるmove_baseでの移動用クラス
要求座標と向きをゴールとした自律移動を制御する
"""
class AutoActionMove():
  def __init__(self, goal, callback_end=None):
    rospy.loginfo('自律移動用クラスコンストラクタ開始')
    # ゴール座標保存
    self._move_goal = goal

    # アクション用クライアントオブジェクト初期化
    self._action_move_client = None

    # 移動完了結果通知コールバック関数保存
    self._callback_end = callback_end

    # 自律移動制御用スレッド開始
    self.th_move = Thread(target=self.action_move)
    self.th_move.start()

  # 自律移動制御用スレッド関数
  # 本スレッドは起動されると保存されているゴール座標をもって自律移動制御を開始する
  def action_move(self):
    rospy.loginfo('自律移動用スレッド開始')

    # アクション用ROSクライアントノード作成
    rospy.loginfo('アクション用ROSクライアントノード作成')
    topic_name = RemoteControlDefine.NS_CONTROLL_ROBOT + '/move_base'
    self._action_move_client = actionlib.SimpleActionClient(topic_name, MoveBaseAction)

    # ロボット側に接続できるまで待ち
    rospy.logdebug('AutoActionMove:サーバ接続待ち')
    self._action_move_client.wait_for_server()
    rospy.logdebug('AutoActionMove:サーバ接続完了')
    rospy.loginfo('ロボット側への接続完了')

    # move_baseのゴール情報を設定
    goal = MoveBaseGoal()

    goal.target_pose.header.stamp =  rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'

    goal.target_pose.pose.position.x = self._move_goal.goal_position_x
    goal.target_pose.pose.position.y = self._move_goal.goal_position_y
    # ヨー(yaw)値を求める
    if self._move_goal.goal_position_degree == 0.0:
      yaw = 0.0
    elif self._move_goal.goal_position_degree <= 180.0:
      yaw = math.pi * (self._move_goal.goal_position_degree / 180) * -1.0
    else:
      yaw = math.pi * ((360 - self._move_goal.goal_position_degree) / 180)
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    # サーバへゴール情報送信
    # フィードバックのコールバック関数も指定
    rospy.loginfo('AutoActionMove:ゴール送信')
    rospy.loginfo('AutoActionMove:ゴール情報 {}'.format(goal))
    self._action_move_client.send_goal(goal)

    # サーバへのゴール結果待ち
    rospy.loginfo('AutoActionMove:結果待ち開始')
    result = self._action_move_client.wait_for_result()
    rospy.loginfo('AutoActionMove:移動完了にて復帰 結果:{}'.format(result))

    rospy.loginfo('AutoActionMove:アクション状態取得')
    status = self._action_move_client.get_state()
    rospy.loginfo('AutoActionMove:アクション状態 {}'.format(status))

    # 結果通知コールバック関数が登録されていれば呼び出し
    if self._callback_end is not None:
      self._callback_end(result, status)

    rospy.loginfo('自律移動用スレッド終了')

  # サーバへのキャンセル送信
  def action_cancel(self):
    rospy.loginfo('AutoActionMove:サーバへキャンセル送信')
    self._action_move_client.cancel_goal()
