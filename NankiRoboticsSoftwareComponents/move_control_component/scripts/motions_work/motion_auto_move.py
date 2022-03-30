#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from motion_control_define import MotionControlDefine
from motion_control_define import MotionResult
from threading import Thread

"""
ROSアクションにおけるmove_baseでの移動用クラス
要求座標と向きをゴールとした自律移動を制御する

name_spaceの指定例：/robot001
"""
class MotionAutoMove():
  def __init__(self, goal, name_space, callback_motion_end=None):
    rospy.loginfo('自律移動用クラスコンストラクタ開始')
    # ゴール座標保存
    self.move_goal = goal
    # アクション用クライアントオブジェクト初期化
    self.action_move_client = None
    # 名前空間保存
    self.name_space = name_space
    # 移動完了結果通知コールバック関数保存
    self.callback_motion_end = callback_motion_end
    # 自律移動制御用スレッド開始
    self.th_move = Thread(target=self.action_move)
    self.th_move.start()

  # 自律移動制御用スレッド関数
  # 本スレッドは起動されると保存されているゴール座標をもって自律移動制御を開始する
  def action_move(self):
    rospy.loginfo('自律移動用スレッド開始')

    # アクション用ROSクライアントノード作成
    rospy.loginfo('アクション用ROSクライアントノード作成')
    topic_name = self.name_space + '/move_base'
    rospy.loginfo('アクション用トピック名：{}'.format(topic_name))
    self.action_move_client = actionlib.SimpleActionClient(topic_name, MoveBaseAction)

    # ロボット側に接続できるまで待ち
    rospy.logdebug('MotionAutoMove:サーバ接続待ち')
    self.action_move_client.wait_for_server()
    rospy.logdebug('MotionAutoMove:サーバ接続完了')
    rospy.loginfo('ロボット側への接続完了')

    # move_baseのゴール情報を設定
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp =  rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'

    goal.target_pose.pose.position.x = self.move_goal.goal_x
    goal.target_pose.pose.position.y = self.move_goal.goal_y
    orientation = tf.transformations.quaternion_from_euler(0, 0, self.move_goal.goal_rpy_yaw)
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    # サーバへゴール情報送信
    # フィードバックのコールバック関数も指定
    rospy.loginfo('MotionAutoMove:ゴール送信')
    rospy.logdebug('MotionAutoMove:ゴール情報 {}'.format(goal))
    self.action_move_client.send_goal(goal)

    # サーバへのゴール結果待ち
    rospy.loginfo('MotionAutoMove:結果待ち開始')
    result = self.action_move_client.wait_for_result()
    rospy.loginfo('MotionAutoMove:移動完了にて復帰 結果:{}'.format(result))

    rospy.loginfo('MotionAutoMove:アクション状態取得')
    status = self.action_move_client.get_state()
    rospy.loginfo('MotionAutoMove:アクション状態 {}'.format(status))

    # statusから結果を載せ替え
    if status == actionlib.GoalStatus.SUCCEEDED:
      result = MotionResult.MOTION_RESULT_OK
    else:
      result = MotionResult.MOTION_RESULT_NG

    # 結果通知コールバック関数が登録されていれば呼び出し
    # detailは将来的なもので現時点では未使用
    detail = ''
    if self.callback_motion_end is not None:
      self.callback_motion_end(result, detail)

    rospy.loginfo('自律移動用スレッド終了')

  # モーション中止指示受信
  def motion_cancel(self):
    rospy.loginfo('MotionAutoMove:サーバへキャンセル送信')
    self.action_move_client.cancel_goal()
