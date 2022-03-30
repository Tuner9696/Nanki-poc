#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import math
import numpy
from threading import Thread
from geometry_msgs.msg import Twist
from motion_control_define import MotionControlDefine
from motion_control_define import MotionResult

"""
ROSのcmd_velによる手動転回用クラス
現在座標、向きを元に方向転換を制御する
"""
class MotionManualRotate():
  def __init__(self, goal, name_space, callback_motion_end=None):
    rospy.loginfo('手動転回用クラスコンストラクタ開始')
    # ゴール座標保存
    self.move_goal = goal
    # 名前空間保存
    self.name_space = name_space
    # 転回完了結果通知コールバック関数保存
    self.callback_motion_end = callback_motion_end
    # モーション中止フラグ初期化
    self.flag_motion_cancel = False
    # 速度指定Publisher生成
    self.pub_cmd_vel = rospy.Publisher(self.name_space+'/cmd_vel', Twist, queue_size=10)

    # 手動転回制御用スレッド開始
    self.th_move = Thread(target=self.manual_rotate)
    self.th_move.start()

  """
  手動転回制御用スレッド関数

  いくつかのステップを実施(StateMachineを導入してもよいが時間がないのでシリアル的処理)
   Step1:現在座標tfを1回取得
   Step3:転回により指示角度へ向く
  """
  def manual_rotate(self):
    rospy.loginfo('手動転回用スレッド開始')

    # 結果情報
    motion_result = MotionResult.MOTION_RESULT_OK

    # tf取得ノード生成
    listener = tf.TransformListener()

    # 必ず1回は処理を実行するためのWhile
    while True:
      # Step1:現在座標tf取得
      rospy.loginfo('ステップ1:現在座標tf取得開始')
      # tf取得可能になったら1回tfを取得する
      try:
        listener.waitForTransform(MotionControlDefine.MOTION_NS_FOR_MAP+'/map', self.name_space+'/base_link', rospy.Time(0), rospy.Duration(3.0))
        ([pos_x, pos_y, pos_z], [rot_x, rot_y, rot_z, rot_w]) = listener.lookupTransform(MotionControlDefine.MOTION_NS_FOR_MAP+'/map', self.name_space+'/base_link', rospy.Time(0))
      except:
        rospy.logerr('tf取得失敗')
        # tf取得失敗で処理中断
        motion_result = MotionResult.MOTION_RESULT_NG
        break
      rospy.loginfo('現在座標 postion:{}, {}, {} rotation:{}, {}, {}, {}'.format(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w))

      # ステップ間でのモーション中止確認
      if self.flag_motion_cancel:
        motion_result = MotionResult.MOTION_RESULT_NG
        break

      # Step2:目標座標への向きを算出
      rospy.loginfo('ステップ2:転回により指示角度へ向く')
      # 目標角度を算出する
      orientation = tf.transformations.quaternion_from_euler(0, 0, self.move_goal.goal_rpy_yaw)
      target_degree, yaw = self.quaternion2degree(orientation)
      rospy.loginfo('目的角度 target_dgree:{}, target_yaw:{}, target_quaternion:{}'.format(target_degree, self.move_goal.goal_rpy_yaw, orientation))

      # ステップ間でのモーション中止確認
      if self.flag_motion_cancel:
        motion_result = MotionResult.MOTION_RESULT_NG
        break

      # Step3:目標座標へ転回
      rospy.loginfo('ステップ3:目標座標へ転回')
      result = self.rotate_target4degree(target_degree)
      # 転回NGであったならループを抜ける
      if result != 0:
        motion_result = MotionResult.MOTION_RESULT_NG
        break
        
      # ステップ間でのモーション中止確認
      if self.flag_motion_cancel:
        motion_result = MotionResult.MOTION_RESULT_NG
        break

      break

    # 結果通知コールバック関数が登録されていれば呼び出し
    # detailは将来的なもので現時点では未使用
    detail = ''
    if self.callback_motion_end is not None:
      self.callback_motion_end(motion_result, detail)

    rospy.loginfo('手動移動用スレッド終了')

  # 目的角度への転回処理(度数指定)
  # 戻り値:0/-1=OK/NG
  def rotate_target4degree(self, target_degree, rotate_speed=MotionControlDefine.MOTION_ROTATION_SPEED, arrival_range=MotionControlDefine.MOTION_ROTATE_ARRAIVAL_RANGE):
    rospy.loginfo('rotate_target4quaternion:tf取得可能確認処理開始')
    # tf取得可能になったら転回開始
    listener = tf.TransformListener()
    try:
      listener.waitForTransform(MotionControlDefine.MOTION_NS_FOR_MAP+'/map', self.name_space+'/base_link', rospy.Time(0), rospy.Duration(3.0))
      ([x, y, z], rot) = listener.lookupTransform(MotionControlDefine.MOTION_NS_FOR_MAP+'/map', self.name_space+'/base_link', rospy.Time(0))
    except:
      rospy.logerr('rotate_target4degree:tf取得失敗')
      return -1

    rospy.loginfo('rotate_target4quaternion:転回処理開始')
    # 目的角度への転回を実施
    # 到達フラグ
    flag_arrivaled = False

    # 目的角度の調整
    target_degree = round(target_degree, 3)
    first_point, second_point, division_check_kind = self.set_rangevalue_division(target_degree)

    # 転回速度・方向設定
    rotate_speed = self.determine_rotate_speed(target_degree, rot)

    # 転回速度指定cmd_velパラメータ設定
    twist_msg = Twist()
    twist_msg.angular.z = rotate_speed

    # ループ周期設定
    rate = rospy.Rate(1000/MotionControlDefine.MOTION_CHECK_ROTATE_TF)
    while self.flag_motion_cancel is not True:
      # 転回速度cmd_vel発行
      self.pub_cmd_vel.publish(twist_msg)
      try:
        # 現在座標取得
        listener.waitForTransform(MotionControlDefine.MOTION_NS_FOR_MAP+'/map', self.name_space+'/base_link', rospy.Time(0), rospy.Duration(3.0))
        (pos, q) = listener.lookupTransform(MotionControlDefine.MOTION_NS_FOR_MAP+'/map', self.name_space+'/base_link', rospy.Time(0))
      except:
        rospy.logerr('rotate_target4degree:tf取得失敗')
        flag_arrivaled = False
        break

      # 目的角度到達チェック
      # クォータニオンから角度数とヨー値へ変換
      current_degree, current_yaw = self.quaternion2degree(q)
      if division_check_kind == 0:
        if first_point <= current_degree and second_point >= current_degree:
          rospy.loginfo('rotate_target4degree:目的角度到達 target_degree:{}, current_degree:{}'.format(target_degree, current_degree))
          # 到達フラグをTureにしてループを抜ける
          flag_arrivaled = True
          break
      else:
        if ((first_point >= 0 and current_degree >= 0) and (first_point >= current_degree)) \
            or ((second_point < 0 and current_degree < 0) and (second_point <= current_degree)):
          rospy.loginfo('rotate_target4degree:目的角度到達 target_degree:{}, current_degree:{}'.format(target_degree, current_degree))
          # 到達フラグをTureにしてループを抜ける
          flag_arrivaled = True
          break
        if ((first_point < 0 and current_degree < 0) and (first_point >= current_degree)) \
            or ((second_point >= 0 and current_degree < 0) and (second_point <= current_degree)):
          rospy.loginfo('rotate_target4degree:目的角度到達 target_degree:{}, current_degree:{}'.format(target_degree, current_degree))
          # 到達フラグをTureにしてループを抜ける
          flag_arrivaled = True
          break

      # 指定周期処理待ち
      rate.sleep()

    if self.flag_motion_cancel:
      rospy.loginfo('rotate_target4degree:モーション中止フラグによりループ抜け')

    # 転回停止
    twist_msg = Twist()
    twist_msg.angular.z = 0.0
    self.pub_cmd_vel.publish(twist_msg)

    return 0 if flag_arrivaled else -1

  # クォータニオンから角度(度数)とヨー値へ変換
  def quaternion2degree(self, q):
    euler = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
    degree = math.degrees(euler[2])
    yaw = euler[2]
    return degree, yaw

  # 目的角度到達判定最小・最大値、二分割チェック情報
  def set_rangevalue_division(self, target_degree):
    range_first_point = 0.0
    range_second_point = 0.0
    division_check_kind = 0
    range_value = MotionControlDefine.MOTION_ROTATE_ARRAIVAL_RANGE

    rospy.loginfo('set_rangevalue_division target_degree:{}'.format(target_degree))

    # 目的角度と到達判断範囲から最小・最大、二分割チェックを決める
    # 二分割チェック
    # 0:二分割チェックなし(fp以上&&sp以下)
    # 1:二分割チェックあり(fp以下||sp以上)
    if target_degree >= 0:
      if target_degree < range_value:
        range_first_point = target_degree + range_value
        range_second_point = target_degree - range_value
        division_check_kind = 1
      elif target_degree > (180 - range_value):
        range_first_point = -180 - ((180 - target_degree) - range_value)
        range_second_point = target_degree - range_value
        division_check_kind = 1
      else:
        range_first_point = target_degree - range_value
        range_second_point = target_degree + range_value
        division_check_kind = 0
    else:
      if abs(target_degree) < range_value:
        range_first_point = range_value - abs(target_degree)
        range_second_point = target_degree - (range_value)
        division_check_kind = 1
      elif abs(target_degree) > (180 - range_value):
        range_first_point = target_degree + range_value
        range_second_point = 180 - (range_value - (180 - (abs(target_degree))))
        division_check_kind = 1
      else:
        range_first_point = target_degree - range_value
        range_second_point = target_degree + range_value
        division_check_kind = 0

    rospy.loginfo('set_rangevalue_division target_degree:{}, fp:{}, sp:{}, dck:{}'.format(target_degree, range_first_point, range_second_point, division_check_kind))

    return range_first_point, range_second_point, division_check_kind

  # 転回速度・向きを設定
  def determine_rotate_speed(self, target_degree, current_q):
    # 現在のクォータニオンから角度数を算出
    current_degree, yaw = self.quaternion2degree(current_q)
    rospy.loginfo('determine_rotate_speed target_degree:{}, current_dgree:{}'.format(target_degree, current_degree))

    # ターゲットと現在角度を360度ベースにする
    if target_degree < 0:
      target_degree = 360 + target_degree
    if current_degree < 0:
      current_degree = 360 + current_degree

    rotate_speed = MotionControlDefine.MOTION_ROTATION_SPEED

    rospy.loginfo('determine_rotate_speed (360度計算) target_degree:{}, current_dgree:{}'.format(target_degree, current_degree))

    # ターゲットと現在角度から転回速度(向きも考慮)を設定
    if target_degree >= current_degree:
      if target_degree - current_degree <= 180:
        rotate_speed = MotionControlDefine.MOTION_ROTATION_SPEED
      else:
        rotate_speed = MotionControlDefine.MOTION_ROTATION_SPEED * -1
    else:
      if abs(target_degree - current_degree) <= 180:
        rotate_speed = MotionControlDefine.MOTION_ROTATION_SPEED * -1
      else:
        rotate_speed = MotionControlDefine.MOTION_ROTATION_SPEED

    rospy.loginfo('determine_rotate_speed rotate_speed:{}'.format(rotate_speed))
    return rotate_speed

  # モーション中止指示受信
  def motion_cancel(self):
    rospy.loginfo('MotionManualMove:モーション中止受信')
    self.flag_motion_cancel = True
