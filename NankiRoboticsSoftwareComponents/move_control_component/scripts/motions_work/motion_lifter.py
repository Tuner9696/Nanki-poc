#!/usr/bin/env python
# -*- coding: utf-8 -*-
from tabnanny import check
import rospy
# import moveit_commander
# import moveit_msgs.msg
import tf
import math
import numpy
from threading import Thread
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from motion_control_define import MotionControlDefine, MotionKind, LifterControlKind
from motion_control_define import MotionResult
from std_msgs.msg import Float32

"""
ROSによるリフター制御
リフター制御を指示し、その伸縮の値にてゴール監視をする
"""
class MotionLifter():
  def __init__(self, goal, name_space, callback_motion_end=None):
    rospy.loginfo('リフター制御用クラスコンストラクタ開始')
    # 制御指示内容保存
    self.motion_goal = goal
    # 名前空間保存
    self.name_space = name_space
    # 移動完了結果通知コールバック関数保存
    self.callback_motion_end = callback_motion_end
    # モーション中止フラグ初期化
    self.flag_motion_cancel = False
    # リフター制御対象軸名
    self.lifter_joint_name = ['ankle_joint', 'knee_joint']

    # リフター制御指定Publisher生成
    self.pub_lifter_cmd = rospy.Publisher(self.name_space+'/lifter_controller/command', JointTrajectory, queue_size=10)

    # 制御終了フラグOFF
    self.flag_end_control = False

    # リフター制御用スレッド開始
    if MotionControlDefine.LIFTER_CONTROL_KIND == LifterControlKind.KIND_CUSTOMIZE:
      self.th_move = Thread(target=self.lifter_control_customize)
      self.th_move.start()
    else:
      self.th_move = Thread(target=self.lifter_control)
      self.th_move.start()
      self.check_range = MotionControlDefine.LIFTER_MOVE_RANGE
    # else:
    #   self.th_move = Thread(target=self.lifter_control_moveit)
    #   self.th_move.start()

  """
  リフター制御用スレッド(Publish版)
  """
  def lifter_control(self):
    rospy.loginfo('リフター制御用(Publish版)スレッド開始')

    # 結果情報
    motion_result = MotionResult.MOTION_RESULT_OK

    # リフター上昇、降下の種別を取り出し
    self.control_kind = self.motion_goal.motion_kind

    rospy.loginfo('リフター制御 昇降:{}'.format(self.control_kind))

    # バッテリー残量によるリフター制御ガード処理
    if MotionControlDefine.LIFTER_RESTRICTION_BY_VOLTAGE:
      rospy.loginfo('バッテリー残量によるガード処理')
      # 1度バッテリー残量を取得
      if self.name_space == '':
        voltage_topic_name = '/seed_r7_ros_controller/voltage'
      else:
        voltage_topic_name = self.name_space + '_seed_r7_ros_controller/voltage'
      voltage_massage = rospy.wait_for_message(voltage_topic_name, Float32)

      rospy.loginfo('バッテリー残量: {}%'.format(voltage_massage.data))
      # バッテリー残量が閾値以下であればリフター制御を行わない
      if voltage_massage.data <= MotionControlDefine.LIFTER_RESTRICTION_VOLTAGE_VALUE:
        rospy.loginfo('バッテリー残量により制御なし')
        # 正常終了としてコールバック関数をコールして終了
        detail = ''
        if self.callback_motion_end is not None:
          motion_result = MotionResult.MOTION_RESULT_OK
          self.callback_motion_end(motion_result, detail)

        rospy.loginfo('リフター制御用(Publish版)スレッド終了')

        return

    msg_lifter = JointTrajectory()
    msg_lifter.header.seq = 1
    msg_lifter.header.stamp = rospy.Time.now()
    msg_lifter.points = [JointTrajectoryPoint()]
    msg_lifter.joint_names = self.lifter_joint_name
    if self.control_kind == MotionKind.MOTION_KIND_LIFTERUP:
      msg_lifter.points[0].positions = [MotionControlDefine.LIFTER_MOVE_UP_ANKLE, MotionControlDefine.LIFTER_MOVE_UP_KNEE]
      msg_lifter.points[0].time_from_start = rospy.Time(MotionControlDefine.LIFTER_MOVE_UP_TIME)
    else:
      msg_lifter.points[0].positions = [MotionControlDefine.LIFTER_MOVE_DOWN_ANKLE, MotionControlDefine.LIFTER_MOVE_DOWN_KNEE]
      msg_lifter.points[0].time_from_start = rospy.Time(MotionControlDefine.LIFTER_MOVE_DOWN_TIME)

    # リフター状態Subscribe開始
    self.sub_lifter_state = rospy.Subscriber(self.name_space+'/lifter_controller/state', JointTrajectoryControllerState, self.callback_lifter_state)

    # Publish送信ガードカウンタ
    count = 0

    # 制御完了待ちループ
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      if self.flag_end_control:
        break

      # Lifter制御トピックPublish
      # rospy.loginfo('リフター制御ROS発行:{}'.format(msg_lifter))
      if count <= MotionControlDefine.LIFTER_PUBLISH_RESEND:
        self.pub_lifter_cmd.publish(msg_lifter)
        count = count + 1

      rate.sleep()

    # リフター状態Subscribe停止
    self.sub_lifter_state.unregister()

    # 結果通知コールバック関数が登録されていれば呼び出し
    # detailは将来的なもので現時点では未使用
    detail = ''
    if self.callback_motion_end is not None:
      if self.flag_motion_cancel:
        motion_result = MotionResult.MOTION_RESULT_NG
      self.callback_motion_end(motion_result, detail)

    rospy.loginfo('リフター制御用(Publish版)スレッド終了')

  """
  リフター制御用スレッド(MoveIt!版)
  """
  # def lifter_control_moveit(self):
  #   rospy.loginfo('リフター制御用(MoveIt!版)スレッド開始')

  #   # 結果情報
  #   motion_result = MotionResult.MOTION_RESULT_OK

  #   # リフター上昇、降下の種別を取り出し
  #   self.control_kind = self.motion_goal.motion_kind

  #   rospy.loginfo('リフター制御 昇降:{}'.format(self.control_kind))

  #   # バッテリー残量によるリフター制御ガード処理
  #   if MotionControlDefine.LIFTER_RESTRICTION_BY_VOLTAGE:
  #     rospy.loginfo('バッテリー残量によるガード処理')
  #     # 1度バッテリー残量を取得
  #     if self.name_space == '':
  #       voltage_topic_name = '/seed_r7_ros_controller/voltage'
  #     else:
  #       voltage_topic_name = self.name_space + '_seed_r7_ros_controller/voltage'
  #     voltage_massage = rospy.wait_for_message(voltage_topic_name, Float32)

  #     rospy.loginfo('バッテリー残量: {}%'.format(voltage_massage.data))
  #     # バッテリー残量が閾値以下であればリフター制御を行わない
  #     if voltage_massage.data <= MotionControlDefine.LIFTER_RESTRICTION_VOLTAGE_VALUE:
  #       rospy.loginfo('バッテリー残量により制御なし')
  #       # 正常終了としてコールバック関数をコールして終了
  #       detail = ''
  #       if self.callback_motion_end is not None:
  #         motion_result = MotionResult.MOTION_RESULT_OK
  #         self.callback_motion_end(motion_result, detail)

  #       rospy.loginfo('リフター制御用(MoveIt!版)スレッド終了')

  #       return

  #   # MoveIt!での制御開始
  #   rospy.loginfo("リフター制御のMoveIt!関連オブジェクト生成")
  #   mi_commandar = moveit_commander.RobotCommander()
  #   mi_psi = moveit_commander.PlanningSceneInterface()
  #   mi_group_commandar = moveit_commander.MoveGroupCommander("torso")
  #   mi_group_commandar.set_pose_reference_frame(self.name_space + "/base_link")
  #   mi_group_commandar.set_end_effector_link(self.name_space + "/body_link")

  #   # リフター昇降での各Link位置と速度設定
  #   if self.control_kind == MotionKind.MOTION_KIND_LIFTERUP:
  #     ankle_pos = MotionControlDefine.LIFTER_MOVE_UP_ANKLE
  #     knee_pos = MotionControlDefine.LIFTER_MOVE_UP_KNEE
  #     vel = MotionControlDefine.LIFTER_MOVE_UP_VEL
  #   else:
  #     ankle_pos = MotionControlDefine.LIFTER_MOVE_DOWN_ANKLE
  #     knee_pos = MotionControlDefine.LIFTER_MOVE_DOWN_KNEE
  #     vel = MotionControlDefine.LIFTER_MOVE_DOWN_VEL

  #   rospy.loginfo("リフター制御パラメータ Ankle:{}, Knee:{}, Vel:{}".format(ankle_pos, knee_pos, vel))

  #   mi_group_commandar.set_joint_value_target('ankle_joint', ankle_pos)
  #   mi_group_commandar.set_joint_value_target('knee_joint', knee_pos)
  #   mi_group_commandar.set_max_velocity_scaling_factor(vel)
  #   mi_group_commandar.go(wait=True)

  #   # 結果通知コールバック関数が登録されていれば呼び出し
  #   # detailは将来的なもので現時点では未使用
  #   detail = ''
  #   if self.callback_motion_end is not None:
  #     if self.flag_motion_cancel:
  #       motion_result = MotionResult.MOTION_RESULT_NG
  #     self.callback_motion_end(motion_result, detail)

  #   rospy.loginfo('リフター制御用(MoveIt!版)スレッド終了')

  """
  リフター制御用スレッド(加減速版)
  """
  def lifter_control_customize(self):
    rospy.loginfo('リフター制御用(加減速版)スレッド開始')

    # 結果情報
    motion_result = MotionResult.MOTION_RESULT_OK

    # リフター上昇、降下の種別を取り出し
    self.control_kind = self.motion_goal.motion_kind

    rospy.loginfo('リフター制御 昇降:{}'.format(self.control_kind))

    # バッテリー残量によるリフター制御ガード処理
    if MotionControlDefine.LIFTER_RESTRICTION_BY_VOLTAGE:
      rospy.loginfo('バッテリー残量によるガード処理')
      # 1度バッテリー残量を取得
      if self.name_space == '':
        voltage_topic_name = '/seed_r7_ros_controller/voltage'
      else:
        voltage_topic_name = self.name_space + '_seed_r7_ros_controller/voltage'
      voltage_massage = rospy.wait_for_message(voltage_topic_name, Float32)

      rospy.loginfo('バッテリー残量: {}%'.format(voltage_massage.data))
      # バッテリー残量が閾値以下であればリフター制御を行わない
      if voltage_massage.data <= MotionControlDefine.LIFTER_RESTRICTION_VOLTAGE_VALUE:
        rospy.loginfo('バッテリー残量により制御なし')
        # 正常終了としてコールバック関数をコールして終了
        detail = ''
        if self.callback_motion_end is not None:
          motion_result = MotionResult.MOTION_RESULT_OK
          self.callback_motion_end(motion_result, detail)

        rospy.loginfo('リフター制御用(加減速版)スレッド終了')

        return

    # リフター制御によるAnkleアクチュエータの遅延始動が必要な場合
    # 現時点でのKneeとAnkleの軸値を取得
    if MotionControlDefine.LIFTER_ACCDCC_DELAY_ANKLE_FLAG:
      rospy.loginfo("Ankle遅延始動あり")
      current_pos = rospy.wait_for_message(self.name_space+'/lifter_controller/state', JointTrajectoryControllerState, timeout=30)
      current_knee = current_pos.actual.positions[0]
      current_ankle = current_pos.actual.positions[1]
      rospy.loginfo("現在の軸値 knee:{} ankle:{}".format(current_knee, current_ankle))

      # Publishメッセージ作成
      msg_lifter = JointTrajectory()
      msg_lifter.header.seq = 1
      msg_lifter.header.stamp = rospy.Time.now()
      msg_lifter.points = [JointTrajectoryPoint()]
      msg_lifter.joint_names = self.lifter_joint_name
      msg_lifter.points[0].time_from_start = rospy.Time(MotionControlDefine.LIFTER_ACCDCC_DELAY_ANKLE*5)
      if self.control_kind == MotionKind.MOTION_KIND_LIFTERUP:
        msg_lifter.points[0].positions = [current_ankle, current_knee+MotionControlDefine.LIFTER_ACCDCC_DELAY_MOVERANGE]
      else:
        msg_lifter.points[0].positions = [current_ankle, current_knee-MotionControlDefine.LIFTER_ACCDCC_DELAY_MOVERANGE]

      # Publish送信ガードカウンタ
      count = 0
      # 制御完了待ちループ
      rate = rospy.Rate(10)
      loop_count = 0
      while not rospy.is_shutdown():
        # Lifter制御トピックPublish
        # rospy.loginfo('リフター制御ROS発行:{}'.format(msg_lifter))
        if count <= MotionControlDefine.LIFTER_PUBLISH_RESEND:
          self.pub_lifter_cmd.publish(msg_lifter)
          count = count + 1

        rate.sleep()

        # 遅延時間が終わったら本制御へ移行
        loop_count = loop_count + 1
        if loop_count * 100 >= MotionControlDefine.LIFTER_ACCDCC_DELAY_ANKLE * 1000:
          rospy.loginfo('遅延終了で本制御へ移行')
          break

    # シーケンス番号初期化
    seq_index = 1

    # Publishメッセージ作成
    msg_lifter = JointTrajectory()
    msg_lifter.header.seq = seq_index
    msg_lifter.header.stamp = rospy.Time.now()
    msg_lifter.points = [JointTrajectoryPoint()]
    msg_lifter.joint_names = self.lifter_joint_name

    # リフター上昇制御
    if self.control_kind == MotionKind.MOTION_KIND_LIFTERUP:
      list_count = len(MotionControlDefine.LIFTER_ACCDCC_UP_LIST)
      rospy.loginfo('リフター制御リスト数 list_count:{}'.format(list_count))
      index = 1
      for array in MotionControlDefine.LIFTER_ACCDCC_UP_LIST:
        point = array[0]
        time = array[1]
        rospy.loginfo('取り出し軸値と制御時間 point:{} time:{}'.format(point, time))
        if index < list_count:
          # 最終制御以外は連続モーションのため到達軸値を調整
          check_point = point - MotionControlDefine.LIFTER_ACCDCC_UP_PRERANGE
          # 到達確認軸値を保存
          self.knee_check_point = check_point
          rospy.loginfo('チェック軸値 self.knee_check_point:{}'.format(self.knee_check_point))
        else:
          # 到達確認軸値を保存
          self.knee_check_point = point - MotionControlDefine.LIFTER_MOVE_RANGE
          rospy.loginfo('チェック軸値 self.knee_check_point:{}'.format(self.knee_check_point))

        msg_lifter.header.stamp = rospy.Time.now()
        msg_lifter.points[0].positions = [(point*-1.0), point]
        msg_lifter.points[0].time_from_start = rospy.Time(time)

        # 制御中インデックスをインクリメント
        index = index + 1

        # 制御終了フラグクリア
        self.flag_end_control = False

        # リフター状態Subscribe開始
        rospy.loginfo('リフター状態Subscriber開始')
        self.sub_lifter_state = rospy.Subscriber(self.name_space+'/lifter_controller/state', JointTrajectoryControllerState, self.callback_lifter_state_customize)

        # Publish送信ガードカウンタ
        count = 0
        # 制御完了待ちループ
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
          if self.flag_end_control:
            # リフター状態Subscribe停止
            rospy.loginfo('リフター状態Subscriber停止')
            self.sub_lifter_state.unregister()
            break

          # Lifter制御トピックPublish
          # rospy.loginfo('リフター制御ROS発行:{}'.format(msg_lifter))
          if count <= MotionControlDefine.LIFTER_PUBLISH_RESEND:
            self.pub_lifter_cmd.publish(msg_lifter)
            count = count + 1

          rate.sleep()

        # リフター状態Subscribe停止
        rospy.loginfo('リフター状態Subscriber停止')
        self.sub_lifter_state.unregister()

    # リフター降下制御
    else:
      list_count = len(MotionControlDefine.LIFTER_ACCDCC_DOWN_LIST)
      rospy.loginfo('リフター制御リスト数 list_count:{}'.format(list_count))
      index = 1
      for array in MotionControlDefine.LIFTER_ACCDCC_DOWN_LIST:
        point = array[0]
        time = array[1]
        rospy.loginfo('取り出し軸値と制御時間 point:{} time:{}'.format(point, time))
        if index < list_count:
          # 最終制御以外は連続モーションのため到達軸値を調整
          check_point = point + MotionControlDefine.LIFTER_ACCDCC_DOWN_PRERANGE
          # 到達確認軸値を保存
          self.knee_check_point = check_point
          rospy.loginfo('チェック軸値 self.knee_check_point:{}'.format(self.knee_check_point))
        else:
          # 到達確認軸値を保存
          self.knee_check_point = point + MotionControlDefine.LIFTER_MOVE_RANGE
          rospy.loginfo('チェック軸値 self.knee_check_point:{}'.format(self.knee_check_point))
        
        msg_lifter.header.stamp = rospy.Time.now()
        msg_lifter.points[0].positions = [(point*-1.0), point]
        msg_lifter.points[0].time_from_start = rospy.Time(time)

        # 制御中インデックスをインクリメント
        index = index + 1

        # 制御終了フラグクリア
        self.flag_end_control = False

        # リフター状態Subscribe開始
        rospy.loginfo('リフター状態Subscriber開始')
        self.sub_lifter_state = rospy.Subscriber(self.name_space+'/lifter_controller/state', JointTrajectoryControllerState, self.callback_lifter_state_customize)

        # Publish送信ガードカウンタ
        count = 0
        # 制御完了待ちループ
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
          if self.flag_end_control:
            # リフター状態Subscribe停止
            rospy.loginfo('リフター状態Subscriber停止')
            self.sub_lifter_state.unregister()
            break

          # Lifter制御トピックPublish
          # rospy.loginfo('リフター制御ROS発行:{}'.format(msg_lifter))
          if count <= MotionControlDefine.LIFTER_PUBLISH_RESEND:
            self.pub_lifter_cmd.publish(msg_lifter)
            count = count + 1

          rate.sleep()

    # 結果通知コールバック関数が登録されていれば呼び出し
    # detailは将来的なもので現時点では未使用
    detail = ''
    if self.callback_motion_end is not None:
      if self.flag_motion_cancel:
        motion_result = MotionResult.MOTION_RESULT_NG
      self.callback_motion_end(motion_result, detail)

    rospy.loginfo('リフター制御用(加減速版)スレッド終了')

  # リフター状態受信コールバック
  def callback_lifter_state(self, lifter_state):
    # 制御種別によりAnkleの数値チェックを行う
    if self.flag_end_control is not True:
      # リフター上昇の場合
      if self.control_kind == MotionKind.MOTION_KIND_LIFTERUP:
        if lifter_state.actual.positions[1] <= (MotionControlDefine.LIFTER_MOVE_UP_ANKLE + self.check_range):
          rospy.loginfo('リフター上昇完了にてイベント発行')
          self.flag_end_control = True
      else:
        if lifter_state.actual.positions[1] >= (MotionControlDefine.LIFTER_MOVE_DOWN_ANKLE - self.check_range):
          rospy.loginfo('リフター降下完了にてイベント発行')
          self.flag_end_control = True

  # リフター状態受信コールバック(加減速版)
  def callback_lifter_state_customize(self, lifter_state):
    # 制御種別によりKneeの数値チェックを行う
    if self.flag_end_control is not True:
      if self.control_kind == MotionKind.MOTION_KIND_LIFTERUP:
        if lifter_state.actual.positions[0] >=self.knee_check_point:
          rospy.loginfo('リフター上昇完了にて終了フラグON')
          self.flag_end_control = True
      else:
        if lifter_state.actual.positions[0] <= self.knee_check_point:
          rospy.loginfo('リフター降下完了にて終了フラグON')
          self.flag_end_control = True

  # モーション中止指示受信
  def motion_cancel(self):
    rospy.loginfo('MotionLifter:モーション中止受信')
    self.flag_motion_cancel = True
    self.event_end_control.set()
