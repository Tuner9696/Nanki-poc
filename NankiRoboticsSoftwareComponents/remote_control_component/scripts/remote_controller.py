#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from remote_controller_define import RemoteControlDefine
from threading import Thread
from remote_control_component.msg import ControllerEvent
from remote_control_component.msg import RemoteMode
from remote_control_component.msg import RemoteMove
from remote_control_component.msg import MissionCommand
from auto_action_move import AutoActionMove
from remote_control_component.msg import ObstacleInformation
from remote_control_component.msg import Obstacle

"""
遠隔制御コンポーネントのメインクラス
コントローラ監視およびクラウドからの制御情報を受信し
それに合わせた処理を実行
"""
class RemoteMotion():
  # コンストラクタ
  def __init__(self):
    rospy.loginfo('RemoteMotionクラス開始')
    # プロパティ初期化
    self._remote_mode = False
    self._manual_motion = True
    self._axes_fb = 0.0
    self._axes_lr = 0.0
    self._axes_angular = 0.0
    self._lifter_up = True

    # 手動モーション制御スレッドオブジェクト初期化
    self.th_move = None
    self.th_lifter = None

    # 自律移動用クラスオブジェクト
    self.obj_automove = None

    # 障害物あり・なしフラグ初期化
    self._obstacle_flag = False

    # Mover移動用ROSパブリッシャー作成
    rospy.loginfo('Mover移動用ROSパブリッシャー作成')
    # 制御対象ロボットの名前空間を併せてトピック名生成
    topic_name = RemoteControlDefine.NS_CONTROLL_ROBOT + '/cmd_vel'
    self.pub_cmd_vel = rospy.Publisher(topic_name, Twist, queue_size=10)
    # リフター制御用ROSパブリッシャー作成
    rospy.loginfo('リフター制御用ROSパブリッシャー作成')
    # 制御対象ロボットの名前空間を併せてトピック名生成
    topic_name = RemoteControlDefine.NS_CONTROLL_ROBOT + '/lifter_controller/command'
    self.pub_lifter = rospy.Publisher(topic_name, JointTrajectory, queue_size=10)

    # ミッション制御パブリッシャー生成
    rospy.loginfo('ミッション制御用パブリッシャー作成')
    self.pub_ext_mission = rospy.Publisher('/ext_mission', MissionCommand, queue_size=10)

  # 遠隔制御モードプロパティ
  @property
  def remote_mode(self):
    return self._remote_mode
  @remote_mode.setter
  def remote_mode(self, value):
    self._remote_mode = value

  # 手動制御中フラグプロパティ
  @property
  def manual_motion(self):
    return self._manual_motion
  @manual_motion.setter
  def manual_motion(self, value):
    self._manual_motion = value

  # 前後用スティックの傾斜量保存プロパティ
  @property
  def axes_fb(self):
    return self._axes_fb
  @axes_fb.setter
  def axes_fb(self, value):
    self._axes_fb = value

  # 左右用スティックの傾斜量保存プロパティ
  @property
  def axes_lr(self):
    return self._axes_lr
  @axes_lr.setter
  def axes_lr(self, value):
    self._axes_lr = value

  # 転回用スティックの傾斜量保存プロパティ
  @property
  def axes_angular(self):
    return self._axes_angular
  @axes_angular.setter
  def axes_angular(self, value):
    self._axes_angular = value

  # リフター状態保存のプロパティ
  @property
  def lifter_up(self):
    return self._lifter_up
  @lifter_up.setter
  def lifter_up(self, value):
    self._lifter_up = value

  # 障害物あり・なしの情報フラグ
  @property
  def obstacle_flag(self):
    return self._obstacle_flag
  @obstacle_flag.setter
  def obstacle_flag(self, value):
    self._obstacle_flag = value

  # 外部コンポーネントからのリモートモード受信のイベントコールバック関数
  def on_remote_mode(self, remote_mode):
    # 前モードを保存
    pre_mode = self._remote_mode
    # 受信モードを保存
    self._remote_mode = remote_mode.remote_control_mode

    if pre_mode == False and self._remote_mode:
      rospy.loginfo('リモートモードをONへ')
      # ミッション制御にリモートモードONによるミッション停止通知
      msg = MissionCommand()
      msg.command = 0 if self._remote_mode else 1
      self.pub_ext_mission.publish(msg)

      # 手動モーション制御スレッドを起動
      # Mover移動用ワーカースレッド起動
      rospy.loginfo('Mover移動用ワーカースレッド起動')
      self.th_move = Thread(target=self.move_worker)
      self.th_move.start()
      # リフター制御用ワーカースレッド起動
      rospy.loginfo('リフター制御用ワーカースレッド起動')
      self.th_lifter = Thread(target=self.lifter_worker)
      self.th_lifter.start()
    elif pre_mode and self._remote_mode == False:
      rospy.loginfo('リモートモードをOFFへ')
      # ミッション制御にリモートモードOFFによるミッション再開通知
      msg = MissionCommand()
      msg.command = 0 if self._remote_mode else 1
      self.pub_ext_mission.publish(msg)

  # Joyからの操作イベントコールバック関数
  def on_joydata(self, joy_data):
    # 前モードを保存
    pre_mode = self._remote_mode
    # PSボタンを押されたら遠隔制御モード変更
    if joy_data.button_ps == True:
      rospy.loginfo('PSボタンが押された')
      self._remote_mode = not bool(self._remote_mode)
      rospy.loginfo('モード状態:{}'.format(self._remote_mode))

    if pre_mode == False and self._remote_mode:
      # ミッション制御にリモートモードONによるミッション停止通知
      msg = MissionCommand()
      msg.command = 0 if self._remote_mode else 1
      self.pub_ext_mission.publish(msg)

      # 手動モーション制御スレッドを起動
      # Mover移動用ワーカースレッド起動
      rospy.loginfo('Mover移動用ワーカースレッド起動')
      self.th_move = Thread(target=self.move_worker)
      self.th_move.start()
      # リフター制御用ワーカースレッド起動
      rospy.loginfo('リフター制御用ワーカースレッド起動')
      self.th_lifter = Thread(target=self.lifter_worker)
      self.th_lifter.start()
      # 遠隔制御モード移行時はスレッド生成のみ
      return
    elif pre_mode and self._remote_mode == False:
      # ミッション制御にリモートモードOFFによるミッション再開通知
      msg = MissionCommand()
      msg.command = 0 if self._remote_mode else 1
      self.pub_ext_mission.publish(msg)
      return

    # 遠隔制御モードでなければ処理無し
    if self._remote_mode != True:
      return

    # 自律移動キャンセルボタンチェック
    # 四角ボタンが押された際、
    # 自律移動中であったらその移動をキャンセルする
    if joy_data.button_square:
      if self._manual_motion != True and self.obj_automove is not None:
        rospy.loginfo('自律移動キャンセル')
        self.obj_automove.action_cancel()

    # 手動モーション制御中でなければ処理無し
    if self._manual_motion != True:
      return

    # Mover用速度を算出
    self.calc_speed(joy_data.axes_lstick_fb, joy_data.axes_rstick_lr, joy_data.axes_lstick_angular)

    # リフターの上昇/降下のボタン押下検出チェック
    if joy_data.button_triangle:
      rospy.loginfo('リフター上昇ボタン押下')
      self._lifter_up = True
    if joy_data.button_cross:
      rospy.loginfo('リフター降下ボタン押下')
      self._lifter_up = False

  # 座標指定移動要求受信コールバック関数
  def on_remote_move(self, move_goal):
    # リモートモードがONでなければ処理無し
    if self._remote_mode != True:
      return

    # 手動モーション制御中をFalseへ
    self._manual_motion = False

    # すでに自律移動中であればそれをキャンセルするだけ
    if self.obj_automove is not None:
      self.obj_automove.action_cancel()
      return

    # 自律移動用クラス生成と移動開始
    self.obj_automove = AutoActionMove(move_goal, self.on_end_automove)

  # 自律移動完了受信コールバック関数
  def on_end_automove(self, result, status):
    # 自律移動クラスオブジェクトを初期化
    self.obj_automove = None

    # 手動モーション制御中をTrueへ
    self._manual_motion = True

  # Mover移動用ワーカースレッド関数
  def move_worker(self):
    rospy.loginfo('Mover移動用ワーカースレッド開始')
    rate = rospy.Rate(1.5)
    # 遠隔制御モードがOFFになったらスレッド終了
    while self._remote_mode:
      if self._manual_motion:
        # rospy.loginfo('move_worker')
        # 障害検知状態により前進については速度ゼロ固定
        t = Twist()
        if RemoteControlDefine.FLAG_OBSTACLE_STOP and self._obstacle_flag:
          if self._axes_fb >= 0:
            t.linear.x = 0.0
          else:
            t.linear.x = self._axes_fb * RemoteControlDefine.MOVER_SPEED_MAX_BACKWARD
          if self._axes_lr >= 0:
            t.linear.y = self._axes_lr * RemoteControlDefine.MOVER_SPEED_MAX_LEFTMOVE
          else:
            t.linear.y = self._axes_lr * RemoteControlDefine.MOVER_SPEED_MAX_RIGHTMOVE
          t.linear.z = 0.0
          t.angular.z = self._axes_angular * RemoteControlDefine.MOVER_SPEED_MAX_ROTATE
          rospy.logdebug('速度指定:{}'.format(t))
        else:
          if self._axes_fb >= 0:
            t.linear.x = self._axes_fb * RemoteControlDefine.MOVER_SPEED_MAX_FORWARD
          else:
            t.linear.x = self._axes_fb * RemoteControlDefine.MOVER_SPEED_MAX_BACKWARD
          if self._axes_lr >= 0:
            t.linear.y = self._axes_lr * RemoteControlDefine.MOVER_SPEED_MAX_LEFTMOVE
          else:
            t.linear.y = self._axes_lr * RemoteControlDefine.MOVER_SPEED_MAX_RIGHTMOVE
          t.linear.z = 0.0
          t.angular.z = self._axes_angular * RemoteControlDefine.MOVER_SPEED_MAX_ROTATE
          rospy.logdebug('速度指定:{}'.format(t))
        self.pub_cmd_vel.publish(t)

      rate.sleep()
    rospy.loginfo('Mover移動用ワーカースレッド終了')

  # リフター制御用ワーカースレッド関数
  def lifter_worker(self):
    rospy.loginfo('リフター制御用ワーカースレッド開始')

    # リフター軸の名前定義
    lifter_joint_name = ['ankle_joint', 'knee_joint']

    rate = rospy.Rate(1.5)
    pre_status = self._lifter_up
    # 遠隔制御モードがOFFになったらスレッド終了
    while self._remote_mode:
      if self._manual_motion:
        # 保存しているリフター状態が異なる時は制御
        if pre_status != self._lifter_up:
          rospy.loginfo('リフター制御')
          msg_lifter = JointTrajectory()
          msg_lifter.header.stamp = rospy.Time.now()
          msg_lifter.joint_names = lifter_joint_name
          msg_lifter.points = [JointTrajectoryPoint()]
          if self._lifter_up:
            msg_lifter.points[0].positions = [RemoteControlDefine.LIFTER_MOVE_UP_ANKLE, RemoteControlDefine.LIFTER_MOVE_UP_KNEE]
            move_time = RemoteControlDefine.LIFTER_MOVE_UP_TIME
          else:
            msg_lifter.points[0].positions = [RemoteControlDefine.LIFTER_MOVE_DOWN_ANKLE, RemoteControlDefine.LIFTER_MOVE_DOWN_KNEE]
            move_time = RemoteControlDefine.LIFTER_MOVE_DOWN_TIME
          msg_lifter.points[0].time_from_start = rospy.Time(move_time)

          rospy.logdebug('制御指定:{}'.format(msg_lifter))
          self.pub_lifter.publish(msg_lifter)

          pre_status = self._lifter_up
      
      rate.sleep()
    rospy.loginfo('リフター制御用ワーカースレッド終了')

  # Mover移動速度算出
  def calc_speed(self, axes_fb, axes_lr, axes_angular):
    if axes_fb < 0.1 and axes_fb > -0.1:
      self._axes_fb = 0.0
    elif axes_fb > 0.99:
      self._axes_fb = 1.0
    elif axes_fb < -0.99:
      self._axes_fb = -1.0
    else:
      pre_axes = self._axes_fb
      self._axes_fb = RemoteControlDefine.MOVER_SPEED_COEFFICIENT * pre_axes\
                      + (1 - RemoteControlDefine.MOVER_SPEED_COEFFICIENT) * axes_fb

    if axes_lr < 0.1 and axes_lr > -0.1:
      self._axes_lr = 0.0
    elif axes_lr > 0.99:
      self._axes_lr = 1.0
    elif axes_lr < -0.99:
      self._axes_lr = -1.0
    else:
      pre_axes = self._axes_lr
      self._axes_lr = RemoteControlDefine.MOVER_SPEED_COEFFICIENT * pre_axes\
                      + (1 - RemoteControlDefine.MOVER_SPEED_COEFFICIENT) * axes_lr

    if axes_angular < 0.1 and axes_angular > -0.1:
      self._axes_angular = 0.0
    elif axes_angular > 0.99:
      self._axes_angular = 1.0
    elif axes_angular < -0.99:
      self._axes_angular = -1.0
    else:
      pre_axes = self._axes_angular
      self._axes_angular = RemoteControlDefine.MOVER_SPEED_COEFFICIENT * pre_axes\
                      + (1 - RemoteControlDefine.MOVER_SPEED_COEFFICIENT) * axes_angular

    rospy.loginfo('FB:{}, LR:{}, Angular:{}'.format(self._axes_fb, self._axes_lr, self._axes_angular))

  # 障害検知の情報受信のイベントコールバック関数
  def on_obstacle_receive(self, obstacle_msg):
    rospy.logdebug('障害検知イベント受信')
    # 障害検知対応の場合は処理
    if RemoteControlDefine.FLAG_OBSTACLE_STOP is not True:
      rospy.logdebug('障害検知動作フラグOFF')
      self._obstacle_flag = False
      return

    # 障害物あり・なしにて分岐
    if obstacle_msg.obstacle_flg == 1:
      rospy.logdebug('障害物・アリを受信')
      pre_flag = self._obstacle_flag
      self._obstacle_flag = True

      # 手動モードの場合は速度ゼロのcmd_velを発行
      if self._manual_motion:
        if pre_flag == False and self._obstacle_flag:
          rospy.loginfo('手動モードにつき一度速度ゼロを指示')
          t = Twist()
          t.linear.x = 0.0
          t.linear.y = 0.0
          t.linear.z = 0.0
          t.angular.z = 0.0
          self.pub_cmd_vel.publish(t)
      else:
        # 自律走行モードで現時点で制御中であったら
        if self.obj_automove is not None:
          rospy.loginfo('自律走行停止要求')
          self.obj_automove.action_cancel()
    else:
      rospy.logdebug('障害物・ナシを受信')
      self._obstacle_flag = False

  # サブスクライバー制御ワーカー
  def sub_worker(self):
    rospy.Subscriber('remote_mode', RemoteMode, self.on_remote_mode)
    rospy.Subscriber('ext_joycontroller', ControllerEvent, self.on_joydata)
    rospy.Subscriber('remote_move', RemoteMove, self.on_remote_move)
    rospy.Subscriber(RemoteControlDefine.NS_CONTROLL_ROBOT+'/obstacles', Obstacle, self.on_obstacle_receive)

    while not rospy.is_shutdown():
      rospy.spin()

if __name__ == '__main__':
  rospy.loginfo('===遠隔制御コンポーネント開始===')
  rospy.init_node('remote_controller')

  # 遠隔制御メインクラスのインスタンス生成
  rm = RemoteMotion()

  try:
    # メインクラスでのSubscriber用スレッドを起動
    rm.sub_worker()
  except Exception as e:
    print(e)

  rospy.loginfo('===遠隔制御コンポーネント終了===')
