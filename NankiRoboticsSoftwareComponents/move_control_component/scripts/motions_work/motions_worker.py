#!/usr/bin/env python
# -*- coding: utf-8 -*-
from concurrent.futures import thread
from threading import Thread
from threading import Event
from motion_control_define import MotionAuto
from motion_control_define import MotionKind
from motion_control_define import MotionResult
from motions_work.motion_auto_move import MotionAutoMove
from motions_work.motion_manual_move import MotionManualMove
from motions_work.motion_manual_rotate import MotionManualRotate
from motions_work.motion_lifter import MotionLifter
import rospy

'''
モーション実行ワーカースレッドクラス
指定されたモーション群を実行
'''
class MotionsWorker(Thread):
  def __init__(self, motions_data, callback_end_work=None):
    Thread.__init__(self)

    # モーション情報を保存
    self.motions_data = motions_data
    # 実行フラグON
    self._flag_execute_motions = True
    # 実行中モーションインデックス初期化
    self._motion_index = 0
    # 実行中モーションオブジェクト初期化
    self.obj_current_motion = None
    # 最終モーション実行結果初期化
    self.last_motion_result = MotionResult.MOTION_RESULT_OK
    # モーション終了待ちイベントオブジェクト生成
    self.event_end_motion = Event()
    # モーション実行結果通知コールバック関数保存
    self.callback_end_work = callback_end_work

  # 実行フラグプロパティ
  @property
  def flag_execute_motions(self):
    return self._flag_execute_motions
  @flag_execute_motions.setter
  def flag_execute_motions(self, value):
    self._flag_execute_motions = value

  # 実行中モーションインデックス
  @property
  def motion_index(self):
    return self._motion_index
  @motion_index.setter
  def motion_index(self, value):
    self._motion_index = value

  # モーション実行ワーカーメソッド
  def run(self):
    rospy.loginfo('ジョブワーカースレッド開始')
    # モーションを1つずつ取り出して実行
    # 実行フラグがFlaseの場合は中断でエラー返信
    for motion_goal in self.motions_data.motion_goal_lists:
      # 移動か転回かを判定
      if motion_goal.motion_kind == MotionKind.MOTION_KIND_MOVE:
        rospy.loginfo('移動のモーション群として実行')
        # 自動か手動かを判定
        if self.motions_data.motion_auto == MotionAuto.MOTION_AUTO:
          # ゴールに対し自律移動
          rospy.loginfo('自律移動実行')
          rospy.loginfo('自律移動ゴール情報:{}'.format(motion_goal))
          self.obj_current_motion = MotionAutoMove(motion_goal, self.motions_data.name_space, self.on_end_motion)
          # モーション終了待ち
          self.event_end_motion.wait()
          self.event_end_motion.clear()
          self.obj_current_motion = None
        else:
          # ゴールに対し手動移動
          rospy.loginfo('手動移動実行')
          rospy.loginfo('手動移動ゴール情報:{}'.format(motion_goal))
          self.obj_current_motion = MotionManualMove(motion_goal, self.motions_data.name_space, self.on_end_motion)
          # モーション終了待ち
          self.event_end_motion.wait()
          self.event_end_motion.clear()
          self.obj_current_motion = None
      elif motion_goal.motion_kind == MotionKind.MOTION_KIND_ROTATE:
        # ゴールに対し手動転回
        rospy.loginfo('手動転回実行')
        rospy.loginfo('手動移動ゴール情報:{}'.format(motion_goal))
        self.obj_current_motion = MotionManualRotate(motion_goal, self.motions_data.name_space, self.on_end_motion)
        # モーション終了待ち
        self.event_end_motion.wait()
        self.event_end_motion.clear()
        self.obj_current_motion = None
      elif motion_goal.motion_kind == MotionKind.MOTION_KIND_LIFTERDOWN \
            or motion_goal.motion_kind == MotionKind.MOTION_KIND_LIFTERUP:
        # リフター制御
        rospy.loginfo('リフター制御実行')
        rospy.loginfo('リフター制御情報:{}'.format(motion_goal))
        self.obj_current_motion = MotionLifter(motion_goal, self.motions_data.name_space, self.on_end_motion)
        # モーション終了待ち
        self.event_end_motion.wait()
        self.event_end_motion.clear()
        self.obj_current_motion = None
      else:
        rospy.loginfo('★★★★★ 受信処理種別:{}'.format(motion_goal.motion_kind))

      # 実行フラグチェック
      if self._flag_execute_motions is not True:
        rospy.loginfo('実行フラグOFFのためループ抜け')
        break

      # モーション群のインデックス更新
      rospy.loginfo('モーション群のインデックス更新')
      self._motion_index = self._motion_index + 1

    # メインスレッドへ結果通知
    rospy.loginfo('最終モーション実行結果:{}'.format(self.last_motion_result))
    if self.callback_end_work is not None:
      self.callback_end_work(self.last_motion_result, '', self.motions_data.motions_id, self._motion_index)

    # ジョブ実行フラグをOFF
    self._flag_execute_motions = False

  # モーション実行スレッドからのコールバック関数
  def on_end_motion(self, result, detail):
    rospy.loginfo('モーション終了コールバック')
    # 最終実行結果保存
    self.last_motion_result = result

    # モーション実行結果によって処理分け
    if result == MotionResult.MOTION_RESULT_OK:
      rospy.loginfo('モーション正常終了')
      self.event_end_motion.set()
    else:
      rospy.loginfo('モーション失敗')
      # ジョブ実行フラグをOFF
      self._flag_execute_motions = False
      self.event_end_motion.set()

  # 実行中止指示
  def cancel_work(self):
    rospy.loginfo('メインスレッドからモーション実行中止指示受信')
    # 実行中でなければ即リターン
    if self._flag_execute_motions is not True:
      rospy.loginfo('実行中でないため無視')
      return
    
    # 現モーション実行オブジェクトへ中止指示
    if self.obj_current_motion is not None:
      self.obj_current_motion.motion_cancel()
