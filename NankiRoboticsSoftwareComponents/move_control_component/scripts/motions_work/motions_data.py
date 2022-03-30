#!/usr/bin/env python
# -*- coding: utf-8 -*-
from threading import Thread
from threading import Lock
import rospy
from motion_control_define import MotionControlDefine
from motion_control_define import MotionKind
from motion_control_define import MotionAuto

'''
各モーションごとの情報
'''
class MotionGoal():
  def __init__(self):
    # モーション情報を初期化
    # 名前空間
    self.name_space = ''

    # モーション種別
    self.motion_kind = MotionKind.MOTION_KIND_MOVE

    # 目的地座標
    self.goal_x = 0.0
    self.goal_y = 0.0
    self.goal_z = 0.0

    # 目的角度
    self.goal_rpy_pitch = 0.0
    self.goal_rpy_role = 0.0
    self.goal_rpy_yaw = 0.0

'''
モーション情報クラス
'''
class MotionsData():
  # コンストラクタ
  def __init__(self, name_space='', motions_id=1, motion_auto=MotionAuto.MOTION_AUTO):
    # ジョブゴールリスト初期化
    self.motion_goal_lists = []

    # 名前空間保存
    self.name_space = name_space
    # モーション群のID保存
    self.motions_id = motions_id
    # ジョブ群の自動・手動を保存
    self.motion_auto = motion_auto

    # モーションゴール情報リスト排他オブジェクト生成
    self.lock_goal_lists = Lock()

  # モーションゴール情報追加
  def append_motion_goal(self, goal):
    # 排他をしてゴールを追加
    self.lock_goal_lists.acquire()
    self.motion_goal_lists.append(goal)
    self.lock_goal_lists.release()

  # リストからモーションゴール情報を削除
  def del_motion_goal(self, index=0):
    # 排他をして指定インデックスの情報を削除
    self.lock_goal_lists.acquire()
    del self.motion_goal_lists[index]
    self.lock_goal_lists.release()

  # モーションゴール情報リストをクリア
  def clear_motion_goal(self):
    self.lock_goal_lists.acquire()
    del self.motion_goal_lists[:]
    self.lock_goal_lists.release()
