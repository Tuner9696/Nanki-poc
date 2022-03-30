#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
from enum import Enum
from system_define import SystemDefine
from db_access_helper import DestPoint
from job import Job
from mission.msg import DisplayStatus, TransManagerCommandReq, JointManagerCommandReq, ManagerCommandRes, JointManagerCommandRes

# 障害物正対ジョブ
# （０）停止
# （１）回転する   ★やらない
# （２）リフターを上げる
# （３）タブレット表示
# （４）待機（〇分）
# （５）待機タイムアウトしたら、巡回方向に回転する   ★やらな
# （６）ジョブ停止

class StandFrontJob(Job):

  # コンストラクタ
  def __init__(self, id, points, ctx, jobtype):
    super(Job, self).__init__()
    self.robot_id = id
    self.destination_pos = points
    self.ctx = ctx
    # 障害物正対前のジョブ
    self.before_jobtype = jobtype

    # 案内先選択待機中
    self.guide_pos_waiting = False

    # 案内先選択待機タイマー
    self.standbyThread = None

  # ジョブ開始
  def job_start(self):
    rospy.loginfo('ROBOT{} : ----------障害物正対ジョブ開始----------'.format(self.robot_id))
    self.trans_req_flg = False
    self.joint_req_flg = False
    self.guide_pos_waiting = False
    # （０）停止
    # 停止要求送信（応答はdon't care）
    rospy.loginfo('ROBOT{} : 停止要求送信'.format(self.robot_id))
    self.send_stopmng_req()

  # ジョブ停止前の後始末
  def cleanup(self):
    if self.standbyThread != None:
      self.standbyThread.cancel()
    
    # 送信済みの要求の応答を待つ
    while(self.trans_req_flg == True or self.joint_req_flg == True):
      time.sleep(0.5)

    rospy.loginfo('ROBOT{} : ----------ジョブ停止----------'.format(self.robot_id))

  # リフター要求
  def send_jointmng_req(self, param):
    rospy.loginfo('ROBOT{} : リフター要求送信({})'.format(self.robot_id,param))
    self.joint_exec(param)

  # リフター応答受信
  def _callback_joint(self, msg):
    if msg.robot_id == int(self.robot_id) and self.joint_req_flg == True:
      rospy.loginfo('ROBOT{} : リフター応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.joint_req_flg = False
      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return
      # （３）タブレット表示
      rospy.loginfo('ROBOT{} : ディスプレイ表示 :案内先選択'.format(self.robot_id))
      self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_SELECT)
      # （４）待機（〇分）
      # 案内先選択待機タイマー
      self.guide_pos_waiting = True
      self.standbyThread = threading.Timer(SystemDefine.STANDBY_DURATION, self._timer_stdby_callback)
      self.standbyThread.start()

  # 移動応答受信
  def _callback_trs(self, msg):
    if msg.robot_id == int(self.robot_id) and self.trans_req_flg == True:
      rospy.loginfo('ROBOT{} : 移動応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.trans_req_flg = False
      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return
      # （２）リフターを上げる
      self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_TOP)

  # 案内先選択待機タイマーコールバック
  def _timer_stdby_callback(self):
    rospy.loginfo('ROBOT{} : 待機タイムアウト'.format(self.robot_id))
    self.standbyThread.cancel()
    self.guide_pos_waiting = False
    # （６）ジョブ停止
    self.stop()





