#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
from enum import Enum
from system_define import SystemDefine, LifterControlKind
from db_access_helper import DestPoint
from job import Job
from mission.msg import DisplayStatus, TransManagerCommandReq, JointManagerCommandReq, ManagerCommandRes, JointManagerCommandRes

# 開始位置移動ジョブ
# ①リフターを下げる
# ②開始位置へ移動する
# ③ジョブ停止

class StartPosMoveJob(Job):

  # コンストラクタ
  def __init__(self, id, points, ctx, flg):
    super(Job, self).__init__()
    self.robot_id = id
    self.destination_pos = points
    self.ctx = ctx
    self.run_kind_auto = flg

    self.current_destination_no = 1

  # 現在の目的地が何番目の巡回地かを取得
  def get_current_destination_no(self):
    return self.current_destination_no
    
  # ジョブ開始
  def job_start(self):
    rospy.loginfo('ROBOT{} : ----------開始位置移動ジョブ開始----------'.format(self.robot_id))

    self.trans_req_flg = False
    self.joint_req_flg = False

    self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_BOTTOM)
    #if SystemDefine.LIFTER_CONTROL_KIND == LifterControlKind.KIND_CONTROL:
    #  self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_BOTTOM)
    #else:
    #  # ミッション開始時のみリフター制御する
    #  if self.run_kind_auto == False:
    #    self.send_jointmng_particularreq(SystemDefine.REQ_LIFTER_CTL_BOTTOM)
    #  else:
    #    self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_BOTTOM)

  # ジョブ停止前の後始末
  def cleanup(self): 
    # ロボット002は移動中にジョブ停止する場合があるので、送信済みの要求の応答を待つ
    # ロボット001はリフター応答は待ってからジョブを停止する
    if self.robot_id == '002':
      while(self.trans_req_flg == True or self.joint_req_flg == True):
        time.sleep(0.5)
    else:
      while(self.joint_req_flg == True):
        time.sleep(0.5)

    rospy.loginfo('ROBOT{} : ----------ジョブ停止----------'.format(self.robot_id))
    pass

  # リフター要求
  def send_jointmng_req(self, param):
    rospy.loginfo('ROBOT{} : リフター要求送信({})'.format(self.robot_id,param))
    self.joint_exec(param)

  # 移動要求
  def send_trsmng_req(self):
    rospy.loginfo('ROBOT{} : 移動要求送信'.format(self.robot_id))
    if self.run_kind_auto == True:
      # サーバ指示型走行, 移動
      self.trs_exec(SystemDefine.RUN_KIND_NOAUTO, SystemDefine.TRANS_FLG, self.destination_pos[0])
    else:
      # 自立走行, 移動
      self.trs_exec(SystemDefine.RUN_KIND_AUTO, SystemDefine.TRANS_FLG, self.destination_pos[0])


  # リフター応答受信
  def _callback_joint(self, msg):
    if msg.robot_id == int(self.robot_id) and self.joint_req_flg == True:
      rospy.loginfo('ROBOT{} : リフター応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.joint_req_flg = False
      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return
      self.send_trsmng_req()

  # 移動応答受信
  def _callback_trs(self, msg):
    if msg.robot_id == int(self.robot_id) and self.trans_req_flg == True:
      rospy.loginfo('ROBOT{} : 移動応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.trans_req_flg = False
      self.current_destination_no = self.current_destination_no + 1
      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return
      self.stop()

  # リフター要求
  # リフター制御問題対処により追加
  def send_jointmng_particularreq(self, param):
    rospy.loginfo('ROBOT{} : リフター要求送信({})'.format(self.robot_id,param))
    self.joint_particularexec(param)

  # リフター要求送信実行
  # リフター制御問題対処により追加
  def joint_particularexec(self,param):
    pub_joint = rospy.Publisher('/jointmng_req', JointManagerCommandReq, queue_size=10)
    time.sleep(1)
    id = int(self.robot_id)
    _command = JointManagerCommandReq()
    _command.robot_id = id
    _command.lifter_control_kind = param
    #_command.lifter_control_kind = SystemDefine.REQ_LIFTER_CTL_TOP
    #_command.lifter_control_kind = SystemDefine.REQ_LIFTER_CTL_BOTTOM
    pub_joint.publish(_command)
    self.joint_req_flg = True
