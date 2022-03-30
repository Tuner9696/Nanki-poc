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

# 広告巡回ジョブ
# 巡回処理
# （１）リフターを下げる
# （２）巡回地へ移動する
# （３）到着したらリフターを上げる
# （４）１分経過  　　　　　　　　　←３６０度回転しない
# （５）最終巡回地ならジョブ停止
# （６）最終巡回地でないなら、（１）に戻る


class ADPATROL_ACTION(Enum):
  IDLE = 0,
  # 巡回地点へ移動する
  MOVEMENT = 1
  # 止まって巡回する
  ADVERTISEMENT = 2

class AdPatrolJob(Job):

  # コンストラクタ
  def __init__(self, id, points, ctx, dst_no):
    super(Job, self).__init__()
    self.robot_id = id
    self.destination_pos = points
    self.ctx = ctx

    # 巡回地リストの何番目から巡回開始するかを設定
    self.current_destination_no = dst_no

    self.current_action = ADPATROL_ACTION.IDLE

    # 巡回地リストの地点数を設定
    self.final = 0
    for row in self.destination_pos:
      self.final = self.final + 1

  # 次の目的地が巡回地リストの何番目かを取得
  def get_current_destination_no(self):
    return self.current_destination_no

  # ジョブ開始
  def job_start(self):
    rospy.loginfo('ROBOT{} : ----------広告巡回ジョブ開始----------'.format(self.robot_id))
    self.trans_req_flg = False
    self.joint_req_flg = False
    self.timer_flg = False
    self.counterThread = None
    rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(self.robot_id))
    self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_IDLE)
      # （１）リフターを下げる
    self.current_action = ADPATROL_ACTION.MOVEMENT
    self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_BOTTOM)

  # ジョブ停止前の後始末
  def cleanup(self):
    if self.counterThread != None:
      self.counterThread.cancel()
    
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
    n = self.current_destination_no - 1
    self.trs_exec(SystemDefine.RUN_KIND_NOAUTO, SystemDefine.TRANS_FLG, self.destination_pos[n])

  # リフター応答受信
  def _callback_joint(self, msg):
    if msg.robot_id == int(self.robot_id) and self.joint_req_flg == True:
      rospy.loginfo('ROBOT{} : リフター応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.joint_req_flg = False

      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return

      if self.current_action == ADPATROL_ACTION.MOVEMENT:
        # （２）巡回地へ移動する
        self.send_trsmng_req()
      
      elif self.current_action == ADPATROL_ACTION.ADVERTISEMENT:
        if self.robot_id == '001':
          rospy.loginfo('ROBOT{} : ディスプレイ表示 :案内先選択'.format(self.robot_id))
          self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_SELECT)
        # （４）１分経過
        rospy.loginfo('ROBOT{} : 1分経過start'.format(self.robot_id))
        self.counterThread = threading.Timer(SystemDefine.ADVERTISE_DURATION, self._timer_callback)
        self.counterThread.start()

  def _timer_callback(self):
    rospy.loginfo('ROBOT{} : 1分経過end'.format(self.robot_id))
    rospy.loginfo('ROBOT{} : 巡回地点 現在 {} : 最終 {}'.format(self.robot_id,self.current_destination_no,self.final))
    self.counterThread.cancel()
    rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(self.robot_id))
    self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_IDLE)

    if self.current_destination_no <= self.final:
      # （６）最終巡回地でないなら、（１）に戻る
      # （１）リフターを下げる
      self.current_action = ADPATROL_ACTION.MOVEMENT
      self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_BOTTOM)
    else:
      # （５）最終巡回地ならジョブ停止
      self.stop()

  # 移動応答受信
  def _callback_trs(self, msg):
    if msg.robot_id == int(self.robot_id) and self.trans_req_flg == True:
      rospy.loginfo('ROBOT{} : 移動応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.trans_req_flg = False
      # 到着したら次の目的地が巡回地リストの何番目かを設定
      self.current_destination_no = self.current_destination_no + 1

      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return

      if self.current_action == ADPATROL_ACTION.MOVEMENT:
        # （３）到着したらリフターを上げる
        self.current_action = ADPATROL_ACTION.ADVERTISEMENT
        self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_TOP)

