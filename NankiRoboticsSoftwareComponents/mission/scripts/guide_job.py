#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import threading
import math
import time
import tf
from enum import Enum
from system_define import SystemDefine
from db_access_helper import DestPoint
from job import Job
from mission.msg import DisplayStatus, TransManagerCommandReq, JointManagerCommandReq, ManagerCommandRes, JointManagerCommandRes

# 案内ジョブ（目的地は案内先）
# （１）リフターを下げる
# （２）案内先へ移動する
# （３）到着したら180度回転する
# （４）リフターを上げる
# （５）タブレット表示
# （６）〇秒経過
# （７）タブレット表示消去
# （８）ジョブ停止

class GUIDE_ACTION(Enum):
  IDLE = 0,
  # 案内先へ移動する
  GOTO= 1
  # 180度回転する
  TURN = 2
  # 到着を表示する
  ARRIVAL = 3

class GuideJob(Job):

  # コンストラクタ
  def __init__(self, id, points, ctx):
    super(Job, self).__init__()
    self.robot_id = id
    self.destination_pos = points
    self.ctx = ctx

    self.current_action = GUIDE_ACTION.IDLE
    self.arrivalThread = None

  # ジョブ開始
  def job_start(self):
    rospy.loginfo('ROBOT{} : ----------案内ジョブ（目的地は案内先）開始----------'.format(self.robot_id))
    self.trans_req_flg = False
    self.joint_req_flg = False
    rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(self.robot_id))
    self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_IDLE)
    # （１）リフターを下げる
    self.current_action = GUIDE_ACTION.GOTO
    self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_BOTTOM)

  # ジョブ停止前の後始末
  def cleanup(self):
    if self.arrivalThread != None:
      self.arrivalThread.cancel()
    
    # 送信済みの要求の応答を待つ
    while(self.trans_req_flg == True or self.joint_req_flg == True):
      time.sleep(0.5)

    rospy.loginfo('ROBOT{} : ----------ジョブ停止----------'.format(self.robot_id))

  # リフター要求
  def send_jointmng_req(self, param):
    rospy.loginfo('ROBOT{} : リフター要求送信({})'.format(self.robot_id,param))
    self.joint_exec(param)

  # 移動要求
  def send_trsmng_req(self):
    rospy.loginfo('ROBOT{} : 移動要求送信'.format(self.robot_id))
    # サーバ指示型走行, 移動
    self.trs_exec(SystemDefine.RUN_KIND_NOAUTO, SystemDefine.TRANS_FLG, self.destination_pos[0])

    # 回転要求
  def send_turnmng_req(self, value):
    rospy.loginfo('ROBOT{} : 回転要求送信'.format(self.robot_id))
    form_euler = DestPoint(0.0, 0.0, 0.0, 0.0, 0.0, value)
    self.trs_exec(SystemDefine.RUN_KIND_NOAUTO, SystemDefine.ROTATE_FLG, form_euler)

  # リフター応答受信
  def _callback_joint(self, msg):
    if msg.robot_id == int(self.robot_id) and self.joint_req_flg == True:
      rospy.loginfo('ROBOT{} : リフター応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.joint_req_flg = False
      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return
      if self.current_action == GUIDE_ACTION.GOTO:
        # （２）案内先へ移動する
        self.send_trsmng_req()
      elif self.current_action == GUIDE_ACTION.ARRIVAL:
        # （５）タブレット表示
        rospy.loginfo('ROBOT{} : ディスプレイ表示 :案内先到着'.format(self.robot_id))
        self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_ARRIVAL)
        # 到着表示タイマー
        self.guide_pos_waiting = True
        self.arrivalThread = threading.Timer(SystemDefine.ARRIVAL_DURATION, self._timer_arvl_callback)
        self.arrivalThread.start()        

  # 移動応答受信
  def _callback_trs(self, msg):
    if msg.robot_id == int(self.robot_id) and self.trans_req_flg == True:
      rospy.loginfo('ROBOT{} : 移動/回転応答受信 結果 :{}'.format(self.robot_id,msg.result))
      self.trans_req_flg = False
      if self.ctx["stop"]: # main側から終了を指示されていたら即終了
        return
      if self.current_action == GUIDE_ACTION.GOTO:
        # （３）到着したら180度回転する
        self.current_action = GUIDE_ACTION.TURN
        # 現在姿勢から目標の回転角度算出
        target = self.cul_rotate_degree(self.robot_id, 180)
        v = math.radians(target)
        rospy.loginfo('ROBOT{} : 180度回転 :目標角度 {}度 {}ラジアン'.format(self.robot_id,target,v))
        self.send_turnmng_req(v)
      elif self.current_action == GUIDE_ACTION.TURN:
        # （４）リフターを上げる
        self.current_action = GUIDE_ACTION.ARRIVAL
        self.send_jointmng_req(SystemDefine.REQ_LIFTER_CTL_TOP)

  # 到着表示タイマーコールバック
  def _timer_arvl_callback(self):
    rospy.loginfo('ROBOT{} : 到着表示時間タイムアウト'.format(self.robot_id))
    self.arrivalThread.cancel()
    rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(self.robot_id))
    self.publish_display_status(self.robot_id, SystemDefine.DISPLAY_IDLE)
    # （８）ジョブ停止
    self.stop()

  # 回転角度算出
  def cul_rotate_degree(self, robot_id, goal_degree):
    ret_degree = goal_degree

    # tf取得ノード生成
    listener = tf.TransformListener()

    nameapace_str = SystemDefine.ROBOT_NAMESPACE_HEAD + robot_id

    # tf取得可能になったらtfを取得する
    try:
      nameapace_str = SystemDefine.ROBOT_NAMESPACE_HEAD + robot_id
      listener.waitForTransform(SystemDefine.MOTION_NS_FOR_MAP+'/map', '/'+nameapace_str+'/base_link', rospy.Time(0), rospy.Duration(3.0))
      (trans, rot) = listener.lookupTransform(SystemDefine.MOTION_NS_FOR_MAP+'/map', '/'+nameapace_str+'/base_link', rospy.Time(0))
    except:
      rospy.logerr('ROBOT{} : tf取得失敗'.format(self.robot_id))
      # tf取得失敗で処理中断
      return ret_degree
    rospy.loginfo('ROBOT{} : 現在座標 postion:{}, {}, {} rotation:{}, {}, {}, {}'.format(self.robot_id,trans[0], trans[1], trans[1], rot[0], rot[1], rot[2], rot[3]))

    # クォータニオンから角度(度数)とヨー値へ変換
    euler = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    current_degree = math.degrees(euler[2])
    current_yaw = euler[2]

    # 右回りなので負の向き
    # 実際には移動制御がその時の正確な姿勢で近回りするので左右どちらの回転になるかは不定
    ret_degree = current_degree - goal_degree

    return ret_degree


