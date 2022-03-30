#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import sys, time, datetime
import threading
from enum import Enum
from system_define import SystemDefine, LifterControlKind
from db_access_helper import DestPoint
from mission.msg import DisplayStatus, TransManagerCommandReq, JointManagerCommandReq, ManagerCommandRes, JointManagerCommandRes

# 排他制御付print
#def print_lock( ctx, str):
#    with ctx["lock"]:
#        print( str)

class Job(threading.Thread):

  # コンストラクタ
  def __init__(self, id, points, ctx):
    super(Job, self).__init__()
    # ロボット識別子
    self.robot_id = id
    # ロボット識別子
    self.destination_pos = points
    # ジョブ停止トリガー（MissionとJobで共有するデータ）
    self.ctx = ctx

    self.sub_trans
    self.sub_joint

  def run(self):
    #rospy.loginfo('ROBOT{} : ----------ジョブ開始----------'.format(self.robot_id))

    # ROS Publisher生成
    #rospy.loginfo('ROBOT{} : Publisher生成'.format(self.robot_id))
    #self.pub_trans = rospy.Publisher('/trsmng_req', TransManagerCommandReq, queue_size=1)
    #self.pub_joint = rospy.Publisher('/jointmng_req', JointManagerCommandReq, queue_size=1)
    #self.pub_dspsts = rospy.Publisher('/' + SystemDefine.ROBOT_NAMESPACE_HEAD + self.robot_id + '/ext_display', DisplayStatus, queue_size=1)

    self.job_start()
    self.exec_sub()

    while True: # 無限ループ
        if self.ctx["stop"]: # main側から終了を指示されたら終了
            break
        time.sleep(0.5)
        #print_lock( self.ctx, "sub  : " + str(datetime.datetime.today()))

    self.cleanup()
    self.sub_trans.unregister()
    self.sub_joint.unregister()

    #rospy.loginfo('ROBOT{} : ----------ジョブ停止----------'.format(self.robot_id))

  # ジョブ開始
  def job_start(self):
    pass

  # ジョブ停止前の後始末
  def cleanup(self):
    pass

  # ジョブ停止
  def stop(self):
    self.ctx["stop"] = True

  # Subscriber登録
  def exec_sub(self):
    topic_name = '/jointmng_res'
    self.sub_joint = rospy.Subscriber(topic_name, JointManagerCommandRes, self._callback_joint)
    topic_name = '/trsmng_res'
    self.sub_trans = rospy.Subscriber(topic_name, ManagerCommandRes, self._callback_trs)
    #rospy.spin()
    self.sub_flg = True

  # リフター応答受信
  def _callback_joint(self, msg):
    pass

  # 移動応答受信
  def _callback_trs(self, msg):
    pass

  # リフター要求
  def send_jointmng_req(self, param):
    pass

  # リフター要求送信実行
  def joint_exec(self,param):
    if SystemDefine.LIFTER_CONTROL_KIND == LifterControlKind.KIND_CONTROL:
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
    else:
      self.joint_req_flg = True
      id = int(self.robot_id)
      _res = JointManagerCommandRes()
      _res.robot_id = id
      _res.result = 1
      self._callback_joint(_res)

  # 移動要求
  def send_trsmng_req(self):
    pass

    # 回転要求
  def send_turnmng_req(self, value):
    pass

    # 停止要求
  def send_stopmng_req(self):
    #rospy.loginfo('ROBOT{} : 停止要求送信'.format(self.robot_id))
    form_euler = DestPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self.trs_exec(SystemDefine.RUN_KIND_NOAUTO, SystemDefine.STOP_FLG, form_euler)

  # 移動要求送信実行
  def trs_exec(self,kind,flg,pos):
    pub_trans = rospy.Publisher('/trsmng_req', TransManagerCommandReq, queue_size=1)
    time.sleep(1)
    id = int(self.robot_id)
    _command = TransManagerCommandReq()
    _command.robot_id = id
    _command.trans_kind = kind
    _command.goal.trs_flg = flg
    if flg == SystemDefine.TRANS_FLG:
      #rospy.loginfo('ROBOT{} : 移動要求送信'.format(self.robot_id))
      _command.goal.point_info.transform.translation.x = pos.pos_x
      _command.goal.point_info.transform.translation.y = pos.pos_y
      _command.goal.point_info.transform.translation.z = pos.pos_z
      _command.goal.point_info.transform.rotation.ori_p =pos.ori_p
      _command.goal.point_info.transform.rotation.ori_r =pos.ori_r
      _command.goal.point_info.transform.rotation.ori_y =pos.ori_y
      _command.goal.whl_sp_info.linear.x = SystemDefine.CMDVEL_LINEAR_X
      _command.goal.whl_sp_info.linear.y = SystemDefine.CMDVEL_LINEAR_Y
      _command.goal.whl_sp_info.linear.z = SystemDefine.CMDVEL_LINEAR_Z
      _command.goal.whl_sp_info.angular.x = SystemDefine.CMDVEL_ANGULAR_X
      _command.goal.whl_sp_info.angular.y = SystemDefine.CMDVEL_ANGULAR_Y
      _command.goal.whl_sp_info.angular.z = SystemDefine.CMDVEL_ANGULAR_Z
    elif flg == SystemDefine.ROTATE_FLG:
      #rospy.loginfo('ROBOT{} : 回転要求送信'.format(self.robot_id))
      _command.goal.point_info.transform.translation.x = 0.0
      _command.goal.point_info.transform.translation.y = 0.0
      _command.goal.point_info.transform.translation.z = 0.0
      _command.goal.point_info.transform.rotation.ori_p = 0.0
      _command.goal.point_info.transform.rotation.ori_r = 0.0
      _command.goal.point_info.transform.rotation.ori_y = pos.ori_y
      _command.goal.whl_sp_info.linear.x = SystemDefine.CMDVEL_LINEAR_X
      _command.goal.whl_sp_info.linear.y = SystemDefine.CMDVEL_LINEAR_Y
      _command.goal.whl_sp_info.linear.z = SystemDefine.CMDVEL_LINEAR_Z
      _command.goal.whl_sp_info.angular.x = SystemDefine.CMDVEL_ANGULAR_X
      _command.goal.whl_sp_info.angular.y = SystemDefine.CMDVEL_ANGULAR_Y
      _command.goal.whl_sp_info.angular.z = SystemDefine.CMDVEL_ANGULAR_Z
    elif flg == SystemDefine.STOP_FLG:      
      #rospy.loginfo('ROBOT{} : 停止要求送信'.format(self.robot_id))
      _command.goal.point_info.transform.translation.x = 0.0
      _command.goal.point_info.transform.translation.y = 0.0
      _command.goal.point_info.transform.translation.z = 0.0
      _command.goal.point_info.transform.rotation.ori_p = 0.0
      _command.goal.point_info.transform.rotation.ori_r = 0.0
      _command.goal.point_info.transform.rotation.ori_y = 0.0
      _command.goal.whl_sp_info.linear.x = 0.0
      _command.goal.whl_sp_info.linear.y = 0.0
      _command.goal.whl_sp_info.linear.z = 0.0
      _command.goal.whl_sp_info.angular.x = 0.0
      _command.goal.whl_sp_info.angular.y = 0.0
      _command.goal.whl_sp_info.angular.z = 0.0
    else:
      return
    pub_trans.publish(_command)
    self.trans_req_flg = True

  # ディスプレイ表示送信
  def publish_display_status(self, id, status):
    #rospy.loginfo('ROBOT{} : ディスプレイ表示 :{}'.format(id,status))
    pub_dspsts = rospy.Publisher('/' + SystemDefine.ROBOT_NAMESPACE_HEAD + self.robot_id + '/ext_display', DisplayStatus, queue_size=1)
    time.sleep(1)
    _displayStatus = DisplayStatus()
    _displayStatus.status = status
    pub_dspsts.publish(_displayStatus)

  # デストラクタ  
  def __del__(self):
    pass



