#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import Enum
#os
import os

class SystemDefine():
  # システム全体の定義

  #メッセージの受信回数
  RECEIVED_MESSAGES = 0

  #Iot hub接続文字列
  CONNECTION_STRING = os.environ["CONNECTION_STRING"]

  #各ミッション制御メッセージ
  MISSION_TRIGGER_MESSAGE = "mission"
  MISSION_CONTROL_ON_MESSAGE = "True"
  MISSION_CONTROL_ON = True
  MISSION_CONTROL_OFF_MESSAGE = "False"
  MISSION_CONTROL_OFF = False
  
  #各遠隔制御制御メッセージ
  REMOTE_TRIGGER_MESSAGE = "remote_control"
  REMOTE_CONTROL_ON_MESSAGE = "True"
  REMOTE_CONTROL_ON = True
  REMOTE_CONTROL_OFF_MESSAGE = "False"
  REMOTE_CONTROL_OFF = False

  #ロボット状態番号
  ROBOT_STATUS_ADPATOL = 1  
  ROBOT_STATUS_GUIDE   = 2  
  ROBOT_STATUS_IDLE    = 99  

  #ロボット状態メッセージ
  STATUS_MESSAGE_ADPATOL = "ADVERTISING"
  STATUS_MESSAGE_GUIDE = "GUIDE"
  STATUS_MESSAGE_IDLE = "IDLE_STOP"

  #案内地点メッセージ
  GUIDE_TRIGGER_MESSAGE = "destination_no"

  #デバイス座標(tf)でのフレーム:mapでの付与名前空間
  NAMESPACE_FOR_MAP = ''

##########################################################################
#
#        単体試験用メッセージ。以下の変更して対応してください。  
#
########################################################################## 
  #遠隔制御制御移動座標
  REMOTE_CONTROL_POSITION_X = 1
  REMOTE_CONTROL_POSITION_Y = 2
  REMOTE_CONTROL_POSITION_DEGREE = 3
  #遠隔制御制御トリガー。基本はFalseにしときますが、遠隔制御をするときは変更してください
  
  #REMOTE_CONTROL_TEST = True
  REMOTE_CONTROL_TEST = False
  
  #ミッション開始トリガー。基本はTrueにしときますが、状況によって変更してください

  MISSION_CONTROL_TEST = True
  #MISSION_CONTROL_TEST = False




