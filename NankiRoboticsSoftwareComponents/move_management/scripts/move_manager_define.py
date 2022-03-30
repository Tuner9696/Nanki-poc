#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
移動管理コンポーネント共通定義
"""
class MoveManagementDefine():
  # 状態遷移
  DEVICE_STATE = ''
  #ルート情報格納用リスト
  ROUTE_LIST = []
  #回転角度[rad]
  ROTADE_RAD = 0.785
  #停止時間[sec]
  WAIT_TIME = 3
  #45度回転が終わるのを待つサイクル[sec]
  TURN_WAIT_TIME = 0.5
