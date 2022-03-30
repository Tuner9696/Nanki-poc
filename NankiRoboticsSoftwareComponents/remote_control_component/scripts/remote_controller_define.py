#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
遠隔制御コンポーネントにおける共通定義
"""
class RemoteControlDefine():
  # Moverの最高速度
  MOVER_SPEED_MAX_FORWARD = 0.2
  MOVER_SPEED_MAX_BACKWARD = 0.2
  MOVER_SPEED_MAX_LEFTMOVE = 0.2
  MOVER_SPEED_MAX_RIGHTMOVE = 0.2
  MOVER_SPEED_MAX_ROTATE = 0.2

  # Moverの速度変化に対するなまら値
  MOVER_SPEED_COEFFICIENT = 0.95

  # リフターを上昇/降下させる時間
  LIFTER_MOVE_UP_TIME = 12.0
  LIFTER_MOVE_DOWN_TIME = 10.0

  # リフターの伸ばし値
  LIFTER_MOVE_UP_ANKLE = 0.0
  LIFTER_MOVE_UP_KNEE = 0.0
  # リフターの曲げ値
  LIFTER_MOVE_DOWN_ANKLE = 1.0
  LIFTER_MOVE_DOWN_KNEE = -1.0

  # 制御するロボットの名前空間
  NS_CONTROLL_ROBOT = 'thk001'

  # 障害検知対応フラグ
  FLAG_OBSTACLE_STOP = True
