#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import IntEnum

"""
コントローラの種別
"""
class ControllerKind(IntEnum):
  CONTROLLER_PS4 = 0
  CONTROLLER_LOGICOOL = 1

"""
コントローラのボタンマッピング
"""
class ButtonMap(IntEnum):
  BUTTON_CROSS = 0
  BUTTON_CIRCLE = 1
  BUTTON_TRIANGLE = 2
  BUTTON_SQUARE = 3
  BUTTON_L1 = 4
  BUTTON_R1 = 5
  BUTTON_L2 = 6
  BUTTON_R2 = 7
  BUTTON_SHARE = 8
  BUTTON_OPTION = 9
  BUTTON_PS = 10
  BUTTON_L3 = 11
  BUTTON_R3 = 12

"""
コントローラのにおいて傾斜量などが存在する部位マッピング
"""
class AxesMap(IntEnum):
  AXES_LSTICK_ANGULAR = 0
  AXES_LSTICK_FB = 1
  AXES_L2 = 2
  AXES_RSTICK_LR = 3
  AXES_RSTICK_LR_LOGICOOL = 2
  AXES_R2 = 5
