#!/usr/bin/python
# -*- coding: utf-8 -*-
from enum import Enum

# リフター制御
class LifterControlKind(Enum):
  KIND_NOCONTROL = 1,   # 制御なし
  KIND_CONTROL = 2      # 制御あり

class SystemDefine():
  # システム全体の定義

  # リフター制御の種別
  # リフター制御問題対処により追加
  LIFTER_CONTROL_KIND = LifterControlKind.KIND_NOCONTROL

  # 巡回回数
  ADPATROL_TIMES = 3

  # タイマー設定値
  # 巡回地点での停止時間(秒)
  ADVERTISE_DURATION = 3
  # 訪問客が案内先を選択するのを待つ時間(秒)
  STANDBY_DURATION = 3
  # 到着表示時間(秒)
  ARRIVAL_DURATION = 3
  # 引き継ぎ表示時間(秒)
  HANDOVER_DURATION = 3

  # 座標情報tfのフレームにおけるmapへの名前空間
  MOTION_NS_FOR_MAP = ''

  # ロボット移動速度の指定
  CMDVEL_LINEAR_X = 0.2
  CMDVEL_LINEAR_Y = 0
  CMDVEL_LINEAR_Z = 0
  CMDVEL_ANGULAR_X = 0
  CMDVEL_ANGULAR_Y = 0
  CMDVEL_ANGULAR_Z = 0.2

  # 走行種別を示すフラグ値
  # 自律走行
  RUN_KIND_AUTO = 1
  # サーバ指示型走行
  RUN_KIND_NOAUTO = 2

  # 指示種別を示すフラグ値
  # 移動
  TRANS_FLG = 1
  # 回転
  ROTATE_FLG = 2
  # 停止
  STOP_FLG = 3

  # 結果
  # 移動/回転/停止正常終了
  RES_OK = 1
  # 移動/回転/停止異常終了
  RES_NG = 2

  # リフター制御指示
  # リフター位置最上部
  REQ_LIFTER_CTL_TOP = 1
  # リフター位置最下部
  REQ_LIFTER_CTL_BOTTOM = 2
  # リフター位置数値指定
  REQ_LIFTER_CTL_POSITION = 3
  # ※REQ_LIFTER_CTL_POSITIONは今回未使用。

  # ディスプレイ表示状態を示す値
  # 案内先選択
  DISPLAY_SELECT = 1
  # 案内引継ぎ
  DISPLAY_HANDOVER = 2
  # 案内先到着
  DISPLAY_ARRIVAL = 3
  # 無表示
  DISPLAY_IDLE = 4

  # 指令　ミッション指示 (遠隔制御)
  # 一時停止
  CMD_PAUSE = 1
  # 再開
  CMD_RESUME = 2

  # 指令　ミッション指示（クラウド）／ミッション指示（ユーザインタフェース）
  # 開始
  CMD_START = 1
  # 停止
  CMD_STOP = 2

  # ロボット状態
  # 広告巡回中
  ROBOT_STATUS_ADPATROL = 1
  # 案内中 
  ROBOT_STATUS_GUIDE = 2
  # アイドル
  ROBOT_STATUS_IDLE = 99

  # 障害物フラグ値
  # 障害物あり
  OBSTACLE_FLG_ON = 1
  # 障害物なし
  OBSTACLE_FLG_OFF = 0

  # ロボットネームスペース接頭部分
  ROBOT_NAMESPACE_HEAD = 'thk'

  