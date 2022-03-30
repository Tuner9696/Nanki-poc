#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import IntEnum

"""
リフター制御種別定義
"""
class LifterControlKind(IntEnum):
  KIND_PUBLISH = 1,   # Publish版
  KIND_MOVEIT = 2,    # MoveIt!版
  KIND_CUSTOMIZE = 3    # 加減速可能版

"""
モーション制御コンポーネントにおける共通定義
"""
class MotionControlDefine():
  # デバイスの移動の基準速度
  MOTION_MOVE_SPEED = 0.4
  # デバイスの転回の基準速度
  MOTION_ROTATION_SPEED = 0.4

  # リフターを上昇/降下させる時間
  LIFTER_MOVE_TIME = 2.0
  # Moverでの許容最高移動速度
  MOTION_THK_SPEED_MAX = 1.5

  # 移動速度変化に対するなまら値
  MOTION_SPEED_COEFFICIENT = 0.95

  # 移動時の到達判断範囲(メートル値)
  MOTION_MOVE_ARRAIVAL_RANGE = 0.1
  # 転回時の到達判断範囲(角度数値)
  MOTION_ROTATE_ARRAIVAL_RANGE = 3

  # 移動・転回時の座標確認間隔(ミリ秒)
  # 移動時の座標確認間隔
  MOTION_CHECK_MOVE_TF = 10
  # 転回時の座標確認間隔
  MOTION_CHECK_ROTATE_TF = 10 

  # 座標情報tfのフレームにおけるmapへの名前空間
  MOTION_NS_FOR_MAP = ''

  # 安全装置関連定義
  # 移動時オーバーラン緊急停止距離(メートル値)
  MOTION_MOVE_EMERGENCY_OVERRUN = 0.5

  # リフター制御の種別
  LIFTER_CONTROL_KIND = LifterControlKind.KIND_PUBLISH

  # リフターを上昇/降下させる時間（制御種別がPUBLISHの場合参照）
  LIFTER_MOVE_UP_TIME = 12.0
  LIFTER_MOVE_DOWN_TIME = 10.0

  # リフターを上昇/降下させる速度（制御種別がMOVEITの場合参照）
  LIFTER_MOVE_UP_VEL = 0.1    # 0.0 - 1.0
  LIFTER_MOVE_DOWN_VEL = 0.1  # 0.0 - 1.0

  # リフターの伸ばし値・上昇時（制御種別がPUBLISHとMOVEITの場合）
  LIFTER_MOVE_UP_ANKLE = 0.0
  LIFTER_MOVE_UP_KNEE = 0.0
  # リフターの曲げ値・下降時（制御種別がPUBLISHとMOVEITの場合）
  LIFTER_MOVE_DOWN_ANKLE = 1.5
  LIFTER_MOVE_DOWN_KNEE = -1.5

  # リフターの許容範囲
  LIFTER_MOVE_RANGE = 0.05

  # リフター制御コマンド送信数（ガード用送信）
  LIFTER_PUBLISH_RESEND = 3

  """
  リフター加減速時の動作定義群
  """
  # リフターの加減速版時の目標軸値（軸名：ankle_jointを対象）と制御時間
  # 
  # 設定の見方
  # ・辞書形式（キーと値の組み合わせ）になっており、
  # 　"目標軸値(Knee)": "制御時間"（float型:float型）を複数指定することが可能
  # ・Kneeは0.0(上昇時)から-1.0(下降時)で値変化となる
  # ・Ankleの目標軸値はKneeの正負反対となる
  # ・なお上記のリフターの許容範囲は考慮されず、下記のプレ動作範囲が考慮される
  LIFTER_ACCDCC_UP_LIST = [[-0.8, 6.0], [-0.6, 4.0], [-0.4, 2.0], [-0.2, 1.0], [0.0, 1.0]]
  LIFTER_ACCDCC_DOWN_LIST = [[-0.2, 2.0], [-0.4, 2.0], [-0.6, 2.0], [-0.8, 2.0], [-1.0, 2.0]]

  # リフターの加減速版時の次コマンド実施判断の範囲
  #  動作が完全に完了してから次のコマンド送信すると
  #  加速トルクが再度必要となってしまうのでそれを防止するため
  #  この値が大きいと目標軸値と制御時間の設定から外れてしまい
  #  小さいと目標到達で止まってしまい加速トルクが必要となってしまう
  LIFTER_ACCDCC_UP_PRERANGE = 0.1
  LIFTER_ACCDCC_DOWN_PRERANGE = 0.1

  # リフター加減速時にAnkleの遅延動作のフラグ
  LIFTER_ACCDCC_DELAY_ANKLE_FLAG = True
  # リフターの加減速時のAnkle動作開始のDelay時間（秒）
  #  実際にはこの倍の時間での制御とすることで処理に猶予を持たせ
  #  目的であるKneeのアクチュエータを最初に動かし、
  #  Ankleのそれを後から動作させることをより確実にする
  LIFTER_ACCDCC_DELAY_ANKLE = 1.0
  # リフターの加減速時のAnkle動作開始のDelayまでのKneeの動作量
  # この軸値動くという意味ではなく目標軸値移動量となる
  LIFTER_ACCDCC_DELAY_MOVERANGE = 0.3

  # バッテリー残量によるリフター制御ガード処理関連定義
  # ガード処理有効定義
  LIFTER_RESTRICTION_BY_VOLTAGE = False
  # ガード処理発動のバッテリー残量定義
  LIFTER_RESTRICTION_VOLTAGE_VALUE = 45
  # ガード処理としてのリフターの固定位置
  LIFTER_RESTRICTION_POSITION_TOP = True

"""
モーション制御自動・手動定義
"""
class MotionAuto(IntEnum):
  MOTION_AUTO = 1,
  MOTION_MANUAL = 2

"""
モーション制御種別定義
"""
class MotionKind(IntEnum):
  MOTION_KIND_MOVE = 1,
  MOTION_KIND_ROTATE = 2,
  MOTION_KIND_STOP = 3,
  MOTION_KIND_LIFTERUP = 10,
  MOTION_KIND_LIFTERDOWN = 11

"""
モーション結果定義
"""
class MotionResult(IntEnum):
  MOTION_RESULT_OK = 0,
  MOTION_RESULT_NG = -1,
