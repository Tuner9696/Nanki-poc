#!/usr/bin/env python
# -*- coding: utf-8 -*-
from motions_work.motions_data import MotionsData
from motions_work.motions_data import MotionGoal
from motions_work.motions_worker import MotionsWorker
import rospy
from motion_control_define import MotionControlDefine, MotionResult
from motion_control_define import MotionAuto
from motion_control_define import MotionKind
from threading import Thread
from move_control_component.msg import TransReq
from move_control_component.msg import TransRes
from move_control_component.msg import LifterControlReq
from move_control_component.msg import LifterControlRes

# 対象デバイス名リスト（＝名前空間）
device_name_lists = ['thk001', 'thk002']

"""
デバイス制御コンポーネントのメインクラス
移動管理コンポーネントからのモーション要求を受信し
それに合わせた処理を実行

コンストラクタで指定するデバイスの名前空間の例：'/robot001'
"""
class DeviceMotionControl():
  # コンストラクタ
  def __init__(self, name_space):
    rospy.loginfo('DeviceMotionControlクラス開始')
    # プロパティ初期化
    # 名前空間
    self._name_space = name_space
    # モーション制御中フラグ
    self._flag_motioning = False
    # モーション中止受信フラグ
    self._flag_cancel_motions = False
    # モーション制御実施スレッドオブジェクト
    self.obj_execute_motions = None
    # リフター制御フラグ
    self._flag_control_lifter = False

    # 上位管理コンポーネントへの結果送信用Publisher生成
    rospy.loginfo('結果送信用パブリッシャー作成')
    self.pub_sendresult = rospy.Publisher(self._name_space + '/trs_res', TransRes, queue_size=10)
    self.pub_sendresult_lifter = rospy.Publisher(self._name_space + '/lifter_control_res', LifterControlRes, queue_size=10)

  # 名前空間プロパティ
  @property
  def name_space(self):
    return self._name_space
  @name_space.setter
  def name_space(self, value):
    self._name_space = value

  # モーション制御処理中フラグ
  @property
  def flag_motioning(self):
    return self._flag_motioning
  @flag_motioning.setter
  def flag_motioning(self, value):
    self._flag_motioning = value

  # モーション中止受信フラグ
  @property
  def flag_cancel_motions(self):
    return self._flag_cancel_motions
  @flag_cancel_motions.setter
  def flag_cancel_motions(self, value):
    self._flag_cancel_motions = value

  # リフター制御フラグ
  @property
  def flag_control_lifter(self):
    return self._flag_control_lifter
  @flag_control_lifter.setter
  def flag_control_lifter(self, value):
    self._flag_control_lifter = value

  # 管理コンポーネントからのモーション実行指示コールバック関数
  def on_trs_req(self, trans_req):
    rospy.loginfo('管理コンポーネントからの実行指示コールバック関数')
    # モーション制御中である場合、
    # 停止指示以外は処理なし
    if self._flag_motioning:
      if trans_req.route_ps_info[0].trs_flg == 3:
        rospy.loginfo('モーション制御中止指示受信')
        # リフター制御中の場合は無視
        if self._flag_control_lifter:
          return

        # モーション制御中止受信フラグON
        self._flag_cancel_motions = True
        # モーション制御ワーカスレッドへ中止要求
        if self.obj_execute_motions is not None:
          rospy.loginfo('モーション制御ワーカースレッドへ中止要求')
          self.obj_execute_motions.cancel_work()
        return
      else:
        rospy.loginfo('モーション制御中のため要求無視')
        return

    # 指示種別により分岐
    # 指示されたモーション群のデータから実行リストを生成
    # リフター制御フラグOFF
    self._flag_control_lifter = False
    rospy.loginfo('受信情報概略:seq={}, auto={}'.format(trans_req.trans_seq, trans_req.trans_kind))
    md = MotionsData(self._name_space,
                  trans_req.trans_seq,
                  MotionAuto.MOTION_AUTO if trans_req.trans_kind== 1 else MotionAuto.MOTION_MANUAL)

    rospy.loginfo('モーション情報抽出するためのループ開始')

    # モーション情報抽出のためのループ
    for info in trans_req.route_ps_info:
      mg = MotionGoal()
      # モーション種別設定
      if info.trs_flg == 1:
        mg.motion_kind = MotionKind.MOTION_KIND_MOVE
      else:
        mg.motion_kind = MotionKind.MOTION_KIND_ROTATE
      # ゴール情報設定
      mg.goal_x = info.point_info.transform.translation.x
      mg.goal_y = info.point_info.transform.translation.y
      mg.goal_z = info.point_info.transform.translation.z
      mg.goal_rpy_pitch = info.point_info.transform.rotation.ori_p
      mg.goal_rpy_role = info.point_info.transform.rotation.ori_r
      mg.goal_rpy_yaw = info.point_info.transform.rotation.ori_y

      # モーションゴール情報を登録
      rospy.loginfo('モーションゴール:trs_flg={}, goal_x={}, goal_y={}, goal_z={}, goal_rpy_pitch={}, goal_rpy_role={}, goal_rpy_yaw={}'.format(
                    info.trs_flg, mg.goal_x, mg.goal_y, mg.goal_z, mg.goal_rpy_pitch, mg.goal_rpy_role, mg.goal_rpy_yaw))
      md.append_motion_goal(mg)

    # モーション実行ワーカースレッド起動
    self._flag_motioning = True
    self.obj_execute_motions = MotionsWorker(md, self.on_end_work)
    self.obj_execute_motions.start()

  # 管理コンポーネントからのリフター制御指示コールバック関数
  def on_lifter_req(self, lifter_req):
    rospy.loginfo('管理コンポーネントからのリフター制御指示コールバック関数')
    # リフター制御フラグON
    self._flag_control_lifter = True

    # モーション制御中である場合は無視
    if self._flag_motioning:
      rospy.loginfo('モーション制御中のため要求無視')
      return

    # 指示種別により分岐
    # 指示されたモーション群のデータから実行リストを生成
    rospy.loginfo('受信情報 リフター制御種別:{}'.format(lifter_req.lifter_control_kind))
    md = MotionsData(self._name_space, 1, MotionAuto.MOTION_AUTO)
    # リフター制御としてのゴール情報設定
    mg = MotionGoal()
    # モーション種別設定
    if lifter_req.lifter_control_kind == 1:
      mg.motion_kind = MotionKind.MOTION_KIND_LIFTERUP
    else:
      mg.motion_kind = MotionKind.MOTION_KIND_LIFTERDOWN

    md.append_motion_goal(mg)

    # モーション実行ワーカースレッド起動
    self._flag_motioning = True
    self.obj_execute_motions = MotionsWorker(md, self.on_end_work_lifter)
    self.obj_execute_motions.start()

  # モーション実行結果受信コールバック
  def on_end_work(self, result, detail, motions_id, comped_index):
    rospy.loginfo('モーション実行結果受信処理開始')
    # 上位管理コンポーネントへ結果送信
    res = TransRes()
    # シーケンスID設定
    res.trans_seq = motions_id
    # ワーカー側からの結果による上位への結果通知設定
    if self._flag_cancel_motions:
      rospy.loginfo('モーション中止指示受信済み')
      # 完了したインデックスからリルート番号も設定
      res.trs_result = 20
      res.reroute_index = (comped_index + 1)
    else:
      if result == MotionResult.MOTION_RESULT_OK:
        res.trs_result = 2
      else:
        # 完了したインデックスからリルート番号も設定
        res.trs_result = 20
        res.reroute_index = (comped_index + 1)

    rospy.loginfo('上位管理コンポーネントへ結果通知:seq={}, result={}, reroute_index={}'.format(res.trans_seq, res.trs_result, res.reroute_index))
    self.pub_sendresult.publish(res)

    # モーション制御処理中フラグクリア
    self._flag_motioning = False
    # モーション中止受信フラグクリア
    self._flag_cancel_motions = False
    # モーション制御実施スレッドオブジェクト初期化
    self.obj_execute_motions = None

  # リフター制御実行結果受信コールバック
  def on_end_work_lifter(self, result, detail, motions_id, comped_index):
    rospy.loginfo('リフター制御結果受信処理開始')
    # 上位管理コンポーネントへ結果送信
    res = LifterControlRes()
    if result == MotionResult.MOTION_RESULT_OK:
      res.lifter_control_result = 1
    else:
      res.lifter_control_result = 2

    rospy.loginfo('上位管理コンポーネントへ結果通知:result={}'.format(res.lifter_control_result))
    self.pub_sendresult_lifter.publish(res)

    # モーション制御処理中フラグクリア
    self._flag_motioning = False
    # モーション中止受信フラグクリア
    self._flag_cancel_motions = False
    # モーション制御実施スレッドオブジェクト初期化
    self.obj_execute_motions = None

  # サブスクライバー制御ワーカー
  def sub_worker(self):
    # 上位管理コンポーネントからのモーション要求のサブスクライバー生成
    rospy.Subscriber(self._name_space + '/trs_req', TransReq, self.on_trs_req)
    rospy.Subscriber(self._name_space + '/joint_ctl_req', LifterControlReq, self.on_lifter_req)

    while not rospy.is_shutdown():
      rospy.spin()

if __name__ == '__main__':
  rospy.loginfo('===モーション制御コンポーネント開始===')
  rospy.init_node('move_control')

  th_lists = []

  # 対象デバイス分ループ
  for device_name in device_name_lists:
    # モーション制御メインクラスのインスタンス生成
    prefix_name = '' if device_name == '' else '/'
    rm = DeviceMotionControl(prefix_name + device_name)

    # デバイスのサブスクリプションワーカースレッド起動
    rospy.loginfo('デバイス：{} サブスクリプションワーカースレッド起動'.format(device_name))
    th_sub = Thread(target=rm.sub_worker)
    th_sub.start()
    th_lists.append(th_sub)

  while not rospy.is_shutdown():
    rospy.spin()

  rospy.loginfo('===モーション制御コンポーネント終了===')
