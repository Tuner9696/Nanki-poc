#!/usr/bin/env python
# -*- coding: utf-8 -*-
from threading import Thread
import rospy
import tf2_ros
from observer_define import ThroughputDefine
import logging

'''
ROSのスループット（と言ってもtfの伝達時間）を
測定するワーカークラス
'''
class RosThroughputWoker():
  # コンストラクタ
  def __init__(self, parent_link, child_link):
    self.parent_link = parent_link
    self.child_link = child_link

  # 測定実体関数
  def sub_worker(self):
    # tf2_rosによる座標取得インスタンス生成
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # 座標取得タイムアウト単位合わせ
    timeout = ThroughputDefine.SAMPLING_TIMEOUT / 1000

    # サンプリング周期設定
    if ThroughputDefine.SAMPLE_DURATION <= 5:
      rospy.logerr('サンプリング周期が5以下で実行不可')
    rate = rospy.Rate(1/(ThroughputDefine.SAMPLE_DURATION/1000))

    # tf取得エラーカウンター初期化
    counter_tf_timeout = 0

    # ブロック内平均到達時間
    avg_space_time = 0
    # ブロック警告連続回数
    avg_warning_count = 0

    # ブロック内伝達時間トータル
    total_space_time = 0

    recieve_time = 0.0
    send_time = 0.0

    while not rospy.is_shutdown():
      # データ保存クラスインスタンス生成
      obj_rtd = RosThroughputData()
      # ファイル出力ブロック数ループ
      for block_count in range(ThroughputDefine.EXPORT_FILE_BLOCK_COUNT):
        avg_space_time = 0
        total_space_time = 0

        for sampling_count in range(ThroughputDefine.SAMPLING_DATA_COUNT):
          if rospy.is_shutdown():
            break
          try:
            trans = tfBuffer.lookup_transform(self.parent_link, self.child_link, rospy.Time(0), rospy.Duration(timeout))
          except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('サンプリング情報取得タイムアウト 基本Link:{} 子Link:{}'.format(self.parent_link, self.child_link))
            counter_tf_timeout+1

            avg_space_time = 0
            avg_warning_count = 0

            sampling_count = 0
            continue

          # 取得時間を採取
          now = rospy.Time.now()

          # 各時間を整形
          recieve_time = now.secs + round(float(now.nsecs) / 10**9, 3)
          send_time = trans.header.stamp.secs + round(float(trans.header.stamp.nsecs) / 10**9, 3)
          space_time = round(recieve_time - send_time, 3)

          # 伝達時間平均を算出
          total_space_time = total_space_time + space_time
          avg_space_time = round(total_space_time / (sampling_count + 1) * 1000)

          # サンプリング周期スリープ
          rate.sleep()

        obj_rtd.regist_data(recieve_time, avg_space_time)

        # 警告レベルアラーム出力判断
        if avg_space_time >= ThroughputDefine.ROS_THROUGHPUT_ALARM:
          rospy.logwarn("ROS伝達速度警告レベル超過: {}".format(avg_space_time))
          avg_warning_count = avg_warning_count + 1
          # エラーアラーム出力判断
          if avg_warning_count >= ThroughputDefine.ERROR_NOTIFY_WARNINGCOUNT:
            rospy.logerr("ROS伝達速度エラー発生")
            avg_warning_count = 0
        else:
          if avg_warning_count >= ThroughputDefine.ERROR_NOTIFY_WARNINGCOUNT:
            rospy.loginfo("ROS伝達速度エラー回復")
            avg_warning_count = 0

      # ファイル出力が必要な場合
      if ThroughputDefine.FLAG_EXPORT_FILE:
        # ファイル出力スレッド生成
        th_fileout = Thread(target=obj_rtd.output_to_file)
        th_fileout.start()

'''
伝達時間測定結果格納クラス
これがサンプリング結果の1ブロック分となる
'''
class RosThroughputData():
  # コンストラクタ
  def __init__(self):
    # 測定データリスト
    self.raw_data_list = []

    # ロガーハンドラ取得
    self.logger = logging.getLogger(ThroughputDefine.EXPORT_FILE_HANDLENAME)

  '''
  プロパティ群
  '''
  # サンプリングブロックデータ登録関数
  def regist_data(self, block_time, space_time):
    data = [block_time, space_time]
    self.raw_data_list.append(data)

  # ロギングを利用したファイル出力スレッド関数
  def output_to_file(self):
    for data in self.raw_data_list:
      self.logger.info('{},{}'.format(data[0], data[1]))
