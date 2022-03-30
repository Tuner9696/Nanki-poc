#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from threading import Thread
import rospy
import tf2_ros
from observer_define import ThroughputDefine
from worker.ros_throughput import RosThroughputWoker
import logging

# スループット用情報出力ロガー生成
def setup_logger4throughput(name, logfile=ThroughputDefine.EXPORT_FILE_PATH+ThroughputDefine.EXPORT_FILE_PREFIX_NAME+ThroughputDefine.EXPORT_FILE_EXTENSION):
  logger = logging.getLogger(name)
  logger.setLevel(logging.DEBUG)

  # ログファイルハンドラ生成
  fh = logging.handlers.RotatingFileHandler(logfile, maxBytes=ThroughputDefine.EXPORT_FILE_SIZE, backupCount=ThroughputDefine.EXPORT_FILE_ROTATE_COUNT)
  fh.setLevel(logging.DEBUG)
  fh_formatter = logging.Formatter('%(message)s')
  fh.setFormatter(fh_formatter)

  # add the handlers to the logger
  logger.addHandler(fh)

  return logger

# ROSスループットデータ出力用ディレクトリ作成
if not os.path.isdir(ThroughputDefine.EXPORT_FILE_PATH):
  os.makedirs(ThroughputDefine.EXPORT_FILE_PATH)
# ROSスループットデータ出力用ロギング生成
throughput_logger = setup_logger4throughput(ThroughputDefine.EXPORT_FILE_HANDLENAME)

if __name__ == '__main__':
  rospy.init_node('throughput_tester')
  rospy.loginfo('===ROSスループットテスタコンポーネント開始===')

  for links in ThroughputDefine.sampling_tf_lists:
    rospy.loginfo('ROSスループット測定ワーカースレッド起動: parent_link {}, child_link {}'.format(links[0], links[1]))
    rtw = RosThroughputWoker(links[0], links[1])
    th_sub = Thread(target=rtw.sub_worker)
    th_sub.start()

  while not rospy.is_shutdown():
    rospy.spin()

  rospy.loginfo('===ROSスループットテスタコンポーネント終了===')
