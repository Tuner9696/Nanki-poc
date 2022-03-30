#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import IntEnum

class ThroughputDefine():
  # サンプリングとしてのTFにおける基準LINK、子LINK
  sampling_tf_lists = [['odom', 'base_link']]

  # サンプルとして情報取得する間隔(ミリ秒)
  SAMPLE_DURATION = 1000
  # サンプリングとして考える1ブロック内のサンプル数
  SAMPLING_DATA_COUNT = 5

  # サンプル取得のタイムアウト時間(ミリ秒)
  SAMPLING_TIMEOUT = 1000

  # アラームログ出力する平均伝達時間(ミリ秒)
  # サンプリング1ブロックの平均伝達時間がこの値以下になるとエラーログを出力
  ROS_THROUGHPUT_ALARM = 1000

  # エラー出力する警告連続回数
  ERROR_NOTIFY_WARNINGCOUNT = 3

  # ファイル出力フラグ
  FLAG_EXPORT_FILE = True

  # ファイル出力用ロガーハンドル名
  EXPORT_FILE_HANDLENAME = 'throuphput_logging'
  # ファイル出力するパス
  EXPORT_FILE_PATH = 'throughput_data/'
  # 出力ファイル名のプレフィックス(この後にアンダーバーを挟んで日時が入る)
  EXPORT_FILE_PREFIX_NAME = 'ros_throughput'
  # 出力ファイル名の拡張子
  EXPORT_FILE_EXTENSION = '.csv'
  # ファイル出力するブロック数
  EXPORT_FILE_BLOCK_COUNT = 5
  # 1ファイルの最大サイズ(バイト)
  EXPORT_FILE_SIZE = 1000000
  # ローテーションファイル数
  EXPORT_FILE_ROTATE_COUNT = 100

