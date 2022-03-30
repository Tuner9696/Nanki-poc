#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import obstacle_define
from threading import Thread
from obstacle_detector_component.msg import ObstacleInformation
from obstacle_detector_component.msg import Obstacle
from obstacle_define import  ObstacleDefine

# 対象デバイス名リスト（＝名前空間）
device_name_lists = ['thk001', 'thk002']

#障害物検知コンポーネント
#機能
"""
scan情報から一定の距離に物体を検知した場合、以下を送信する
・検知した物体の角度
・ロボットがいる現在位置からの回転値
・物体までの距離
"""
#正面から読み込む角度幅
angle = 80

#停止処理を行う物体との距離値
obstacle_range = 1.0

class scan_read():
  def __init__(self, name):
    #scanメッセージのコピー用
    self.robot_name = name
    self.scan_msg = LaserScan()
    self.ns_prefix = '' if name == '' else '/'
    print(name)
    #tf情報取得用
    self.tf_listen = tf.TransformListener()
    self.obstacle_Pub = rospy.Publisher(self.ns_prefix + self.robot_name + "/obstacles",Obstacle, queue_size=10)
    #self.obstacle_Pub = rospy.Publisher("/obstacles",Obstacle, queue_size=10)

  #callback関数
  def scan_callback(self, msg):
    self.angle_min = msg.angle_min #スキャンの開始角度[rad]
    self.angle_max = msg.angle_max #スキャンの終了角度[rad]
    self.angle_increment = msg.angle_increment #スキャンの測定間の角距離[rad]
    self.ranges = msg.ranges #LiDAR情報　緊急停止などに使用できる
    self.range_min = msg.range_min #最小範囲値[m]
    self.range_max = msg.range_max #最長範囲値[m]
    #scanメッセージのコピー
    self.scan_msg = msg
 
    if len(self.scan_msg.ranges) > 0:
      #self.cmd_vel = Twist()
      # i : 前方直線のrange
      front_range = -(float(self.scan_msg.angle_min)) / float(self.scan_msg.angle_increment)
      front_range = int(front_range)
      #rospy.loginfo("i = %d \n",i)

      #angleで設定した角度間隔で物体との距離を確認
      #self.obstacle_range = self.scan_range(i)
      self.scan_range(front_range)

  #tf情報を取得する
  def tf_listener(self):
    (self.trans, self.rot) = self.tf_listen.lookupTransform(ObstacleDefine.OBSTACLE_NS_FOR_MAP + '/map', self.ns_prefix + self.robot_name + '/base_link', rospy.Time(0))
  
  #角度をクォータニオンに変換
  def change_quaternion(self, degree):
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(degree))
    return q[0], q[1], q[2], q[3]

  #クォータニオンからオイラー角に変換
  #変換後、度数法を戻す
  def change_Euler(self, qx, qy, qz, qw):
    e = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
    yaw = e[2]
    return yaw
  
   #指定した角度まで移動
  #戻り値は角度(rad)
  def set_move_radians(self, select_radian):
    self.tf_listener()
    #ロボットの向いている角度を求める
    set_yaw = self.change_Euler(self.rot[0], self.rot[1], self.rot[2], self.rot[3])
    #rospy.loginfo("現在の角度 : %d", set_yaw)
    #右回転
    #＋範囲
    if 0 < set_yaw and 0 <= set_yaw - select_radian :
      move_radian = set_yaw - select_radian
    #-範囲
    elif 0 > set_yaw and math.radians(-180) <= set_yaw - select_radian:
      move_radian = set_yaw - select_radian
    #＋から-範囲
    elif 0 <= set_yaw and set_yaw - select_radian < 0 :
      move_radian = set_yaw - select_radian
    #-から＋
    elif set_yaw < 0 and math.radians(-180) > (set_yaw - select_radian):
      move_radian = math.radians(180) + set_yaw
      move_radian += select_radian
    #左回転
    #＋範囲
    elif 0 <= set_yaw and set_yaw <= math.radians(180) - select_radian:
      move_radian = set_yaw + select_radian
    #-範囲
    elif math.radians(-180) < set_yaw and  select_radian + set_yaw < 0:
      move_radian = set_yaw + select_radian
    #＋から-範囲
    elif math.radians(180) < set_yaw + select_radian and 0 <= set_yaw  :
      move_radian = math.radians(180) - set_yaw 
      move_radian = (select_radian - move_radian) - math.radians(180)
    #-から＋
    elif 0 < set_yaw + select_radian and 0> set_yaw:
      move_radian = set_yaw + select_radian
    return move_radian
  
  #前方直線と左右のrange値算出
  #戻り値 : 物体検出角度(/scan) 正対させる回転位置 物体との距離
  #角度範囲は90～-90°(0～180°)
  #range総数は721 1°あたりのrange数は約4本
  def scan_range(self, f_range):
    #格納用
    self.obstacle_list = []

    #障害物数
    obstacle_num = 0

     #一番近い物体
    fast_angle_tf = 100
    fast_obstacle_distance =100


    range_front_size = 5
    #i-5～i+5の範囲(range11本分で確認)
    start_range = int(f_range - (angle*2))
    obstacle_msg = Obstacle()
    
    while start_range < int(f_range + (angle*2)):
      self.obstacle_list = []
      obstacle_info_msg = ObstacleInformation()

      #加算用
      range_sum = 0
      #カウント用
      range_count = 0

      #range10本で距離確認
      #1本のみでは異常値になることがあり、障害物に衝突してしまう可能性があるため。
      #rangeの値が 0又はinfとなっている場合は計算処理に使用しない。
      for num in range(range_front_size*2 + 1):
        if self.scan_msg.ranges[start_range + num] <= 0:
          rospy.loginfo("range[{}]は0以下です。".format(start_range + num))
        elif self.scan_msg.ranges[start_range + num] == 'inf':
          rospy.loginfo("range[{}]は測定出ません。".format(start_range + num))
        elif self.scan_msg.ranges[start_range + num] > 0:
          range_count+=1
          range_sum+= self.scan_msg.ranges[start_range + num]
          #rospy.loginfo("range_num : {} range : {}".format(start_range + num, self.scan_msg.ranges[start_range + num]))
      
      #値が0以外のrangeの平均を算出
      distance = range_sum / range_count
      #rospy.loginfo("distance [{}] : {}".format(start_range - range_front_size, distance))

      #確認する最初のrange要素番号を更新
      start_range += range_front_size*2

      #range要素番号を元に角度算出
      #4本で1°とし、計算
      #11本のlaserの中心線を基準に指定いるため、約3°ずつ上昇する
      if start_range - range_front_size < 360:
        angle_scan =int(90 - ((360 - (start_range - range_front_size))/4))
      elif start_range - range_front_size > 360:
        angle_scan = (start_range - range_front_size) - 360
        angle_scan = int(90 + (angle_scan / 4))
      
      angle_scan = math.radians(angle_scan)
      #仮
      #物体が付近に迫ってきたら必要な値を格納
      if distance < obstacle_range:
        #配列に以下を格納
        #scanから得た角度[rad]、移動回転位置(/tf)[rad]、物体との距離[m]
        angle_tf = self.set_move_radians(angle_scan)
        #self.obstacle_list = insert(result_degree, angle_tf, distance)
        #rospy.loginfo("scan : {} tf : {} range : {}".format(angle_scan, angle_tf, distance))
        self.obstacle_list.append([angle_scan, angle_tf, distance])

        #一番近い物体
        if fast_obstacle_distance > distance:
          fast_obstacle_distance = distance
          fast_angle_tf = angle_tf

        #障害物数をカウント
        obstacle_num += 1  
        #rospy.loginfo("障害物数 {}".format(obstacle_num))
        obstacle_info_msg.angle_scan = angle_scan
        obstacle_info_msg.angle_tf = angle_tf
        obstacle_info_msg.distance = distance
        obstacle_msg.obstacle_info.append(obstacle_info_msg)
        """
        self.obstacle_msg.obstacle_info[obstacle_num -1].angle_scan = angle_scan
        self.obstacle_msg.obstacle_info[obstacle_num -1].angle_tf = angle_tf
        self.obstacle_msg.obstacle_info[obstacle_num -1].distance = distance
        """
    #障害物が0の場合と、1つ以上見つかった場合でメッセージ内容を変更
    if obstacle_num > 0:
      obstacle_msg.obstacle_flg = 1
      obstacle_msg.obstacle_angle_tf = fast_angle_tf
      obstacle_msg.obstacle_distance = fast_obstacle_distance
      obstacle_msg.obstacle_num = obstacle_num
      #Publish
      self.obstacle_Pub.publish(obstacle_msg)
      time.sleep(0.1)
      #rospy.loginfo("obstacle_msg : {}".format(obstacle_msg))
      rospy.loginfo("障害物検知 Publish finish")
    elif obstacle_num == 0:
      obstacle_msg.obstacle_flg = 0
      self.obstacle_Pub.publish(obstacle_msg)
      time.sleep(0.1)

    #Obstacleに適用させる

  #障害物検知
  #一定の距離に物体を検出した場合、リストに格納し、出力する
  def obstacle_detector(self):
    #scan Subscriber(購読者)の生成
    self.scan_Sub = rospy.Subscriber(self.ns_prefix + self.robot_name + '/scan', LaserScan, self.scan_callback)

    while not rospy.is_shutdown():
      rospy.spin()

if __name__=='__main__':
  rospy.loginfo('===障害監視制御コンポーネント開始===')
  #ノードの作成と初期化 
  rospy.init_node('obstacle_detector')

  # 対象デバイス分ループ
  for device_name in device_name_lists:
    # モーション制御メインクラスのインスタンス生成
    sr = scan_read(device_name)

    # デバイスのサブスクリプションワーカースレッド起動
    rospy.loginfo('デバイス：{} サブスクリプションワーカースレッド起動'.format(device_name))
    th_sub = Thread(target=sr.obstacle_detector)
    th_sub.start()

  while not rospy.is_shutdown():
    rospy.spin()

  rospy.loginfo('===障害監視制御コンポーネント終了===')
