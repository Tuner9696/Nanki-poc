#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
# ROSライブラリーインポート
import tf
import tf2_ros
# ROSメッセージインポート
from iothub_client.msg import status_message
from iothub_client.msg import obstacles
from std_msgs.msg import Float32
# pythonライブラリーインポート
try:
    import queue
except ImportError:
    import Queue as queue

#system_define
from system_define import SystemDefine

# デバイス監視モジュール
class DeviceManagement():
  def __init__(self):
    # デバイス監視モジュール
    # バッテリー情報のサブスクライバー
    self.subsciber1 = rospy.Subscriber("thk001_seed_r7_ros_controller/voltage", Float32, self.r1_voltage_callback)
    self.q_r1_voltage = queue.Queue()
    
    self.subsciber2 = rospy.Subscriber("thk002_seed_r7_ros_controller/voltage", Float32, self.r2_voltage_callback)
    self.q_r2_voltage = queue.Queue()
    
    # ステータス情報のサブスクライバー
    self.subsciber3 = rospy.Subscriber("thk001/status", status_message, self.r1_status_callback)
    self.q_r1_status_sub = queue.Queue()
    self.q_r1_status_pub = queue.Queue()
    self.q_r1_status_message = queue.Queue()
    
    self.subsciber4 = rospy.Subscriber("thk002/status", status_message, self.r2_status_callback)
    self.q_r2_status = queue.Queue()
    self.q_r2_status_pub = queue.Queue()
    self.q_r2_status_message = queue.Queue()
    
    # 障害物情報のサブスクライバー
    self.subsciber5 = rospy.Subscriber("thk001/obstacles", obstacles, self.r1_obstacle_callback)
    self.q_r1_obstacle = queue.Queue()
    
    self.subsciber6 = rospy.Subscriber("thk002/obstacles", obstacles, self.r2_obstacle_callback)
    self.q_r2_obstacle = queue.Queue()

    # tf2_rosリスナー情報の登録
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    # tfのリスナーインスタンス生成
    self.tf_listener = tf.TransformListener()

    self.q_r1_tf_x = queue.Queue()
    self.q_r1_tf_y = queue.Queue()
    self.q_r1_tf_degree = queue.Queue()
   
    self.q_r2_tf_x = queue.Queue()
    self.q_r2_tf_y = queue.Queue()
    self.q_r2_tf_degree = queue.Queue()


    # 以下、各callback関数
  def r1_voltage_callback(self,float32):
    #rospy.loginfo("thk001_battery:  %s",float32.data)
    self.q_r1_voltage.put(float32.data)

  async def r1_voltage(self):
    return self.q_r1_voltage.get()

  def r2_voltage_callback(self,float32):
    #rospy.loginfo("thk002_battery:  %s",float32.data)
    self.q_r2_voltage.put(float32.data)

  async def r2_voltage(self):
    return self.q_r2_voltage.get()


  def r1_status_callback(self,status_message):
    rospy.loginfo("thk001_status_message:  %s",status_message.robot_status)
    #ミッション制御からステータス状態をサブスクライブしデータQに格納
    if (status_message.robot_status == SystemDefine.ROBOT_STATUS_ADPATOL):
    #新しい状態データを格納したときはデータQを空にする
      while not self.q_r1_status_pub.empty():
        self.q_r1_status_pub.get()
    #状態データ更新しデータQに格納
      status = SystemDefine.STATUS_MESSAGE_ADPATOL
      self.q_r1_status_pub.put(status)

    #ミッション制御からステータス状態をサブスクライブしデータQに格納
    elif (status_message.robot_status == SystemDefine.ROBOT_STATUS_GUIDE):
      #新しい状態データを格納したときはデータQを空にする
      while not self.q_r1_status_pub.empty():
        self.q_r1_status_pub.get()
      #状態データ更新しデータQに格納
      status = SystemDefine.STATUS_MESSAGE_GUIDE
      self.q_r1_status_pub.put(status)

    elif (status_message.robot_status == SystemDefine.ROBOT_STATUS_IDLE):
      #新しい状態データを格納したときはデータQを空にする
      while not self.q_r1_status_pub.empty():
        self.q_r1_status_pub.get()
      #状態データ更新しデータQに格納
      status = SystemDefine.STATUS_MESSAGE_IDLE
      self.q_r1_status_pub.put(status)
    
  
  async def r1_status(self):
    #状態データQが空のときはアイドリング状態とする
    if self.q_r1_status_pub.empty():
      self.q_r1_status_message.put(SystemDefine.STATUS_MESSAGE_IDLE)
    else:
      status = self.q_r1_status_pub.get() 

      #状態データQによって分類
      if status == SystemDefine.STATUS_MESSAGE_IDLE:
        self.q_r1_status_message.put(SystemDefine.STATUS_MESSAGE_IDLE)
        self.q_r1_status_pub.put(SystemDefine.STATUS_MESSAGE_IDLE)

      elif status == SystemDefine.STATUS_MESSAGE_GUIDE:
        self.q_r1_status_message.put(SystemDefine.STATUS_MESSAGE_GUIDE)
        self.q_r1_status_pub.put(SystemDefine.STATUS_MESSAGE_GUIDE)
      
      elif status == SystemDefine.STATUS_MESSAGE_ADPATOL:
        self.q_r1_status_message.put(SystemDefine.STATUS_MESSAGE_ADPATOL)
        self.q_r1_status_pub.put(SystemDefine.STATUS_MESSAGE_ADPATOL)
    
    #状態データを返す
    return self.q_r1_status_message.get()

  
  def r2_status_callback(self,status_message):
    rospy.loginfo("thk002_status_message:  %s",status_message.robot_status)
    #ミッション制御からステータス状態をサブスクライブしデータQに格納
    if (status_message.robot_status == SystemDefine.ROBOT_STATUS_ADPATOL):
      #新しい状態データを格納したときはデータQを空にする
      while not self.q_r2_status_pub.empty():
        self.q_r2_status_pub.get()
      #状態データ更新しデータQに格納
      status = SystemDefine.STATUS_MESSAGE_ADPATOL
      self.q_r2_status_pub.put(status)

    elif (status_message.robot_status == SystemDefine.ROBOT_STATUS_GUIDE):
      #新しい状態データを格納したときはデータQを空にする
      while not self.q_r2_status_pub.empty():
        self.q_r2_status_pub.get()
      #状態データ更新しデータQに格納
      status = SystemDefine.STATUS_MESSAGE_GUIDE
      self.q_r2_status_pub.put(status)

    elif (status_message.robot_status == SystemDefine.ROBOT_STATUS_IDLE):
      #新しい状態データを格納したときはデータQを空にする
      while not self.q_r2_status_pub.empty():
        self.q_r2_status_pub.get()
      #状態データ更新しデータQに格納
      status = SystemDefine.STATUS_MESSAGE_IDLE
      self.q_r2_status_pub.put(status)

  async def r2_status(self):
    #状態データQが空のときはアイドリング状態とする
    if self.q_r2_status_pub.empty():
      self.q_r2_status_message.put(SystemDefine.STATUS_MESSAGE_IDLE)
    else:
      status = self.q_r2_status_pub.get() 

      #状態データQによって分類
      if status == SystemDefine.STATUS_MESSAGE_IDLE:
        self.q_r2_status_message.put(SystemDefine.STATUS_MESSAGE_IDLE)
        self.q_r2_status_pub.put(SystemDefine.STATUS_MESSAGE_IDLE)

      elif status == SystemDefine.STATUS_MESSAGE_GUIDE:
        self.q_r2_status_message.put(SystemDefine.STATUS_MESSAGE_GUIDE)
        self.q_r2_status_pub.put(SystemDefine.STATUS_MESSAGE_GUIDE)
      
      elif status == SystemDefine.STATUS_MESSAGE_ADPATOL:
        self.q_r2_status_message.put(SystemDefine.STATUS_MESSAGE_ADPATOL)
        self.q_r2_status_pub.put(SystemDefine.STATUS_MESSAGE_ADPATOL)
    #状態データを返す
    return self.q_r2_status_message.get()
  
  #障害物検知コールバック関数
  def r1_obstacle_callback(self,obstacles):
    rospy.loginfo("thk001_obstacle_flg:  %s",obstacles.obstacle_flg)
    return self.q_r1_obstacle.put(obstacles.obstacle_flg)

  async def r1_obstacle(self):
    return self.q_r1_obstacle.get()

  #障害物検知コールバック関数
  def r2_obstacle_callback(self,obstacles):
    rospy.loginfo("thk002_obstacle_flg:  %s",obstacles.obstacle_flg)
    return self.q_r2_obstacle.put(obstacles.obstacle_flg)
  async def r2_obstacle(self):
    return self.q_r2_obstacle.get()

  #TFコールバック関数
  async def r1_tf_listener(self):
    rate = rospy.Rate(10)
    self.tf_listener.waitForTransform(SystemDefine.NAMESPACE_FOR_MAP+'/map', '/thk001/base_link', rospy.Time(0), rospy.Duration(4.0))
    for count in range(5):
      try:
        self.tf_listener.waitForTransform(SystemDefine.NAMESPACE_FOR_MAP+'/map', '/thk001/base_link', rospy.Time(0), rospy.Duration(3.0))
        ([pos_x, pos_y, pos_z], [rot_x, rot_y, rot_z, rot_w]) = self.tf_listener.lookupTransform(SystemDefine.NAMESPACE_FOR_MAP+'/map', '/thk001/base_link', rospy.Time(0))
      except:
        rate.sleep()
        continue

      break

    if count >= 4:
      rospy.logerr('THK001:tf取得失敗')
      return 0.0, 0.0, 0.0

    rospy.loginfo("thk001: x,y,degree %s,%s,%s", pos_x, pos_y, rot_w)
    self.q_r1_tf_x.put(pos_x)
    self.q_r1_tf_y.put(pos_y)
    self.q_r1_tf_degree.put(rot_w)

    return self.q_r1_tf_x.get(),self.q_r1_tf_y.get(),self.q_r1_tf_degree.get()

    # r1 = self.tfBuffer.lookup_transform(SystemDefine.NAMESPACE_FOR_MAP+'map', 'thk001/odom', rospy.Time(0))
    # rospy.loginfo("thk001: x,y,degree %s,%s,%s",r1.transform.translation.x,r1.transform.translation.y,r1.transform.rotation.w )
    # self.q_r1_tf_x.put(r1.transform.translation.x)
    # self.q_r1_tf_y.put(r1.transform.translation.y)
    # self.q_r1_tf_degree.put(r1.transform.rotation.w)
    # return self.q_r1_tf_x.get(),self.q_r1_tf_y.get(),self.q_r1_tf_degree.get()

  #TFコールバック関数
  async def r2_tf_listener(self):
    rate = rospy.Rate(10)
    self.tf_listener.waitForTransform(SystemDefine.NAMESPACE_FOR_MAP+'/map', '/thk002/base_link', rospy.Time(0), rospy.Duration(4.0))
    for count in range(5):
      try:
        self.tf_listener.waitForTransform(SystemDefine.NAMESPACE_FOR_MAP+'/map', '/thk002/base_link', rospy.Time(0), rospy.Duration(3.0))
        ([pos_x, pos_y, pos_z], [rot_x, rot_y, rot_z, rot_w]) = self.tf_listener.lookupTransform(SystemDefine.NAMESPACE_FOR_MAP+'/map', '/thk002/base_link', rospy.Time(0))
      except:
        rate.sleep()
        continue

      break

    if count >= 4:
      rospy.logerr('THK002:tf取得失敗')
      return 0.0, 0.0, 0.0

    rospy.loginfo("thk002: x,y,degree %s,%s,%s", pos_x, pos_y, rot_w)
    self.q_r1_tf_x.put(pos_x)
    self.q_r1_tf_y.put(pos_y)
    self.q_r1_tf_degree.put(rot_w)
    return self.q_r1_tf_x.get(),self.q_r1_tf_y.get(),self.q_r1_tf_degree.get()

    # r2 = self.tfBuffer.lookup_transform(SystemDefine.NAMESPACE_FOR_MAP+'map', 'thk002/odom', rospy.Time(0))
    # rospy.loginfo("thk002: x,y,degree %s,%s,%s",r2.transform.translation.x,r2.transform.translation.y,r2.transform.rotation.w)
    # self.q_r2_tf_x.put(r2.transform.translation.x)
    # self.q_r2_tf_y.put(r2.transform.translation.y)
    # self.q_r2_tf_degree.put(r2.transform.rotation.w)
    # return self.q_r2_tf_x.get(),self.q_r2_tf_y.get(),self.q_r2_tf_degree.get()
