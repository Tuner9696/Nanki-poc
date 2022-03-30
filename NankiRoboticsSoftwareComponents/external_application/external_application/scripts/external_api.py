#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from external_application.msg import DisplayStatus
from external_application.msg import MissionCommand
from external_application.msg import GuideDestination
from external_application.msg import UserInterfaceMissionCommand
import paho.mqtt.client as mqtt 
import json
import time
import ast
from time import sleep   
from external_api_define import ExternalAPIDefine
import subprocess
import os

# デバイス名リストのグローバル変数
device_lists = ['thk001', 'thk002']

# MQTTクライアントオブジェクトのグローバル変数
mq_cli = None

"""
MEC内部とROSによる情報送受信を行う
ロボットごとに生成されるクラス
"""
class MECInformationTrans:
  def __init__(self, name):
    rospy.loginfo("【{}】外部インターフェース起動".format(name))
    self.ext_destination_msg = DisplayStatus()
    self.ui_mission_msg = UserInterfaceMissionCommand()
    self.robot_name  = name

    # 名前空間プレフィックス設定
    self.ns_prefix = '/' if name == '' else '/'

    # ディスプレイ表示
    self.sub_ext_display = rospy.Subscriber(self.ns_prefix + name + "/ext_display", DisplayStatus, self.ext_display_callback)
    # ミッション指示
    self.Pub_ui_mission = rospy.Publisher('/ui_mission', UserInterfaceMissionCommand, queue_size=10)
    # 案内先
    self.Pub_ext_destination = rospy.Publisher('/ext_destination', GuideDestination, queue_size=10)
  
  # ディスプレイ表示callback
  def ext_display_callback(self, msg):
    rospy.loginfo("【{}】ディスプレイ表示 受信".format(self.robot_name))
    rospy.loginfo("ext_display : {}".format(msg))
    # json形式に変更
    self.ext_destination_msg = msg.status 
    msg_dict = {"status" : msg.status}
    display_message = json.dumps(msg_dict)
    # MQTT 送信
    mq_cli.publish(self.ns_prefix + self.robot_name + "/status/update", display_message)    # トピック名とメッセージを決めて送信
    #メッセージクリア
    status = os.system('/home/ncos/bin/thk001_rst.sh')
    print("thk001_display {}".format(status))
    status = os.system('/home/ncos/bin/thk002_rst.sh')
    print("thk002_display {}".format(status))


# ミッション制御コンポーネントに対するPublisher生成
pub_ui_mission = rospy.Publisher('/ui_mission', UserInterfaceMissionCommand, queue_size=10)
pub_ext_destination = rospy.Publisher('/ext_destination', GuideDestination, queue_size=10)

# ブローカーに接続できたときの処理
def on_connect(client, userdata, flag, rc):
  rospy.loginfo("MQTT接続結果コード:{}".format(rc))  # 接続できた旨表示
   # subするトピックを設定
  client.subscribe("/mission/update") 
  client.subscribe("/destination/update")

# ブローカーが切断したときの処理
def on_disconnect(client, userdata, flag, rc):
  if  rc != 0:
    rospy.logerror("Unexpected disconnection.")

# publishが完了したときの処理
def on_publish(client, userdata, mid):
  rospy.loginfo("published: {0}".format(mid))

# メッセージが届いたときの処理
def on_message(client, userdata, msg):
  # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
  rospy.loginfo("Received message '" + str(msg.payload) + "' on topic '" + msg.topic + "' with QoS " + str(msg.qos))
  # mqttxから送信されるデータ型がbytes型のため辞書型に変換する
  rospy.loginfo(type(msg.payload))
  tmp = msg.payload
  msg_dict = ast.literal_eval(tmp.decode())

  # 受信したトピック名ごとで送信する形を変更
  if msg.topic == "/mission/update":
    # 辞書のキーとして'command'があれば処理
    if 'command' in msg_dict:
      m_msg = msg_dict["command"]
      rospy.loginfo("{} publish".format(msg.topic))
      pub_ui_mission.publish(m_msg)
      #メッセージクリア
      status = os.system('/home/ncos/bin/mission_rst.sh')
      print("mission {}".format(status))
  elif msg.topic == "/destination/update":
    # 辞書のキーとして'destination_no'があれば処理
    if 'destination_no' in msg_dict:
      m_msg = msg_dict["destination_no"]
      rospy.loginfo("{} publish".format(msg.topic))
      pub_ext_destination.publish(m_msg)
      #メッセージクリア
      status = os.system('/home/ncos/bin/destination_rst.sh')
      print("destination {}".format(status))

def mqtt_set():
  rospy.loginfo("MQTTクライアント生成")

  # クライアントインスタンス生成
  client = mqtt.Client()                

  # MQTT通信用コールバック登録
  client.on_connect = on_connect         
  client.on_disconnect = on_disconnect   
  client.on_message = on_message         
  client.on_publish = on_publish

  # MQTTブローカーへ接続
  broker_address = ExternalAPIDefine.MQTT_BROKER_ADDRESS
  broker_port = ExternalAPIDefine.MQTT_BROKER_PORT
  connect_timeout = ExternalAPIDefine.MQTT_CONNECT_TIMEOUT
  client.connect(broker_address, broker_port, connect_timeout)
  client.loop_start()
  return client

if __name__=="__main__":
  rospy.init_node('interface_UI', anonymous=True)
  mq_cli = mqtt_set()
  for device in device_lists:
    mit = MECInformationTrans(device)
  rospy.sleep(0.1)
  #mq_cli = mqtt_set()
  while not rospy.is_shutdown():
    rospy.sleep(0.1)
