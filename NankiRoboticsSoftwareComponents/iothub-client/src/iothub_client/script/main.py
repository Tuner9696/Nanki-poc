#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###azure###
from math import degrees
from azure.iot.device import IoTHubDeviceClient
from azure.iot.device.aio import IoTHubDeviceClient
from azure.iot.device import Message
###python3
import asyncio

# ROSライブラリーインポート
import rospy
from std_msgs.msg import Float32
from iothub_client.msg import remote_mode
##config
import configparser
config = configparser.ConfigParser()
config.read('config.ini')
#json
import json
#remote_control
from remote_hub import RemoteHub
#mission triger
from mission_trigger import MisiionTrigger
#device_manage
from device_management import DeviceManagement
#guide_trigger
from guide_destination import GuideTrigger
#system_define
from system_define import SystemDefine
#log
import logging
# ロギングハンドラ取得
#logger = logging.getLogger('main')
logging.basicConfig(level=logging.DEBUG)
#os
import os
#試験用
import random
#データQ
try:
    import queue
except ImportError:
    import Queue as queue

#CONNECTION_STRING = os.environ["CONNECTION_STRING"]

#本番用メッセージの受信部
async def cloud_message_subscriber(key_meesage):

  #global RECEIVED_MESSAGES
  
  SystemDefine.RECEIVED_MESSAGES += 1

  for property in (vars(key_meesage).items()):
    #クラウドからのメッセージログ出力
    print ("    {}".format(property))
    
    #メッセージのcustom_propertiesが含まれている部分の抜き出し
    if  property[0]=="custom_properties":
      cloud_message = property[1]
      print("S")
      print(cloud_message)
      #for key_meesage in property[1]:
      for key_meesage in cloud_message:
        print(key_meesage)
        #if key_meesage == 'mission_trigger':
        #  print("1")
        #elif key_meesage == 'remote_trigger':
        #  print("2")

      #遠隔制御トリガーのFALSEAAsss sssa
      if q_remote.empty() :
        while not q_x.empty() and q_y.empty() and q_degree.empty():
          q_x.get()
          q_y.get()
          q_degree.get()

      #ミッショントリガーのクラウドメッセージを受信した場合
      if key_meesage == SystemDefine.MISSION_TRIGGER_MESSAGE:
      #クラウドメッセージの抜き出し
        #mission_trigger =  cloud_message['mission_trigger']
        mission_trigger =  cloud_message[SystemDefine.MISSION_TRIGGER_MESSAGE]
        #ミッショントリガーのFALSEの場合キューを空にする
        if mission_trigger == SystemDefine.MISSION_CONTROL_OFF_MESSAGE :
          while not q_mission.empty():
            q_mission.get()
      #ミッショントリガーのTRUEの場合キューにデータ挿入
        elif mission_trigger == SystemDefine.MISSION_CONTROL_ON_MESSAGE :
          result_mission = SystemDefine.MISSION_CONTROL_ON
          q_mission.put(result_mission)

      #案内制御のクラウドメッセージを受信した場合
      elif key_meesage == SystemDefine.GUIDE_TRIGGER_MESSAGE:
        
        while not q_guide.empty():
          q_guide.get()
        q_guide.put(cloud_message[SystemDefine.GUIDE_TRIGGER_MESSAGE])

      #遠隔制御トリガーのクラウドメッセージを受信した場合
      elif key_meesage ==SystemDefine.REMOTE_TRIGGER_MESSAGE:
        #クラウドメッセージの抜き出し
        remote_trigger = cloud_message[SystemDefine.REMOTE_TRIGGER_MESSAGE]
        #遠隔制御トリガーがFALSEの場合キューを空にする
        if remote_trigger == SystemDefine.REMOTE_CONTROL_OFF_MESSAGE :
          while not q_remote.empty():
            q_remote.get()
        #遠隔制御トリガーがTRUEの場合キューにデータ挿入
        elif remote_trigger == SystemDefine.REMOTE_CONTROL_ON_MESSAGE :
          result_remote = SystemDefine.REMOTE_CONTROL_ON
          q_remote.put(result_remote)

      #遠隔制御トリガーがTRUEかつ遠隔移動座標のメッセージを受信した場合
      elif not q_remote.empty() and cloud_message['x'] and cloud_message['y'] and cloud_message['degree'] :
          #キューに移動座標データを挿入
          q_x.put(cloud_message['x'])
          q_y.put(cloud_message['y'])
          q_degree.put(cloud_message['degree'])
      
  #ログ出力
      print("Total calls received: {}".format(SystemDefine.RECEIVED_MESSAGES))
  
  #ROSのPUBLISHER関数
async def ros_publish():
  
  #ミッション制御のキューが残っている場合は取り出す
  if not q_mission.empty():
    if q_mission.get() == SystemDefine.MISSION_CONTROL_ON :
      q_mission.put(SystemDefine.MISSION_CONTROL_ON)
      mission_trigger = SystemDefine.MISSION_CONTROL_ON
    
    elif q_mission.get() == SystemDefine.MISSION_CONTROL_OFF :
      mission_trigger = SystemDefine.MISSION_CONTROL_OFF
  else :
    mission_trigger = SystemDefine.MISSION_CONTROL_OFF


  if not q_guide.empty():
    guide_point = q_guide.get()
    print(guide_point)
    guidetrigger.send_msg(guide_point)


  #遠隔制御のキューが残っている場合は取り出す
  if not q_remote.empty():
    #遠隔制御のキューがTrueの場合
    if q_remote.get() == SystemDefine.REMOTE_CONTROL_ON :
      q_remote.put(SystemDefine.REMOTE_CONTROL_ON) 
      remote_trigger = SystemDefine.REMOTE_CONTROL_ON
   #移動制御のキューが残っている場合は取り出す
      if not (q_x.empty() and q_y.empty() and q_degree.empty() ):
        x = q_x.get()
        y = q_y.get()
        degree = q_degree.get()

    #移動制御のPublishする
        remotehub.remote_move(x,y,degree)
    #遠隔制御のキューがFalseの場合
    elif q_remote.get() == SystemDefine.REMOTE_CONTROL_OFF :
      remote_trigger =SystemDefine.REMOTE_CONTROL_OFF
    #移動制御のキューを空にする
      while not q_x.empty() and q_y.empty() and q_degree.empty():
        q_x.get()
        q_y.get()
        q_degree.get()
  else :
    remote_trigger = SystemDefine.REMOTE_CONTROL_OFF
    while not q_x.empty() and q_y.empty() and q_degree.empty():
        q_x.get()
        q_y.get()
        q_degree.get()

  #ROSのメッセージをPUBLISHする
  missiontrigger.send_msg(mission_trigger)
  remotehub.remote_mode(remote_trigger)


#本番用メッセージの送信部
async def make_message(pub):

  #以下iothub送信用のメッセージ作成
  Robot1 ={}
  Robot2 ={}

  #各ロボット位置座標をsubscribeする
  r1_x, r1_y, r1_degree =await devicemanagement.r1_tf_listener()
  r2_x, r2_y, r2_degree =await devicemanagement.r2_tf_listener()

  #コンフィグの読み取りとデバイスデータをsubscribeし、メッセージに格納
  Robot1.update(Robot_ID=config['ROBOT001']['Robot_ID'])
  Robot1.update(Name=config['ROBOT001']['Name'])
  Robot1.update(Vendor=config['ROBOT001']['Vendor'])
  Robot1.update(Product_Number=config['ROBOT001']['Product_Number'])
  Robot1.update(Product_Type=config['ROBOT001']['Product_Type'])
  Robot1.update(Position_x= r1_x)
  Robot1.update(Position_y= r1_y)
  Robot1.update(Position_z= config['ROBOT001']['position_z'])
  Robot1.update(Position_Degree= r1_degree)
  Robot1.update(Status= await devicemanagement.r1_status())
  Robot1.update(Battery= await devicemanagement.r1_voltage())
  Robot1.update(Obstacle_flg=await devicemanagement.r1_obstacle())
  Robot1.update(Floor=config['ROBOT001']['Floor'])
  Robot1.update(Eroor=config['ROBOT001']['Eroor'])
  robo1_msg = Message(json.dumps(Robot1))
  robo1_msg.content_encoding = "utf-8"
  robo1_msg.content_type = "application/json"

  Robot2.update(Robot_ID=config['ROBOT002']['Robot_ID'])
  Robot2.update(Name=config['ROBOT002']['Name'])
  Robot2.update(Vendor=config['ROBOT002']['Vendor'])
  Robot2.update(Product_Number=config['ROBOT002']['Product_Number'])
  Robot2.update(Product_Type=config['ROBOT002']['Product_Type'])
  Robot2.update(Position_x= r2_x)
  Robot2.update(Position_y= r2_y)
  Robot2.update(Position_z= config['ROBOT002']['position_z'])
  Robot2.update(Position_Degree= r2_degree)
  Robot2.update(Status= await devicemanagement.r2_status())
  Robot2.update(Battery= await devicemanagement.r2_voltage())
  Robot2.update(Obstacle_flg= await devicemanagement.r2_obstacle())
  Robot2.update(Floor=config['ROBOT002']['Floor'])
  Robot2.update(Eroor=config['ROBOT002']['Eroor'])
  robo2_msg = Message(json.dumps(Robot2))
  robo2_msg.content_encoding = "utf-8"
  robo2_msg.content_type = "application/json"

  #メッセージのログ

  rospy.loginfo(robo1_msg)
  rospy.loginfo(robo2_msg)
  #iothubのメッセージの送信
  await pub.send_message(robo1_msg)
  await pub.send_message(robo2_msg)


if __name__ == '__main__':

  try:
    #ROSノードの立ち上げ
    rospy.init_node('devicemanagement')
    
    #インスタンス生成
    devicemanagement = DeviceManagement()
    remotehub = RemoteHub()
    missiontrigger = MisiionTrigger()
    guidetrigger = GuideTrigger()

    #Q生成
    q_remote = queue.Queue()
    q_mission = queue.Queue()
    q_x = queue.Queue()
    q_y = queue.Queue()
    q_degree = queue.Queue()
    q_guide = queue.Queue()

    # Instantiate the client
    client = IoTHubDeviceClient.create_from_connection_string(SystemDefine.CONNECTION_STRING)


    #Send key_meesage loop 
    loop = asyncio.get_event_loop()
    loop2 = asyncio.get_event_loop()

   #入力で１が押されたときは本番用プログラムを実行
   #入力で２が押されたときは本番用プログラムを実行
   #それ以外は終了
    client.on_message_received = cloud_message_subscriber
    print("POC本番用プログラム")
        
    while True:
   #本番用プログラムを実行
      loop.run_until_complete(make_message(client))
      loop2.run_until_complete(ros_publish())
              
  except KeyboardInterrupt:
        print("IoT Hub C2D Messaging device stopped")

  finally:
        print("Shutting down IoT Hub Client")
        #メッセージループのシャットダウン
        loop.run_until_complete(client.shutdown())
        #loop2.run_until_complete(client.shutdown())

        #メッセージループのクローズ
        loop.close()
        loop2.close()
        
        client.shutdown()

