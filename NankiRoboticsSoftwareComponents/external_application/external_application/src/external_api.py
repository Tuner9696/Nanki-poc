#!/usr/bin/env python
# -*- coding: utf-8 -*-
from paho.mqtt import client as mqtt_client
import paho.mqtt.publish as publish
import paho.mqtt.subscribe as subscribe

import time
from time import sleep   
import rospy
from external_application.msg import DisplayStatus, MissionCommand, GuideDestination

#MEC側
class ros_ext():
  def __init__(self):
    self.ext_destination_msg = DisplayStatus()
    self.ext_int = 0
    #ミッションからディスプレイ状態を受け取る
    self.Sub_ext_display = rospy.Subscriber('/DDD/ext_display', DisplayStatus, self.ext_display_callback)
    #受け取った案内先番号をミッションに送信する
    self.Pub_ext_destinaiton = rospy.Publisher('/ext_destination', GuideDestination, queue_size=10)
  
  def ext_display_callback(self, msg):
    rospy.loginfo("ext_display : {}".format(msg))
    self.ext_destination_msg = msg.status
    self.ext_int = msg.status

#mqtt側
class mqttClass(mqtt_client.Client):
  def __init__(self):
    self.distination_msg = GuideDestination()
  
  #callback関数
  def destination_callback(self, client, userdata, message):
    print("Subscribe : TOPIC {} | MESSAGE {}".format(message.topic, message.payload))
    self.distination_msg.destination_no = message
    ext.Pub_ext_destinaiton.publish(self.distination_msg)
  
  def destination_run(self, hostname, topic):
    self.subscribe.callback(topic)
 
  #publish
  def Pub_update(self, hostname, topic, message):
    publish.single(topic, message, hostname = hostname)
    rospy.loginfo("finish")


def main():
  rospy.init_node('listener')
  ext = ros_ext()
  mqttc = mqttClass()
  self.connect("localhost", 1883, 60)
  print("message : {}".format(type(ext.ext_int)))
  rc_mission = mqttc.run("localhost", "/mission/update")
  rc_destination = mqttc.run("localhost", "/destination/update")
  self.loop_start()
  #MEC側から受信したディスプレイ状態を送信する
  mqttc.Pub_update("localhost", "/DDD/display/update", ext.ext_int)

 
  while True:
    time.sleep(0)

if __name__=="__main__":
  main()

