#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
# メッセージの型等のimport

from iothub_client.msg import obstacles


OBSTACLE_FLG_ON = 1  
OBSTACLE_FLG_OFF = 0  


class Obstaclepub():
    def __init__(self):
        # nodeの立ち上げ
        rospy.init_node('Obstaclepub')
        # Publisherを作成
        self.r1_publisher = rospy.Publisher('thk001/obstacles', obstacles, queue_size=10)
        self.r2_publisher = rospy.Publisher('thk002/obstacles', obstacles, queue_size=10)

        # messageの型を作成
        self.message = obstacles()

    def msg_convert(self):
        # 処理を書く
        self.message.obstacle_flg = OBSTACLE_FLG_OFF
       

    def send_msg(self):
        
    
        # messageを送信
        self.msg_convert()
      
        self.r1_publisher.publish(self.message)
        rospy.loginfo("thk001: %s", self.message)


  
        self.msg_convert()
        self.r2_publisher.publish(self.message)
        rospy.loginfo("thk002: %s", self.message)

def main():

    # クラスの作成
    pub = Obstaclepub()

    while not rospy.is_shutdown():
        pub.send_msg()
        rate = rospy.Rate(1)
        rate.sleep()


if __name__ == '__main__':
   main()