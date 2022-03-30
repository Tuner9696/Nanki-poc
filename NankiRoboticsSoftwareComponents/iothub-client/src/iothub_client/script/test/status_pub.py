#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
# メッセージの型等のimport

from iothub_client.msg import status_message


ROBOT_STATUS_ADPATOL = 1  
ROBOT_STATUS_GUIDE   = 2  
ROBOT_STATUS_IDLE    = 99  


class Statuspub():
    def __init__(self):
        # nodeの立ち上げ
        rospy.init_node('Statuspub')
        # Publisherを作成
        self.r1_publisher = rospy.Publisher('thk001/status', status_message, queue_size=10)
        self.r2_publisher = rospy.Publisher('thk002/status', status_message, queue_size=10)

        # messageの型を作成
        self.message = status_message()

    def msg_convert(self,random):
        # 処理を書く
        print(random)
        if random == 1 :
            self.message.robot_status = ROBOT_STATUS_ADPATOL
        elif random == 2 :
            self.message.robot_status = ROBOT_STATUS_GUIDE
        else :
            self.message.robot_status = ROBOT_STATUS_IDLE

        
    def send_msg(self):
        
    
        # messageを送信
        self.msg_convert(random= random.randint(1,3))
      
        self.r1_publisher.publish(self.message)
        rospy.loginfo("thk001: %s", self.message)

  
        self.msg_convert(random=  random.randint(1,3))
        self.r2_publisher.publish(self.message)
        rospy.loginfo("thk002: %s", self.message)

def main():

    # クラスの作成
    pub = Statuspub()

    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        pub.send_msg()
        rate.sleep()


if __name__ == '__main__':
   main()