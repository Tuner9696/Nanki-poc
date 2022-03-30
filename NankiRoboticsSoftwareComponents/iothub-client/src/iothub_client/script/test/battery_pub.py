#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

import random
# メッセージの型等のimport

class Voltagepub():
    def __init__(self):
        # nodeの立ち上げ
        rospy.init_node('Volatagepub')
        # Publisherを作成
        self.r1_publisher = rospy.Publisher('thk001_seed_r7_ros_controller/voltage', Float32, queue_size=10)
        self.r2_publisher = rospy.Publisher('thk002_seed_r7_ros_controller/voltage', Float32, queue_size=10)

        # messageの型を作成
        self.message = Float32()

    def msg_convert1(self):
        # 処理を書く
        self.message.data = random.randrange(10, 50)  # Current temperature in Celsius

    def msg_convert2(self):
        # 処理を書く
        self.message.data = 3  # Current temperature in Celsius

    def send_msg(self):
        
        self.msg_convert1()
        self.r1_publisher.publish(self.message)
        rospy.loginfo("thk001: %s", self.message)

        
        self.msg_convert2()
        self.r2_publisher.publish(self.message)
        rospy.loginfo("thk002: %s", self.message)
   
def main():

    # クラスの作成
    pub = Voltagepub()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        pub.send_msg()
        rate.sleep()


if __name__ == '__main__':
   main()