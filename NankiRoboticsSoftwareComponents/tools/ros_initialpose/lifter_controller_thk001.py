#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
import tf

robot_name = '/thk001'   # 例：'/thk001'  '/thk002'
#robot_name = '/thk002'   # 例：'/thk001'  '/thk002'
#lifter_control_kind = 1   # 1/2 = 上昇/下降
lifter_control_kind = 2   # 1/2 = 上昇/下降

# ROSノード生成
rospy.init_node('lifter_control')

pub_lifter_cmd = rospy.Publisher(robot_name+'/lifter_controller/command', JointTrajectory, queue_size=5)

msg_lifter = JointTrajectory()
msg_lifter.header.seq = 1
msg_lifter.header.stamp = rospy.Time.now()
msg_lifter.points = [JointTrajectoryPoint()]
msg_lifter.joint_names = ['ankle_joint', 'knee_joint']
if lifter_control_kind == 1:
  msg_lifter.points[0].positions = [0.0, 0.0]
else:
  msg_lifter.points[0].positions = [1.5, -1.5]
  #msg_lifter.points[0].positions = [1.0, -1.0]
msg_lifter.points[0].time_from_start = rospy.Time(8.0)

rate = rospy.Rate(10)
index = 0
while not rospy.is_shutdown():
  pub_lifter_cmd.publish(msg_lifter)
  rate.sleep()
  index = index + 1
  if index >= 10:
    break
