#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import threading
from threading import Event
import sys, time, datetime
from enum import Enum
from mission.msg import DisplayStatus, GuideDestination, MissionCommand, RobotStaus, RemoteControlMissionCommand, UserInterfaceMissionCommand,Obstacle
from startposmove_job import StartPosMoveJob
from adpatrol_job import AdPatrolJob
from guide_job import GuideJob
from standfront_job import StandFrontJob
from handover_job import HandoverJob
from db_access_helper import DestPoint, get_point_coordinate, get_adpatrol_path, get_adpatrol_startpoint, get_point_no, get_root_pointno, get_handover_points
from handover_helper import judge_handover, select_handover_point, get_handover_gap_point
from system_define import SystemDefine

def mission_factory():

  mission = Mission()
  return mission

robot_ctx_1 = {"lock":threading.Lock(), "stop":False}
robot_ctx_2 = {"lock":threading.Lock(), "stop":False}

# 遠隔制御モード
remote_mode = False
# ミッションモード
mission_mode = False

# 案内先
guide_dest_pos = None
# 障害物検知
obstacle_flg = False

# 引き継ぎ判定
handover_flg = False
# 引き継ぎ先
handover_pos = None

# 引き継ぎ時のジョブタイミング合わせイベント
event_meetup = Event()
event_separate = Event()


class JOBType(Enum):
  UNKNOWN = 0,
  # 開始位置移動ジョブ
  JOB_STARTPOSTMOVE = 1
  # 広告巡回ジョブ
  JOB_ADPATROL = 2
  # 障害物正対ジョブ
  JOB_STANDFRONT = 3
  # 案内ジョブ（目的地は案内先）
  JOB_GUIDE = 4
  # 案内ジョブ（目的地は引き継ぎ地）
  JOB_HANDOVER = 5

class RobotJob():
  # コンストラクタ
  def __init__(self, id, jobtype, job, ctx):

    # ロボット識別子
    self.robot_id = id
    # ジョブ種別
    self.robot_jobtype = jobtype
    # ジョブ
    self.robotjob = job
    # 広告巡回カウント
    self.robot_patrol_count = 0
    # 現在の目的地が何番目か
    self.robot_current_destination = 1

    # MissionとJobで共有するデータ
    self.ctx = {"lock":threading.Lock(), "stop":False}


class Mission:

  # コンストラクタ
  def __init__(self):
    global robot_ctx_1
    global robot_ctx_2
    global remote_mode
    global mission_mode
    global guide_dest_pos
    global obstacle_flg
    global handover_flg
    global event_meetup
    global event_separate
    global handover_pos

    # ロボット単位ジョブ
    self.robot_job_map = {}
    #for i in range(2):
    #  robot_id = format(i+1, '03d')
    #  if robot_id == '001':
    #    self.robot_job_map[robot_id] = RobotJob(robot_id, JOBType.UNKNOWN, NULL, robot_ctx_1)
    #  else:
    #    self.robot_job_map[robot_id] = RobotJob(robot_id, JOBType.UNKNOWN, NULL, robot_ctx_2)

    # 遠隔制御モード
    remote_mode = False
    # ミッションモード
    mission_mode = False

    self.robotStaus_publisher_map = {}
    for i in range(2):
      robot_id = format(i+1, '03d')
      publisher = rospy.Publisher('/' + SystemDefine.ROBOT_NAMESPACE_HEAD + robot_id + '/status',
                      RobotStaus, queue_size=1, latch=True)
      self.robotStaus_publisher_map[robot_id] = publisher

    self.displayStaus_publisher_map = {}
    for i in range(2):
      robot_id = format(i+1, '03d')
      publisher = rospy.Publisher('/' + SystemDefine.ROBOT_NAMESPACE_HEAD + robot_id + '/ext_display',
                      DisplayStatus, queue_size=1, latch=True)
      self.displayStaus_publisher_map[robot_id] = publisher

    for i in range(2):
      robot_id = format(i+1, '03d')
      # ロボット状態「アイドル」送信
      self._publish_robot_status(robot_id, SystemDefine.ROBOT_STATUS_IDLE)
      # ロボット状態「無表示」送信
      rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(robot_id))
      self._publish_display_status(robot_id, SystemDefine.DISPLAY_IDLE)

    topic_name = '/ext_mission'
    rospy.Subscriber(topic_name, MissionCommand, self._callback_mission)
    topic_name = '/ext_destination'
    rospy.Subscriber(topic_name, GuideDestination, self._callback_destination)
    topic_name = '/rmt_mission'
    rospy.Subscriber(topic_name, RemoteControlMissionCommand, self._callback_rmt_mission)
    topic_name = '/ui_mission'
    rospy.Subscriber(topic_name, UserInterfaceMissionCommand, self._callback_ui_mission)
    for i in range(2):
      robot_id = format(i+1, '03d')
      topic_name = '/' + SystemDefine.ROBOT_NAMESPACE_HEAD + robot_id + '/obstacles'
      rospy.Subscriber(topic_name, Obstacle, self._callback_obstacle)
      break    # ロボット001のみ　訪問客が案内先を選択するのはロボット001のみとする
    rospy.spin()


  # ミッション指示受信(クラウド)
  def _callback_mission(self, msg):
    rospy.loginfo('**********************************')
    rospy.loginfo('ミッション指示受信')
    if msg.command == SystemDefine.CMD_START:
      rospy.loginfo('command :開始')
      rospy.loginfo('**********************************')
      # ミッション開始時どこにいるかわからないので自律走行
      self._mission_start(msg, False)
    elif msg.command == SystemDefine.CMD_STOP:
      rospy.loginfo('command :停止')
      rospy.loginfo('**********************************')
      self._mission_stop(msg)
    else:
      rospy.loginfo('command :不正値')
      rospy.loginfo('**********************************')
      pass

  # ミッション指示受信(遠隔制御)
  def _callback_rmt_mission(self, msg):
    global remote_mode
    rospy.loginfo('**********************************')
    rospy.loginfo('ミッション指示受信')
    if msg.command == SystemDefine.CMD_RESUME:
      rospy.loginfo('command :再開')
      rospy.loginfo('**********************************')
      # ミッション開始時どこにいるかわからないので自律走行
      # 遠隔制御モード
      remote_mode = False
      self._mission_start(msg, False)
    elif msg.command == SystemDefine.CMD_PAUSE:
      rospy.loginfo('command :一時停止')
      rospy.loginfo('**********************************')
      # 遠隔制御モード
      remote_mode = True
      self._mission_stop(msg)
    else:
      rospy.loginfo('command :不正値')
      pass

  # ミッション指示受信(ユーザインタフェース)
  def _callback_ui_mission(self, msg):
    rospy.loginfo('**********************************')
    rospy.loginfo('ミッション指示受信')
    if msg.command == SystemDefine.CMD_START:
      rospy.loginfo('command :開始')
      rospy.loginfo('**********************************')
      # ミッション開始時どこにいるかわからないので自律走行
      self._mission_start(msg, False)
    elif msg.command == SystemDefine.CMD_STOP:
      rospy.loginfo('command :停止')
      rospy.loginfo('**********************************')
      self._mission_stop(msg)
    else:
      rospy.loginfo('command :不正値')
      rospy.loginfo('**********************************')
      pass

  # ミッション開始処理
  def _mission_start(self, msg, flg):
    global robot_ctx_1
    global robot_ctx_2
    global mission_mode
    global guide_dest_pos
    global obstacle_flg
    global handover_flg
    global handover_pos

    if mission_mode == True:
      return

    # ミッションモード
    mission_mode = True
    # 案内先
    guide_dest_pos = None
    # 障害物検知
    obstacle_flg = False
    # 引き継ぎ判定
    handover_flg = False
    # 引き継ぎ先
    handover_pos = None
    
    threads = []
    # ロボット単位にジョブ開始
    for i in range(2):
      robot_id = format(i+1, '03d')

      # DBから巡回開始位置を取得する
      destpoints = get_adpatrol_startpoint(i+1)

      if robot_id == '001':
        robot_ctx_1["stop"] = False
        th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_1, flg)
        th_robot_job.start()
        self.robot_job_map[robot_id] = RobotJob(robot_id, JOBType.JOB_STARTPOSTMOVE, th_robot_job, robot_ctx_1)
      else:
        robot_ctx_2["stop"] = False
        th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_2, flg)
        th_robot_job.start()
        self.robot_job_map[robot_id] = RobotJob(robot_id, JOBType.JOB_STARTPOSTMOVE, th_robot_job, robot_ctx_2)

      # ロボット状態「広告巡回中」送信
      self._publish_robot_status(robot_id, SystemDefine.ROBOT_STATUS_ADPATROL)
      rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(robot_id))
      self._publish_display_status(robot_id, SystemDefine.DISPLAY_IDLE)

      rospy.loginfo('ジョブ終了待ちスレッド開始 ロボット識別子={}'.format(robot_id))
      th = threading.Thread(target=self.thread_waiting_job, args=(robot_id,))
      th.daemon = True
      th.start()
      threads.append(th)

    # ジョブ終了待ちスレッドの終了を待ち合わせる
    for t in threads:
      t.join()
    mission_mode = False


  # ミッション停止処理
  def _mission_stop(self, msg):
    global robot_ctx_1
    global robot_ctx_2
    global mission_mode

    if mission_mode == False:
      return

    # ミッションモード
    mission_mode = False

    robot_ctx_1["stop"] = True
    robot_ctx_2["stop"] = True

  # ジョブ終了待ちスレッド
  def thread_waiting_job(self, robot_id):
    global robot_ctx_1
    global robot_ctx_2
    global mission_mode
    global remote_mode
    global obstacle_flg
    global event_meetup
    global event_separate
    global handover_pos
    global handover_flg
    global guide_dest_pos

    while True:
      # ジョブ終了待ち
      robot_job = self.robot_job_map[robot_id].robotjob
      robot_job.join()

      # 停止したのは広告巡回ジョブ、または開始位置移動ジョブの場合、
      # 現在の目的地が何番目かを保存しておく
      if self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_ADPATROL \
        or self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_STARTPOSTMOVE:
        self.robot_job_map[robot_id].robot_current_destination = robot_job.get_current_destination_no()
        n = self.robot_job_map[robot_id].robot_current_destination
        rospy.loginfo('停止時の目的地は{}番目 ロボット識別子={}'.format(n,robot_id))

      # 停止したのは障害物正対ジョブの場合、
      # 障害物正対ジョブに入る前のジョブを保存しておく
      before_jobtype = JOBType.UNKNOWN
      if self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_STANDFRONT:
        before_jobtype = robot_job.before_jobtype

      del robot_job
      self.robot_job_map[robot_id].robotjob = None
      rospy.loginfo('ジョブ停止確認OK ロボット識別子={}'.format(robot_id))

      rospy.loginfo('ミッションモード={}'.format(mission_mode))
      
      if remote_mode == True:
        self.robot_job_map[robot_id].robot_jobtype = JOBType.UNKNOWN
        self.robot_job_map[robot_id].robot_patrol_count = 0
        self.robot_job_map[robot_id].robot_current_destination = 1
        rospy.loginfo('ミッション停止 ロボット識別子={}'.format(robot_id))
        # ロボット状態「アイドル」送信
        self._publish_robot_status(robot_id, SystemDefine.ROBOT_STATUS_IDLE)
        # ロボット状態「無表示」送信
        rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(robot_id))
        self._publish_display_status(robot_id, SystemDefine.DISPLAY_IDLE)
        break
      elif mission_mode == False:
        self.robot_job_map[robot_id].robot_jobtype = JOBType.UNKNOWN
        self.robot_job_map[robot_id].robot_patrol_count = 0
        self.robot_job_map[robot_id].robot_current_destination = 1
        rospy.loginfo('ミッション停止 ロボット識別子={}'.format(robot_id))
        # ロボット状態「アイドル」送信
        self._publish_robot_status(robot_id, SystemDefine.ROBOT_STATUS_IDLE)
        # ロボット状態「無表示」送信
        rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(robot_id))
        self._publish_display_status(robot_id, SystemDefine.DISPLAY_IDLE) 
        break
      else:
        # 次のジョブ開始

         # 停止したのは開始位置移動ジョブ
        if self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_STARTPOSTMOVE:
          # ロボット001が訪問客とみなした障害物を検知
          #rospy.loginfo('obstacle_flg :{}'.format(obstacle_flg))
          if obstacle_flg == True and robot_id == '001':
            # 障害物正対ジョブ開始
            destpoints = get_adpatrol_startpoint(int(robot_id))
            robot_ctx_1["stop"] = False
            form_dummy = DestPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            th_robot_job = StandFrontJob(robot_id, form_dummy, robot_ctx_1, JOBType.JOB_STARTPOSTMOVE)
            th_robot_job.start() 
            self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STANDFRONT
            self.robot_job_map[robot_id].robotjob = th_robot_job

          # ロボット002が案内引き継ぎのためのジョブ停止
          elif handover_flg == True and robot_id == '002':
            handover_flg = False
            # ロボット001の現在地点（向かっていた巡回地点）を取得
            n = self.robot_job_map['001'].robot_current_destination
            # ロボット002の現在地点（向かっていた巡回地点）を取得
            another_n = self.robot_job_map['002'].robot_current_destination
            # 引き継ぎ地選択
            handover_pos = select_handover_point(n, another_n)
            # ロボット001を動かす
            event_meetup.set()

            if handover_pos != None:
              # 案内ジョブ（目的地は引き継ぎ地）開始
              robot_ctx_2["stop"] = False
              pos = get_handover_gap_point(robot_id, handover_pos)
              destpoints = []
              destpoints.append(pos)
              th_robot_job = HandoverJob(robot_id, destpoints, robot_ctx_2)
              th_robot_job.start()
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_HANDOVER
              self.robot_job_map[robot_id].robotjob = th_robot_job
            else:
              # 開始位置移動ジョブ開始
              # DBから巡回開始位置を取得する  
              destpoints = get_adpatrol_startpoint(int(robot_id))
              robot_ctx_2["stop"] = False
              th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_2, True)
              th_robot_job.start() 
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE
              self.robot_job_map[robot_id].robotjob = th_robot_job

          else:
            current_count = self.robot_job_map[robot_id].robot_patrol_count
            rospy.loginfo('ROBOT{} : 巡回回数 現在 {} : 最終 {}'.format(robot_id,current_count,SystemDefine.ADPATROL_TIMES))

            if current_count == SystemDefine.ADPATROL_TIMES:
              # 広告巡回回数到達したのでミッション終了
              self.robot_job_map[robot_id].robot_jobtype = JOBType.UNKNOWN
              self.robot_job_map[robot_id].robot_patrol_count = 0
              self.robot_job_map[robot_id].robot_current_destination = 1
              rospy.loginfo('ミッション停止 ロボット識別子={}'.format(robot_id))
              # ロボット状態「アイドル」送信
              self._publish_robot_status(robot_id, SystemDefine.ROBOT_STATUS_IDLE)
              # ロボット状態「無表示」送信
              rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(robot_id))
              self._publish_display_status(robot_id, SystemDefine.DISPLAY_IDLE) 
              break

            else:
              # 広告巡回ジョブ開始
              # DBから巡回経路を取得する        
              route = get_adpatrol_path(int(robot_id))
              destpoints = []
              for i in range(len(route)):
                if i > 0:
                  destpoints.append(route[i])
              if robot_id == '001':
                robot_ctx_1["stop"] = False
                th_robot_job = AdPatrolJob(robot_id, destpoints, robot_ctx_1, 1)
                th_robot_job.start()
              else:
                robot_ctx_2["stop"] = False
                th_robot_job = AdPatrolJob(robot_id, destpoints, robot_ctx_2, 1)
                th_robot_job.start()
              self.robot_job_map[robot_id].robot_patrol_count = self.robot_job_map[robot_id].robot_patrol_count + 1
              self.robot_job_map[robot_id].robot_current_destination = 1
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_ADPATROL
              self.robot_job_map[robot_id].robotjob = th_robot_job

        # 停止したのは広告巡回ジョブ
        elif self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_ADPATROL:
          # ロボット001が訪問客とみなした障害物を検知
          #rospy.loginfo('obstacle_flg :{}'.format(obstacle_flg))
          if obstacle_flg == True and robot_id == '001':
            # 障害物正対ジョブ開始
            destpoints = get_adpatrol_startpoint(int(robot_id))
            robot_ctx_1["stop"] = False
            form_dummy = DestPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            th_robot_job = StandFrontJob(robot_id, form_dummy, robot_ctx_1, JOBType.JOB_ADPATROL)
            th_robot_job.start() 
            self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STANDFRONT
            self.robot_job_map[robot_id].robotjob = th_robot_job

          # ロボット001が訪問客から案内先を選択された
          elif guide_dest_pos != None and robot_id == '001':
            # ロボット001の現在地点（向かっていた巡回地点）を取得
            n = self.robot_job_map[robot_id].robot_current_destination

            # 引き継ぎ判定
            handover_pos = None
            handover_flg = judge_handover(n, guide_dest_pos)

            if handover_flg == True:
              rospy.loginfo('引き継ぎありと判定')
              # 引き継ぎあり案内
              # ロボット002のジョブ停止
              robot_ctx_2["stop"] = True
              # ロボット001のジョブは開始せず、ロボット002のジョブ停止を待つ
              event_meetup.wait()
              event_meetup.clear()

              # 案内ジョブ（目的地は引き継ぎ地）開始
              if handover_pos != None:
                robot_ctx_1["stop"] = False
                pos = get_handover_gap_point(robot_id, handover_pos)
                destpoints = []
                destpoints.append(pos)
                th_robot_job = HandoverJob(robot_id, destpoints, robot_ctx_1)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_HANDOVER
                self.robot_job_map[robot_id].robotjob = th_robot_job
              else:
                # 引き継ぎなし案内
                # 案内ジョブ（目的地は案内先）開始
                robot_ctx_1["stop"] = False
                destpoints = []
                destpoints.append(guide_dest_pos)
                th_robot_job = GuideJob(robot_id, destpoints, robot_ctx_1)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_GUIDE
                self.robot_job_map[robot_id].robotjob = th_robot_job

            else:
              # 引き継ぎなし案内
              # 案内ジョブ（目的地は案内先）開始
              robot_ctx_1["stop"] = False
              destpoints = []
              destpoints.append(guide_dest_pos)
              th_robot_job = GuideJob(robot_id, destpoints, robot_ctx_1)
              th_robot_job.start()
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_GUIDE
              self.robot_job_map[robot_id].robotjob = th_robot_job

          # ロボット002が案内引き継ぎのためのジョブ停止
          elif handover_flg == True and robot_id == '002':
            handover_flg = False
            # ロボット001の現在地点（向かっていた巡回地点）を取得
            n = self.robot_job_map['001'].robot_current_destination
            # ロボット002の現在地点（向かっていた巡回地点）を取得
            another_n = self.robot_job_map['002'].robot_current_destination
            # 引き継ぎ地選択
            handover_pos = select_handover_point(n, another_n)
            # ロボット001を動かす
            event_meetup.set()

            if handover_pos != None:
              # 案内ジョブ（目的地は引き継ぎ地）開始
              robot_ctx_2["stop"] = False
              pos = get_handover_gap_point(robot_id, handover_pos)
              destpoints = []
              destpoints.append(pos)
              th_robot_job = HandoverJob(robot_id, destpoints, robot_ctx_2)
              th_robot_job.start()
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_HANDOVER
              self.robot_job_map[robot_id].robotjob = th_robot_job
            else:
              # 開始位置移動ジョブ開始
              # DBから巡回開始位置を取得する  
              destpoints = get_adpatrol_startpoint(int(robot_id))
              robot_ctx_2["stop"] = False
              th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_2, True)
              th_robot_job.start() 
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE
              self.robot_job_map[robot_id].robotjob = th_robot_job

          else:
            # 開始位置移動ジョブ開始
            # DBから巡回開始位置を取得する  
            destpoints = get_adpatrol_startpoint(int(robot_id))
            if robot_id == '001':
              robot_ctx_1["stop"] = False
              th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_1, True)
              th_robot_job.start()
            else:
              robot_ctx_2["stop"] = False
              th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_2, True)
              th_robot_job.start() 
            self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE
            self.robot_job_map[robot_id].robotjob = th_robot_job

        # 停止したのは障害物正対ジョブ
        elif self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_STANDFRONT:
          obstacle_flg = False
          # ロボット001が訪問客から案内先を選択された
          if guide_dest_pos != None and robot_id == '001':

            # ロボット001の現在地点（向かっていた巡回地点）を取得
            n = self.robot_job_map[robot_id].robot_current_destination

            # 引き継ぎ判定
            handover_pos = None
            handover_flg = judge_handover(n, guide_dest_pos)

            if handover_flg == True:
              rospy.loginfo('引き継ぎありと判定')
              # 引き継ぎあり案内
              # ロボット002のジョブ停止
              robot_ctx_2["stop"] = True
              # ロボット001のジョブは開始せず、ロボット002のジョブ停止を待つ
              event_meetup.wait()
              event_meetup.clear()
              # 案内ジョブ（目的地は引き継ぎ地）開始
              if handover_pos != None:
                robot_ctx_1["stop"] = False
                pos = get_handover_gap_point(robot_id, handover_pos)
                destpoints = []
                destpoints.append(pos)
                th_robot_job = HandoverJob(robot_id, destpoints, robot_ctx_1)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_HANDOVER
                self.robot_job_map[robot_id].robotjob = th_robot_job
              else:
                # 引き継ぎなし案内
                # 案内ジョブ（目的地は案内先）開始
                robot_ctx_1["stop"] = False
                destpoints = []
                destpoints.append(guide_dest_pos)
                th_robot_job = GuideJob(robot_id, destpoints, robot_ctx_1)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_GUIDE
                self.robot_job_map[robot_id].robotjob = th_robot_job

            else:
              # 引き継ぎなし案内
              # 案内ジョブ（目的地は案内先）開始
              robot_ctx_1["stop"] = False
              destpoints = []
              destpoints.append(guide_dest_pos)
              th_robot_job = GuideJob(robot_id, destpoints, robot_ctx_1)
              th_robot_job.start()
              self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_GUIDE
              self.robot_job_map[robot_id].robotjob = th_robot_job

          # ロボット001が訪問客から案内先を選択されずに待機タイムアウト
          else:
            n = self.robot_job_map[robot_id].robot_current_destination
            rospy.loginfo('停止時の目的地は{}番目 ロボット識別子={}'.format(n,robot_id))

            # 障害物正対ジョブ開始前のジョブにより開始ジョブが異なる

            # 開始位置移動ジョブ
            if before_jobtype == JOBType.JOB_STARTPOSTMOVE:

              if n == 2:   # 開始位置に到着している
                current_count = self.robot_job_map[robot_id].robot_patrol_count
                rospy.loginfo('ROBOT{} : 巡回回数 現在 {} : 最終 {}'.format(robot_id,current_count,SystemDefine.ADPATROL_TIMES))
                if current_count == SystemDefine.ADPATROL_TIMES:
                  # 広告巡回回数到達したのでミッション終了
                  self.robot_job_map[robot_id].robot_jobtype = JOBType.UNKNOWN
                  self.robot_job_map[robot_id].robot_patrol_count = 0
                  self.robot_job_map[robot_id].robot_current_destination = 1
                  rospy.loginfo('ミッション停止 ロボット識別子={}'.format(robot_id))
                  # ロボット状態「アイドル」送信
                  self._publish_robot_status(robot_id, SystemDefine.ROBOT_STATUS_IDLE)
                  # ロボット状態「無表示」送信
                  rospy.loginfo('ROBOT{} : ディスプレイ表示 :無表示'.format(robot_id))
                  self._publish_display_status(robot_id, SystemDefine.DISPLAY_IDLE) 
                  break
                else:
                  # 広告巡回ジョブ開始
                  # DBから巡回経路を取得する        
                  route = get_adpatrol_path(int(robot_id))
                  destpoints = []
                  for i in range(len(route)):
                    if i > 0:
                      destpoints.append(route[i])
                  if robot_id == '001':
                    robot_ctx_1["stop"] = False
                    th_robot_job = AdPatrolJob(robot_id, destpoints, robot_ctx_1, 1)
                    th_robot_job.start()
                  else:
                    robot_ctx_2["stop"] = False
                    th_robot_job = AdPatrolJob(robot_id, destpoints, robot_ctx_2, 1)
                    th_robot_job.start()
                  self.robot_job_map[robot_id].robot_patrol_count = self.robot_job_map[robot_id].robot_patrol_count + 1
                  self.robot_job_map[robot_id].robot_current_destination = 1
                  self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_ADPATROL
                  self.robot_job_map[robot_id].robotjob = th_robot_job

              else:   # 開始位置に到着していない
                # 開始位置移動ジョブ開始
                destpoints = get_adpatrol_startpoint(int(robot_id))
                robot_ctx_1["stop"] = False
                th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_1, True)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE
                self.robot_job_map[robot_id].robotjob = th_robot_job

            # 広告巡回ジョブ
            else:
              # DBから巡回経路を取得する      
              destpoints_all = get_adpatrol_path(int(robot_id))

              if n == len(destpoints_all):   # 最終巡回地に到着している
                # 開始位置移動ジョブ開始
                destpoints = get_adpatrol_startpoint(int(robot_id))
                robot_ctx_1["stop"] = False
                th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_1, True)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE
                self.robot_job_map[robot_id].robotjob = th_robot_job

              else:   # 最終巡回地に到着していない
                # 広告巡回ジョブを途中から再開
                destpoints = []
                for i in range(len(destpoints_all)):
                  if i > 0:   #開始位置は飛ばして取得する
                    print(destpoints_all[i])
                    destpoints.append(destpoints_all[i])
                robot_ctx_1["stop"] = False
                th_robot_job = AdPatrolJob(robot_id, destpoints, robot_ctx_1, n)
                th_robot_job.start()
                self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_ADPATROL
                self.robot_job_map[robot_id].robotjob = th_robot_job

        # 停止したのは案内ジョブ（目的地は案内先）
        elif self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_GUIDE:
          guide_dest_pos = None
          # 開始位置移動ジョブ開始
          # DBから巡回開始位置を取得する  
          destpoints = get_adpatrol_startpoint(int(robot_id))
          if robot_id == '001':
            robot_ctx_1["stop"] = False
            th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_1, True)
            th_robot_job.start()
          else:
            robot_ctx_2["stop"] = False
            th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_2, True)
            th_robot_job.start()
          self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE
          self.robot_job_map[robot_id].robotjob = th_robot_job

        # 停止したのは案内ジョブ（目的地は引き継ぎ地）
        elif self.robot_job_map[robot_id].robot_jobtype == JOBType.JOB_HANDOVER:
          if robot_id == '001':
            # 開始位置移動ジョブ開始
            self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_STARTPOSTMOVE

            # 両方のロボットのジョブ停止を待ち合わせをする
            if self.robot_job_map['002'].robot_jobtype == JOBType.JOB_HANDOVER:
              event_separate.wait()
              event_separate.clear()
            else:
              event_separate.set()

            # DBから巡回開始位置を取得する  
            destpoints = get_adpatrol_startpoint(int(robot_id))
            robot_ctx_1["stop"] = False
            th_robot_job = StartPosMoveJob(robot_id, destpoints, robot_ctx_1, True)
            th_robot_job.start()
            self.robot_job_map[robot_id].robotjob = th_robot_job
          else:
            # 案内ジョブ（目的地は案内先）開始
            self.robot_job_map[robot_id].robot_jobtype = JOBType.JOB_GUIDE

            # 両方のロボットのジョブ停止を待ち合わせをする
            if self.robot_job_map['001'].robot_jobtype == JOBType.JOB_HANDOVER:
              event_separate.wait()
              event_separate.clear()
            else:
              event_separate.set()

            robot_ctx_2["stop"] = False
            destpoints = []
            destpoints.append(guide_dest_pos)
            th_robot_job = GuideJob(robot_id, destpoints, robot_ctx_2)
            th_robot_job.start()
            self.robot_job_map[robot_id].robotjob = th_robot_job

        else:
          pass

        time.sleep(0.5)

    rospy.loginfo('ジョブ終了待ちスレッド終了 ロボット識別子={}'.format(robot_id))


  # 案内先受信
  def _callback_destination(self, msg):
    global robot_ctx_1
    global guide_dest_pos
    global obstacle_flg
    rospy.loginfo('**********************************')
    rospy.loginfo('案内先受信 ポイント番号 :{}'.format(msg.destination_no))
    rospy.loginfo('**********************************')
    # 障害物検知しているなら、案内先を保存してロボット001の障害物正対ジョブ停止
    rospy.loginfo('obstacle_flg :{}'.format(obstacle_flg))
    if obstacle_flg == True:
      guide_dest_pos = get_point_coordinate(msg.destination_no)
      print(guide_dest_pos)

      if self.robot_job_map['001'].robot_jobtype == JOBType.JOB_STANDFRONT:
        robot_ctx_1["stop"] = True
      else:
        rospy.loginfo('ロボット001の状態不正 現在のジョブ :{}'.format(self.robot_job_map['001'].robot_jobtype))
    else:
      # 障害物検知しているなら、案内先を保存してロボット001の広告巡回ジョブ停止
      guide_dest_pos = get_point_coordinate(msg.destination_no)
      print(guide_dest_pos)

      if self.robot_job_map['001'].robot_jobtype == JOBType.JOB_ADPATROL:
        if self.robot_job_map['002'].robot_jobtype == JOBType.JOB_ADPATROL \
            or self.robot_job_map['002'].robot_jobtype == JOBType.JOB_STARTPOSTMOVE:
          robot_ctx_1["stop"] = True
        else:
          rospy.loginfo('ロボット002の状態不正 現在のジョブ :{}'.format(self.robot_job_map['002'].robot_jobtype))
      else:
        rospy.loginfo('ロボット001の状態不正 現在のジョブ :{}'.format(self.robot_job_map['001'].robot_jobtype))

  # ロボット001からの障害物情報受信
  def _callback_obstacle(self, msg):
    global mission_mode
    global obstacle_flg

    # 障害物なしなら無視する
    if msg.obstacle_flg == SystemDefine.OBSTACLE_FLG_OFF:
      return

    # すでに障害物検知しているなら無視する
    if obstacle_flg == True:
      return
    
    if mission_mode == True:
      # 広告巡回ジョブ中、または開始位置移動ジョブ中であれば訪問客とみなし、ロボット001のジョブ停止
      if msg.obstacle_flg == SystemDefine.OBSTACLE_FLG_ON:
        if self.robot_job_map['001'].robot_jobtype == JOBType.JOB_ADPATROL \
          or self.robot_job_map['001'].robot_jobtype == JOBType.JOB_STARTPOSTMOVE:
          if self.robot_job_map['002'].robot_jobtype == JOBType.JOB_ADPATROL \
            or self.robot_job_map['002'].robot_jobtype == JOBType.JOB_STARTPOSTMOVE:
            rospy.loginfo('**********************************')
            rospy.loginfo('障害物情報受信 :{}'.format(msg.obstacle_flg))
            rospy.loginfo('**********************************')
            obstacle_flg = True
            robot_ctx_1["stop"] = True

  # ロボット状態送信
  def _publish_robot_status(self, id, status):
    rospy.loginfo('ROBOT{} : ロボット状態 :{}'.format(id,status))
    _robotStaus = RobotStaus()
    _robotStaus.robot_status = status
    publisher = self.robotStaus_publisher_map[id]
    publisher.publish(_robotStaus)

  # ディスプレイ表示送信
  def _publish_display_status(self, id, status):
    #rospy.loginfo('ROBOT{} : ディスプレイ表示 :{}'.format(id,status))
    _displayStatus = DisplayStatus()
    _displayStatus.status = status
    publisher = self.displayStaus_publisher_map[id]
    publisher.publish(_displayStatus)

  # デストラクタ  
  def __del__(self):
    pass

