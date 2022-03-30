#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import psycopg2 
import os
import time
#import urllib.parse
from move_management.msg import TransManagerCommandReq, JointManagerCommandReq
from move_management.msg import TransReq, TransRes, RouteSP, TransformStampedEuler
from move_management.msg import LifterControlReq, ManagerCommandRes, JointManagerCommandRes
from move_management.msg import Obstacle, ObstacleInformation
from move_management.msg import LifterControlRes
from move_manager_define import MoveManagementDefine
from threading import Thread

dict_device_management = {}
device_lists = ['thk001', 'thk002']

#RouteSPへ値を格納するための構造体
class move_point():
  def __init__(self):
    self.pos_x = 0
    self.pos_y = 0
    self.pos_z = 0
    self.ori_p = 0
    self.ori_r = 0
    self.ori_y = 0

#移動指示用クラス
#ミッションから取得した巡回経路をDBから取り出し、移動経路へ送信
#移動結果の取得

class DeviceMoveManagement():
  def __init__(self, name):
    self.robot_name = name
    rospy.loginfo("【{}】【移動管理】移動管理起動".format(self.robot_name))

    self.status = MoveManagementDefine()
    self.status.DEVICE_STATE = "アイドル"

    #メッセージ保存用変数
    self.move_req_cp = TransManagerCommandReq()
    self.move_point = TransformStampedEuler()
    self.move_req_msg = TransReq()
    self.lifter_req_cp = JointManagerCommandReq()

    #シーケンス番号(カウント)
    self.count_seq = 1
    #障害物フラグ
    self.Obstacle_flg_now = 0

    #リルート開始要素番号
    self.reroute_index = None

    self.move_req_seq = 0
    self.turn_req_seq = 0

    # デバイス名保存
    self.device_name = name

    #Publisher
    #移動/回転/停止指示
    self.Pub_move_req = rospy.Publisher('/' + self.device_name + '/trs_req', TransReq, queue_size=10)
    #移動応答
    self.Pub_move_res = rospy.Publisher('/trsmng_res', ManagerCommandRes, queue_size=10)
    #リフター指示
    self.Pub_lifter_req = rospy.Publisher('/'+self.device_name + '/joint_ctl_req', LifterControlReq, queue_size=10)
    #リフター応答
    self.Pub_lifter_res = rospy.Publisher('/jointmng_res', JointManagerCommandRes, queue_size=10)

  #DBアクセスとテーブルからデータ取り出し、route_ps_infoへ値を挿入する
  def DB_point_info_in(self, robot_id, pos_x, pos_y, pos_z):
    #postgres ログイン
#    urllib.parse.uses_netloc.append("postgres")
#    db_url = urllib.parse.urlparse(os.environ.get('DATABASE_URL'))
#    users = db_url.username
#    dbnames = db_url.path[1:]
#    passwords = db_url.password
#    conn = psycopg2.connect("user=" + users + " dbname=" + dbnames +" password = " + passwords)
    conn = psycopg2.connect("user=postgres dbname=msleq")
    cur = conn.cursor()

    rospy.loginfo("goal  pos_x : {} pos_y : {} pos_z : {}".format(str(pos_x) ,pos_y, pos_z))
    print(str(pos_x))

    #ゴールポイント(目的地)指定
    cur.execute('SELECT point_no FROM point_info where pos_x = ' + str(pos_x) +' and pos_y = ' + str(pos_y) + ' and pos_z = ' + str(pos_z) + ';')
    for row in cur:
      goal_point = row[0]
    print(goal_point)
    #現在位置の指定
    # nabe chg start
    self.now_pos_x
    cur.execute('SELECT point_no FROM point_info where pos_x = \
      %s and pos_y = %s and pos_z = %s', (self.now_pos_x, self.now_pos_y, self.now_pos_z,))
    results = cur.fetchall()
    for row in results:
      current_point = row[0]
      print(current_point)

    #ゴールポイントまでの経路の指定
    goal_route = cur.execute('SELECT points FROM guide_path_info where goal_point=' + str(goal_point) + ' and current_point = ' + str(current_point) + ';')
    for row in cur:
      goal_route = row[0]
    rospy.loginfo("g_p : {},  c_p : {}, g_r : {}".format(goal_point, current_point,  goal_route))
    #ゴールまでの経路数を格納
    self.goal_point_num = len(goal_route)

    rospy.loginfo("len {} ".format(len(goal_route)))

    for i in range(len(goal_route)):
      rospy.loginfo("goal_route {}  {}".format(i, goal_route[i]))
      cur.execute('SELECT * FROM point_info where point_no = ' + str(goal_route[i]) + ';')
      #print("point_no {}".format(cur.fetchall()))
      for row in cur:
        #構造体 move_pointを使用して　DBから取り出した値を取り出す
        print("point : {}".format(row))
        rospy.loginfo("x : {} y : {} z : {} ".format(row[2], row[3], row[4]))
        move_p = TransformStampedEuler()
        move_p.transform.translation.x = row[2]
        move_p.transform.translation.y = row[3]
        move_p.transform.translation.z = row[4]
        move_p.transform.rotation.ori_p = row[5]
        move_p.transform.rotation.ori_r = row[6]
        move_p.transform.rotation.ori_y = row[7]
        #self.route_ps_info.point_info = self.move_p
      
      route_ps_info = RouteSP()
      #rotue_ps_infoへ格納
      route_ps_info.trs_flg = self.move_req_cp.goal.trs_flg
      route_ps_info.point_info=move_p
      route_ps_info.whl_sp_info.linear.x = self.move_req_cp.goal.whl_sp_info.linear.x
      route_ps_info.whl_sp_info.linear.y = self.move_req_cp.goal.whl_sp_info.linear.y
      route_ps_info.whl_sp_info.linear.z = self.move_req_cp.goal.whl_sp_info.linear.z
      route_ps_info.whl_sp_info.angular.x = self.move_req_cp.goal.whl_sp_info.angular.x
      route_ps_info.whl_sp_info.angular.y = self.move_req_cp.goal.whl_sp_info.angular.y
      route_ps_info.whl_sp_info.angular.z = self.move_req_cp.goal.whl_sp_info.angular.z
      
      self.move_req_msg.route_ps_info.append(route_ps_info)
      self.status.ROUTE_LIST.append(route_ps_info)
      rospy.loginfo("【{}】ROUTE_LIST : {}".format(self.robot_name, self.status.ROUTE_LIST))

  #移動要求
  def move_instructer_publish(self, msg):
    self.move_req_cp = msg
    #既に入っている場合(消去できていない)は空にする
    if len(self.status.ROUTE_LIST) >= 1:
      self.status.ROUTE_LIST = []
    if self.move_req_cp.goal.trs_flg == 1:
      # 移動なら経路作成
      rospy.loginfo("change state move {}".format(self.robot_name))
      goal_pos_x = msg.goal.point_info.transform.translation.x
      goal_pos_y = msg.goal.point_info.transform.translation.y
      goal_pos_z = msg.goal.point_info.transform.translation.z

      if self.move_req_cp.trans_kind == 2:
        rospy.loginfo("【{}】【移動管理】サーバ指示受信".format(self.robot_name))
        #サーバ指示が選ばれた場合
        #目的地(座標)をmsgから取り出す
        rospy.loginfo("【{}】【移動管理】指定座標 x: {} y: {} z: {}".format(self.robot_name, goal_pos_x, goal_pos_y, goal_pos_z))
        #DBへアクセスし、route_ps_infoへ値を入れる
        self.DB_point_info_in(self.move_req_cp.robot_id, goal_pos_x, goal_pos_y, goal_pos_z)
        self.move_req_msg.trans_seq = self.count_seq
        self.move_req_msg.trans_kind = msg.trans_kind
        self.move_req_msg.route_ps_num = self.goal_point_num
        #rospy.loginfo("msg : {}".format(self.move_req_msg))
        print("change sep {} ".format(self.robot_name))
        self.count_seq += 1
        self.move_req_seq = self.move_req_msg.trans_seq
        rospy.loginfo("【{}】【移動管理】移動指示受信".format(self.robot_name))
        # 障害物がなければ移動する
        if self.Obstacle_flg_now == 0:
          self.status.DEVICE_STATE = "移動"
          rospy.logdebug("move_req_seq : {}".format(self.move_req_seq))
          self.Pub_move_req.publish(self.move_req_msg)

      elif self.move_req_cp.trans_kind == 1:
        rospy.loginfo("【{}】【移動管理】自律走行指示受信".format(self.robot_name))
        rospy.loginfo("【{}】【移動管理】指定座標 x: {} y: {} z: {}".format(self.robot_name, goal_pos_x, goal_pos_y, goal_pos_z))
        route_ps_start = RouteSP()
        self.move_req_msg.trans_seq = self.count_seq
        self.move_req_msg.trans_kind = self.move_req_cp.trans_kind
        self.move_req_msg.route_ps_num = 1 # 開始地点へ移動するのみのため
        route_ps_start.trs_flg = self.move_req_cp.goal.trs_flg
        route_ps_start.point_info.transform.translation.x = self.move_req_cp.goal.point_info.transform.translation.x
        route_ps_start.point_info.transform.translation.y = self.move_req_cp.goal.point_info.transform.translation.y
        route_ps_start.point_info.transform.translation.z = self.move_req_cp.goal.point_info.transform.translation.z
        route_ps_start.point_info.transform.rotation.ori_p = self.move_req_cp.goal.point_info.transform.rotation.ori_p
        route_ps_start.point_info.transform.rotation.ori_r = self.move_req_cp.goal.point_info.transform.rotation.ori_r
        route_ps_start.point_info.transform.rotation.ori_y = self.move_req_cp.goal.point_info.transform.rotation.ori_y
        route_ps_start.whl_sp_info.linear.x = self.move_req_cp.goal.whl_sp_info.linear.x
        route_ps_start.whl_sp_info.linear.y = self.move_req_cp.goal.whl_sp_info.linear.y
        route_ps_start.whl_sp_info.linear.z = self.move_req_cp.goal.whl_sp_info.linear.z
        route_ps_start.whl_sp_info.angular.x = self.move_req_cp.goal.whl_sp_info.angular.x
        route_ps_start.whl_sp_info.angular.y = self.move_req_cp.goal.whl_sp_info.angular.y
        route_ps_start.whl_sp_info.angular.z = self.move_req_cp.goal.whl_sp_info.angular.z
        self.move_req_msg.route_ps_info.append(route_ps_start)
        self.status.ROUTE_LIST.append(route_ps_start)
        #rospy.loginfo("msg : {}".format(self.move_req_msg))
        print("change sep {} ".format(self.robot_name))
        self.count_seq += 1
        self.move_req_seq = self.move_req_msg.trans_seq
        rospy.logdebug("move_req_seq : {}".format(self.move_req_seq))
        rospy.loginfo("【{}】【移動管理】移動指示受信".format(self.robot_name)) 
        self.Pub_move_req.publish(self.move_req_msg)
    elif self.move_req_cp.goal.trs_flg == 2:
      # 回転
      #self.status.DEVICE_STATE = "移動"
      rospy.loginfo("change state rotate {}".format(self.robot_name))
      route_ps_start = RouteSP()
      self.move_req_msg.trans_seq = self.count_seq
      self.move_req_msg.trans_kind = self.move_req_cp.trans_kind
      self.move_req_msg.route_ps_num = 1
      route_ps_start.trs_flg = self.move_req_cp.goal.trs_flg
      route_ps_start.point_info.transform.translation.x = self.move_req_cp.goal.point_info.transform.translation.x
      route_ps_start.point_info.transform.translation.y = self.move_req_cp.goal.point_info.transform.translation.y
      route_ps_start.point_info.transform.translation.z = self.move_req_cp.goal.point_info.transform.translation.z
      route_ps_start.point_info.transform.rotation.ori_p = self.move_req_cp.goal.point_info.transform.rotation.ori_p
      route_ps_start.point_info.transform.rotation.ori_r = self.move_req_cp.goal.point_info.transform.rotation.ori_r
      route_ps_start.point_info.transform.rotation.ori_y = self.move_req_cp.goal.point_info.transform.rotation.ori_y
      route_ps_start.whl_sp_info.linear.x = self.move_req_cp.goal.whl_sp_info.linear.x
      route_ps_start.whl_sp_info.linear.y = self.move_req_cp.goal.whl_sp_info.linear.y
      route_ps_start.whl_sp_info.linear.z = self.move_req_cp.goal.whl_sp_info.linear.z
      route_ps_start.whl_sp_info.angular.x = self.move_req_cp.goal.whl_sp_info.angular.x
      route_ps_start.whl_sp_info.angular.y = self.move_req_cp.goal.whl_sp_info.angular.y
      route_ps_start.whl_sp_info.angular.z = self.move_req_cp.goal.whl_sp_info.angular.z
      self.move_req_msg.route_ps_info.append(route_ps_start)
      self.count_seq += 1
      self.turn_req_seq = self.move_req_msg.trans_seq
      rospy.logdebug("turn_req_seq : {}".format(self.turn_req_seq))
      rospy.loginfo("【{}】【移動管理】移動指示受信(回転)".format(self.robot_name))
      self.Pub_move_req.publish(self.move_req_msg)

    else:
      rospy.loginfo("【{}】【移動管理】移動指示受信(停止)".format(self.robot_name))
      self.move_res_msg = ManagerCommandRes()
      self.move_res_msg.robot_id = self.move_req_cp.robot_id
      self.move_res_msg.result = 2
      self.move_req_msg.route_ps_info = []
      self.Pub_move_res.publish(self.move_res_msg)
      self.status.ROUTE_LIST = []

  #リルート指示
  def  reroute_move(self, reroute_num):
    #リルート用メッセージ作成
    reroute_ps_msg = TransReq()
    rospy.loginfo("【{}】【移動管理】リルート指示開始".format(self.robot_name))
    point_cnt = 0
    #リルート後の点群挿入
    for num  in range(reroute_num-1, len(self.status.ROUTE_LIST)-1):
      route_ps_cp = RouteSP()
      route_ps_cp = self.status.ROUTE_LIST[num]
      reroute_ps_msg.route_ps_info.append(route_ps_cp)
      point_cnt += 1
    #メッセージへ値格納
    reroute_ps_msg.trans_seq = self.count_seq
    reroute_ps_msg.trans_kind = self.move_req_cp.trans_kind
    reroute_ps_msg.route_ps_num = point_cnt # リルート後の地点数
    rospy.loginfo("【{}】【移動管理】リルート地点 {}".format(self.robot_name, reroute_ps_msg))
    self.count_seq += 1 
    self.move_req_seq = reroute_ps_msg.trans_seq 
    rospy.logdebug("move_req_seq : {}".format(self.move_req_seq))
    self.status.ROUTE_LIST = []
    #publish
    self.Pub_move_req.publish(reroute_ps_msg)

  #移動、回転、停止応答(リルート要求含む)
  def move_res_callback(self, msg):
    if self.status.DEVICE_STATE == "回転中":
      self.status.DEVICE_STATE = "停止"
      rospy.loginfo("【{}】【移動管理】状態を回転中->停止へ変更しました。".format(self.robot_name))
      return 
    rospy.loginfo("【{}】【移動管理】移動応答(リルート要求含む) 送信 移動結果 : {}".format(self.robot_name, msg.trs_result))
    #リルート開始要素番号の挿入
    self.reroute_index = msg.reroute_index
    rospy.loginfo("リルート番号 : {}".format(self.reroute_index))
    self.move_res_msg = ManagerCommandRes()
    self.move_res_msg.robot_id = self.move_req_cp.robot_id
    self.move_res_msg.result = msg.trs_result
    if msg.trans_seq == self.move_req_seq:
      #要求のRoute_SPの中身を初期化する
      self.move_req_msg.route_ps_info = []
      if msg.trs_result == 2:   # 正常終了
        self.status.DEVICE_STATE = "アイドル"
        self.Pub_move_res.publish(self.move_res_msg)
        #正常終了したので目的地を現在地として保存
        self.now_pos_x = self.move_req_cp.goal.point_info.transform.translation.x
        self.now_pos_y = self.move_req_cp.goal.point_info.transform.translation.y
        self.now_pos_z = self.move_req_cp.goal.point_info.transform.translation.z
        #正常終了時はROUTE_LISTの中身を初期化する
        self.status.ROUTE_LIST = []
     
    elif msg.trans_seq == self.turn_req_seq:
      self.Pub_move_res.publish(self.move_res_msg)
      self.move_req_msg.route_ps_info = []
    print("now_pos = x : {} y : {} z : {} ".format(self.now_pos_x, self.now_pos_y, self.now_pos_z))

  #障害物検知callback
  def obstacles_callback(self, msg):
    move_state_msg = TransReq()
    route_ps = RouteSP()
    self.Obstacle_flg_now = msg.obstacle_flg
    #print("self.Obstacle_flg_now={}".format(self.Obstacle_flg_now))
    #rospy.loginfo("【{}】【移動管理】状態は{}です".format(self.robot_name,self.status.DEVICE_STATE))
    
    if self.status.DEVICE_STATE == "移動":
      if self.Obstacle_flg_now == 1:
        #停止指示
        rospy.loginfo("【{}】【移動管理】障害物検知 障害物情報受信".format(self.robot_name))
        self.status.DEVICE_STATE = "待機"
        rospy.loginfo("【{}】【移動管理】状態を移動->待機へ変更しました。".format(self.robot_name))
        move_state_msg.trans_seq = self.count_seq
        move_state_msg.trans_kind = self.move_req_cp.trans_kind
        route_ps.trs_flg = 3
        route_ps.whl_sp_info.linear.x = 0
        route_ps.whl_sp_info.linear.y = 0
        route_ps.whl_sp_info.linear.z = 0
        route_ps.whl_sp_info.angular.x = 0
        route_ps.whl_sp_info.angular.y = 0
        route_ps.whl_sp_info.angular.z = 0
        move_state_msg.route_ps_info.append(route_ps)
        rospy.loginfo("【{}】【移動管理】障害物検知 状態: 待機 停止指示送信".format(self.robot_name))
        self.count_seq += 1
        self.Pub_move_req.publish(move_state_msg)
        #1分間停止
        rospy.sleep(MoveManagementDefine.WAIT_TIME)

        if self.Obstacle_flg_now == 1:
          # 1分後も障害物ありなら45度回転
          self.status.DEVICE_STATE = "回転中"
          rospy.loginfo("【{}】【移動管理】状態を待機->回転中へ変更しました。".format(self.robot_name))
          rospy.loginfo("【{}】【移動管理】障害物検知 状態 : 待機 45度回転指示送信".format(self.robot_name))
          move_state_msg.trans_seq = self.count_seq
          move_state_msg.trans_kind = self.move_req_cp.trans_kind
          route_ps.trs_flg = 2
          route_ps.point_info.transform.rotation.ori_p = 0
          route_ps.point_info.transform.rotation.ori_r = 0
          route_ps.point_info.transform.rotation.ori_y = MoveManagementDefine.ROTADE_RAD #現在は45度の方へ向く
          move_state_msg.route_ps_info.append(route_ps)
          self.count_seq += 1
          self.Pub_move_req.publish(move_state_msg)

    elif self.status.DEVICE_STATE == "待機":
        pass
        # 動かない

    elif self.status.DEVICE_STATE == "停止":
      if self.Obstacle_flg_now == 0:  # 障害物なし 
        if len(self.move_req_msg.route_ps_info) >= 1:
          self.status.DEVICE_STATE = "移動"
          rospy.loginfo("【{}】【移動管理】状態をアイドル->移動へ変更しました。".format(self.robot_name))
          rospy.loginfo("【{}】【移動管理】障害物検知 状態:アイドル 移動指示送信".format(self.robot_name))
          #ミッションからの移動要求による処理
          rospy.logdebug("move_req_seq : {}".format(self.move_req_seq))
          self.Pub_move_req.publish(self.move_req_msg)  

        elif len(self.status.ROUTE_LIST) >= 1:     
          self.status.DEVICE_STATE = "移動"
          rospy.loginfo("【{}】【移動管理】状態を停止->移動へ変更しました。".format(self.robot_name))
          rospy.loginfo("【{}】【移動管理】障害物検知 状態: 停止 リルート指示送信".format(self.robot_name))
          #リルート処理
          self.reroute_move(self.reroute_index)
    


  #ミッション制御 リフター要求
  def lifter_req_callback_sub(self, msg):
    rospy.loginfo("【{}】【移動管理】リフター要求受信".format(self.robot_name))
    while(self.status.DEVICE_STATE == "回転中"):
      rospy.sleep(MoveManagementDefine.TURN_WAIT_TIME)
    self.lifter_req_cp = msg
    self.lifter_req_msg = LifterControlReq()
    self.lifter_req_msg.lifter_control_kind = msg.lifter_control_kind
    self.Pub_lifter_req.publish(self.lifter_req_msg)
    rospy.loginfo("【{}】【移動管理】リフター指示送信".format(self.robot_name))
  
   #リフター指示応答
  def lifter_res_callback(self, msg):
    rospy.loginfo("【{}】【移動管理】リフター指示応答受信".format(self.robot_name))
    self.lifter_res_msg = JointManagerCommandRes()
    self.lifter_res_msg.robot_id = self.lifter_req_cp.robot_id
    self.lifter_res_msg.result = msg.lifter_control_result
    #リフター応答 Publisher
    self.Pub_lifter_res.publish(self.lifter_res_msg)
    rospy.loginfo("【{}】【移動管理】リフター応答送信".format(self.robot_name))

  # Subscriber用ワーカースレッド関数
  def sub_worker(self):
    #Subscriber
    #移動指示応答
    self.Sub_move_res = rospy.Subscriber('/' + self.device_name + '/trs_res', TransRes, self.move_res_callback)
    #障害物情報
    self.Sub_obstacles = rospy.Subscriber('/' + self.device_name + '/obstacles', Obstacle, self.obstacles_callback)
    #リフター指示応答
    self.Sub_lifter_res = rospy.Subscriber('/'+self.device_name + '/lifter_control_res', LifterControlRes, self.lifter_res_callback)

    while not rospy.is_shutdown():
      rospy.spin()

class MoveManager:
  def __init__(self):
    rospy.loginfo("【移動管理】移動管理起動")

  #移動要求
  def move_req_callback(self, msg):
    rospy.loginfo("【移動管理】移動要求受信")
    # 各DeviceMoveManagementで処理
    dmm = dict_device_management[msg.robot_id]
    dmm.move_instructer_publish(msg)

  #リフター要求
  def lifter_req_callback(self, msg):
    rospy.loginfo("【移動管理】リフター要求受信")
    dmm = dict_device_management[msg.robot_id]
    dmm.lifter_req_callback_sub(msg)

  # Subscriber用ワーカースレッド関数
  def sub_worker(self):
    #ミッション制御からの移動要求
    self.Sub_move_req = rospy.Subscriber('/trsmng_req', TransManagerCommandReq, self.move_req_callback)
    #ミッション制御からのリフター要求
    self.Sub_lifter_req = rospy.Subscriber('/jointmng_req', JointManagerCommandReq, self.lifter_req_callback)

    while not rospy.is_shutdown():
      rospy.spin()

if __name__=='__main__':
  rospy.loginfo('===移動管理コンポーネント開始===')
  rospy.init_node('move_manager')

  index = 1
  #辞書型にrobot_id(index)とクラスオブジェクトを追加
  for device in device_lists:
    dmm = DeviceMoveManagement(device)
    dict_device_management.setdefault(index, dmm)
    index +=1

    # Subscriberワーカースレッド起動
    th_sub = Thread(target=dmm.sub_worker)
    th_sub.start()

  # 移動管理のクラスのインスタンス生成
  mm = MoveManager()
  # Subscriberワーカースレッド起動
  th_sub = Thread(target=mm.sub_worker)
  th_sub.start()

  while not rospy.is_shutdown():
    rospy.spin()

  rospy.loginfo('===移動管理コンポーネント終了===')
