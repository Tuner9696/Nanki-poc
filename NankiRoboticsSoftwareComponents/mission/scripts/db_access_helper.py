#!/usr/bin/env python
# -*- coding: utf-8 -*-


## DBアクセスクラス
#class PostgreConnect:

import os
import psycopg2
import psycopg2.extras
import urllib.parse

# 地点座標格納クラス
class DestPoint:
  def __init__(self, _pos_x,_pos_y,_pos_z,_ori_p,_ori_r,_ori_y):
    self.pos_x = _pos_x
    self.pos_y = _pos_y
    self.pos_z = _pos_z
    self.ori_p = _ori_p
    self.ori_r = _ori_r
    self.ori_y = _ori_y

def get_connection():
    urllib.parse.uses_netloc.append("postgres")
    db_url = urllib.parse.urlparse(os.environ.get('DATABASE_URL'))
    users = db_url.username
    dbnames = db_url.path[1:]
    passwords = db_url.password
    conn = psycopg2.connect("user=" + users + " dbname=" + dbnames +" password = " + passwords)
    return conn
    #return psycopg2.connect("user=postgres dbname=msleq ")

# ポイント番号から座標を取得
def get_point_coordinate(no):
  with get_connection() as conn:
      with conn.cursor() as cur:
          cur.execute('SELECT pos_x,pos_y,pos_z,ori_p,ori_r,ori_y FROM point_info WHERE point_no = %s', (no,))
          results = cur.fetchall()
          for row in results:
            print(row[0], row[1], row[2], row[3], row[4], row[5])
            ret = DestPoint(row[0], row[1], row[2], row[3], row[4], row[5])
  return ret

# 座標からポイント番号を取得
def get_point_no(coordinate):
  with get_connection() as conn:
      with conn.cursor() as cur:
          cur.execute('SELECT point_no FROM point_info where pos_x = \
            %s and pos_y = %s and pos_z = %s and ori_p = %s and ori_r = %s and ori_y = %s', \
            (coordinate.pos_x, coordinate.pos_y, coordinate.pos_z, coordinate.ori_p, coordinate.ori_r, coordinate.ori_y,))
          results = cur.fetchall()
          for row in results:
            print(row[0])
            ret = row[0]
  return ret

# 巡回経路の座標を取得
def get_adpatrol_path(robot_id):
  ret_pos = []
  with get_connection() as conn:
      with conn.cursor() as cur:
          cur.execute('SELECT points FROM patrol_path_info WHERE robot_id = %s', (robot_id,))
          pathes = cur.fetchall()
          for path in pathes:
            for points in path:
              for point in points:
                cur.execute('SELECT pos_x,pos_y,pos_z,ori_p,ori_r,ori_y FROM point_info where point_no = %s', (point,))
                results = cur.fetchall()
                for row in results:
                  #print(row[0], row[1], row[2], row[3], row[4], row[5])
                  ret_pos.append(DestPoint(row[0], 
                                          row[1], 
                                          row[2], 
                                          row[3],
                                          row[4],
                                          row[5]))
  return ret_pos

# 巡回経路のポイント番号を取得
def get_adpatrol_pointno(robot_id):
  ret_no = []
  with get_connection() as conn:
      with conn.cursor() as cur:
          cur.execute('SELECT points FROM patrol_path_info WHERE robot_id = %s', (robot_id,))
          pathes = cur.fetchall()
          for path in pathes:
            for points in path:
              ret_no = points
              print(points)
  return ret_no

# 巡回開始位置の座標を取得
def get_adpatrol_startpoint(robot_id):
  ret_pos = []
  with get_connection() as conn:
    with conn.cursor() as cur:
      cur.execute('SELECT points FROM patrol_path_info where robot_id = %s', (robot_id,))
      results = cur.fetchall()
      for row in results:
        cur.execute('SELECT pos_x,pos_y,pos_z,ori_p,ori_r,ori_y FROM point_info where point_no = %s', (row[0][0],))
        results = cur.fetchall()
        for row in results:
          ret_pos.append(DestPoint(row[0], 
                                   row[1], 
                                   row[2], 
                                   row[3],
                                   row[4],
                                   row[5]))
          return ret_pos

# 引き継ぎ地点の座標を取得
def get_handover_points():
  ret_pos = []
  with get_connection() as conn:
      with conn.cursor() as cur:
          str = 'handover_point'
          cur.execute('SELECT pos_x,pos_y,pos_z,ori_p,ori_r,ori_y FROM point_info WHERE point_kind = %s', (str,))
          results = cur.fetchall()
          for row in results:
            print(row[0], row[1], row[2], row[3], row[4], row[5])
            ret_pos.append(DestPoint(row[0], 
                                    row[1], 
                                    row[2], 
                                    row[3],
                                    row[4],
                                    row[5]))
  return ret_pos

# 現在地から目的地までの経路の座標を取得
def get_root_path(robot_id, goal_point, current_point):
  ret_pos = []
  with get_connection() as conn:
      with conn.cursor() as cur:
        #ゴールポイントまでの経路の指定
        #cur.execute('SELECT points FROM guide_path_info where goal_point=' + goal_point + ' and current_point = ' + current_point + ';')
        cur.execute('SELECT points FROM guide_path_info \
          where robot_id=%s and goal_point=%s and current_point=%s', (robot_id,goal_point,current_point,))
        pathes = cur.fetchall()
        for path in pathes:
          for points in path:
            for point in points:
              cur.execute('SELECT pos_x,pos_y,pos_z,ori_p,ori_r,ori_y FROM point_info where point_no = %s', (point,))
              results = cur.fetchall()
              for row in results:
                print(row[0], row[1], row[2], row[3], row[4], row[5])
                ret_pos.append(DestPoint(row[0], 
                                        row[1], 
                                        row[2], 
                                        row[3],
                                        row[4],
                                        row[5]))
  return ret_pos

# 現在地から目的地までの経路のポイント番号を取得
def get_root_pointno(robot_id, goal_point, current_point):
  ret_no = []
  with get_connection() as conn:
      with conn.cursor() as cur:
        #ゴールポイントまでの経路の指定
        #cur.execute('SELECT points FROM guide_path_info where goal_point=' + goal_point + ' and current_point = ' + current_point + ';')
        cur.execute('SELECT points FROM guide_path_info \
          where robot_id=%s and goal_point=%s and current_point=%s', (robot_id,goal_point,current_point,))
        pathes = cur.fetchall()
        for path in pathes:
          for points in path:
            ret_no = points
            print(points)
  return ret_no

# 「引継ぎ地点」の座標を取得
def get_handover_points_coordinate():
  ret_pos = []
  with get_connection() as conn:
      with conn.cursor() as cur:
          str = 'handover_point'
          cur.execute('SELECT pos_x,pos_y,pos_z,ori_p,ori_r,ori_y FROM point_info WHERE point_kind = %s', (str,))
          results = cur.fetchall()
          for row in results:
            print(row[0], row[1], row[2], row[3], row[4], row[5])
            ret_pos.append(DestPoint(row[0], 
                                    row[1], 
                                    row[2], 
                                    row[3],
                                    row[4],
                                    row[5]))
  return ret_pos




