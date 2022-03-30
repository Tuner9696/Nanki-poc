#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
from db_access_helper import DestPoint, get_point_coordinate, get_adpatrol_path, get_handover_points, get_point_no, get_root_pointno, get_adpatrol_pointno, get_handover_points_coordinate


# 引き継ぎ要否判定
def judge_handover(n, dest_pos):

  # ロボット001の現在地点（向かっていた巡回地点）を取得
  route = get_adpatrol_path('001')
  if n > len(route):
    now =route[0]
  else:
    now =route[n-1]

  # 目的地までの距離
  to_dest = get_distance(now.pos_x, now.pos_y, dest_pos.pos_x, dest_pos.pos_y)
  print('目的地までの距離')
  print(to_dest)

  # DBから引き継ぎ地点(複数)の座標を取得
  print('DBから引き継ぎ地点(複数)を取得')
  handover_points = get_handover_points()

  now_no = get_point_no(now)
  for handover_point in handover_points:

    # 引き継ぎ地点のポイント番号取得
    print('引き継ぎ地点のポイント番号取得')
    handover_no = get_point_no(handover_point)

    # 引き継ぎ地点までの経路のポイント番号を取得
    print('引き継ぎ地点までの経路のポイント番号を取得')
    points = get_root_pointno('001', handover_no, now_no)

    # 引き継ぎ地点までの距離計算
    print('引き継ぎ地点までの距離計算')  
    from_pos = now
    to_handover = 0
    for point in points:
      to_pos = get_point_coordinate(point)
      v = get_distance(from_pos.pos_x, from_pos.pos_y, to_pos.pos_x, to_pos.pos_y)
      to_handover = to_handover + v
      from_pos = to_pos

    print('引き継ぎ地点までの距離') 
    print(to_handover)

    if to_dest > to_handover:
      print('目的地は引き継ぎ地点より遠い')
      return True

  return False


# 二点間の距離を求める
def get_distance(x1, y1, x2, y2):
  d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
  return d


# 引き継ぎ地点を決定する
def select_handover_point(n, another_n):
  # ロボット001の現在地点（向かっていた巡回地点）を取得
  print('ロボット001の現在地点（向かっていた巡回地点）を取得')
  destpoints_all = get_adpatrol_pointno(int('001'))
  if n > len(destpoints_all):
    now = destpoints_all[0]
  else:
    now = destpoints_all[n-1]

  # ロボット002の現在地点（向かっていた巡回地点）を取得
  print('ロボット002の現在地点（向かっていた巡回地点）を取得')
  destpoints_all = get_adpatrol_pointno(int('002'))
  if n > len(destpoints_all):
    another_robot_now = destpoints_all[0]
  else:
    another_robot_now = destpoints_all[another_n-1]

  # DBから引き継ぎ地点(複数)を取得
  print('DBから引き継ぎ地点(複数)を取得')
  handover_points = get_handover_points()

  result = {}  # {引き継ぎ地点のポイント : 各ロボットの引き継ぎ地点までの距離の差}
  for handover_point in handover_points:

    # 引き継ぎ地点のポイント番号取得
    print('引き継ぎ地点のポイント番号取得')
    handover_no = get_point_no(handover_point)

    distance = []
    for i in range(2):
      if i==0:
        no = now
      else:
        no = another_robot_now
      distance.append(0)

      # 引き継ぎ地点までの経路のポイント番号を取得
      print('引き継ぎ地点までの経路のポイント番号を取得')
      str = format(i+1, '03d')
      points = get_root_pointno(str, handover_no, no)

      # 引き継ぎ地点までの距離計算
      #print('引き継ぎ地点までの距離計算')   
      from_pos = get_point_coordinate(no)
      for point in points:
        to_pos = get_point_coordinate(point)
        _v = distance[i]
        v = get_distance(from_pos.pos_x, from_pos.pos_y, to_pos.pos_x, to_pos.pos_y)
        distance[i] = _v + v
        from_pos = to_pos
        #print(distance[i])

      print('引き継ぎ地点までの距離合計')
      print(distance[i])

    # 各ロボットの引き継ぎ地点までの距離の差を計算
    v = abs(distance[0] - distance[1])
    result[handover_point] = v
    print('各ロボットの引き継ぎ地点までの距離の差を計算')
    print(v)

  # 各ロボットの引き継ぎ地点までの距離の差が最も小さいものを選択
  print('各ロボットの引き継ぎ地点までの距離の差が最も小さいものを選択')
  ret_pos = min(result, key=result.get)
  print(ret_pos)
  return ret_pos

# 引き継ぎ地点手前の到着地点を計算する
# （ロボットが衝突しないように引き継ぎ地点からずらした位置を目的地に設定する）
def get_handover_gap_point(id, hndvr_pos):

  ret_pos = None

  # ２つの引き継ぎ地点を取得
  poslist = get_handover_points_coordinate()

  if len(poslist) == 2:
    if poslist[0].pos_x == hndvr_pos.pos_x and poslist[0].pos_y == hndvr_pos.pos_y:
      no = 1
      if id == '001':
        # ロボット001の場合
        ret_pos = get_point_coordinate(90)
      else:
        # ロボット002の場合
        ret_pos = get_point_coordinate(91)
    else:
      if id == '001':
        # ロボット001の場合
        ret_pos = get_point_coordinate(92)
      else:
        # ロボット002の場合
        ret_pos = get_point_coordinate(93)

  return ret_pos



