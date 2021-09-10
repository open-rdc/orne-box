#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
import tf
from geometry_msgs.msg import Point
from rospy.exceptions import ROSException
from rospy.topics import Publisher
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math

import copy
import numpy as np



class MirrorLiDAR():

    def __init__(self):

        try:
            self.mirror_d = rospy.get_param("/mirror_lidar/mirror_d", 0.1)
            self.scan_front_begin = rospy.get_param("/mirror_lidar/scan_front_begin", -math.pi/3)
            self.scan_front_end   = rospy.get_param("/mirror_lidar/scan_front_end"  , math.pi/3)
            self.scan_left_begin  = rospy.get_param("/mirror_lidar/scan_left_begin" , math.pi/3)
            self.scan_left_end    = rospy.get_param("/mirror_lidar/scan_left_end"   , math.pi*2/3)
            self.scan_right_begin = rospy.get_param("/mirror_lidar/scan_right_begin", -math.pi*2/3)
            self.scan_right_end   = rospy.get_param("/mirror_lidar/scan_right_end"  , -math.pi/3)
        except ROSException:
            rospy.loginfo("param load error")

        self.sub_scan = rospy.Subscriber('/tilted_scan', LaserScan, self.callback)
        self.pub_scan_front = rospy.Publisher('scan_front', LaserScan, queue_size=1)
        self.pub_scan_right = rospy.Publisher('scan_right', LaserScan, queue_size=1)
        self.pub_scan_left  = rospy.Publisher('scan_left' , LaserScan, queue_size=1)
        self.pub_fit_r      = rospy.Publisher('fit_line_R', Marker, queue_size=1)
        self.pub_fit_l      = rospy.Publisher('fit_line_L', Marker, queue_size=1)





#   callback
#   scanトピックの更新時に呼ばれるコールバック関数
#   引数：callback(LaserScan data)
#       data    LiDARのスキャン生データ
#

    def callback(self, data):

        # スキャンデータを指定した範囲でトリム
        front_data = self.divide_scan_data(data, -1, 1)
        right_data = self.divide_scan_data(data, 1.6, 2.09)
        left_data = self.divide_scan_data(data, -2.05, -1.5)

        # 左右の鏡で反射したスキャンデータの近似直線を計算
        func_R, x_R = self.calc_fitting_curve(right_data)
        func_L, x_L = self.calc_fitting_curve(left_data)

        # それぞれの近似直線の方程式から２点(Xの最小と最大についてのY)のXY座標を計算
        #line_pos_R = self.calc_function(func_R, x_R[0], x_R[1])
        #fit_r = self.calc_marker(line_pos_R, data.header.stamp)
        #line_pos_L = self.calc_function(func_L, x_L[0], x_L[1])
        #fit_l = self.calc_marker(line_pos_L, data.header.stamp)
        line_pos_R = [self.calc_function(func_R, x_R[0]), self.calc_function(func_R, x_R[1])]
        line_pos_L = [self.calc_function(func_L, x_L[0]), self.calc_function(func_L, x_L[1])]

        # 鏡の点群の近似直線を３次元へ変換
        bottom_r = self.convert_3d(func_R, x_R[0], x_R[1], 1)
        bottom_l = self.convert_3d(func_L, x_L[0], x_L[1], -1)

        # 計測平面の近似直線を表示するためのマーカーデータの作成
        fit_r = self.calc_marker(bottom_r[0], data.header.stamp)
        fit_l = self.calc_marker(bottom_l[0], data.header.stamp)

        # 計測平面からセンサのピッチとロールの傾きを表示
        pose = self.calc_pose(bottom_r[1], bottom_l[1])
        #print(A)

        self.broadcast_pose(pose[0], pose[1], pose[2])
        # パブリッシュ
        self.pub_scan_front.publish(front_data)
        self.pub_scan_right.publish(right_data)
        self.pub_scan_left.publish(left_data)
        self.pub_fit_r.publish(fit_r)
        self.pub_fit_l.publish(fit_l)





#   divide_scan_data
#   スキャンデータから取得角度に基づいてターゲット範囲のスキャンデータを取り出す
#   引数：divide_scan_data(LaserScan data, float begin, float end)
#       data    LiDARのスキャン生データ
#       begin   ターゲットの開始角度[rad]
#       end     ターゲットの終了角度[rad]
#   返り値: LaserScan input_data
#

    def divide_scan_data(self, data, begin, end):
        input_data = copy.deepcopy(data)
        angle_rates = np.arange(input_data.angle_min, input_data.angle_max, input_data.angle_increment)
        target_index = np.where((angle_rates >= begin) & (angle_rates <= end))
        target_index = target_index[0]
        target_ranges = input_data.ranges[target_index[0]:target_index[-1]]
        input_data.ranges = target_ranges
        input_data.angle_min = begin
        input_data.angle_max = end
        return input_data





#   getXY
#   極座標から直行座標へ変換
#   引数：getXY(any r, any rad)
#       r       動径
#       rad     極角(radian)
#   返り値: float x, float y
#       x   x座標
#       y   y座標
#

    def getXY(self, r, rad):
        x = r * np.cos(rad)
        y = r * np.sin(rad)
        return x, y




#   calc_fitting_curve
#   与えられたスキャンデータを一次最小二乗法でフィッテング
#   引数：calc_fitting_curve(LaserScan data)
#       data    フィッティングしたい点群を含むスキャンデータ
#   返り値:list func[float slope,float intercept], list pos[float x_begin, float x_end]
#       func[slope, intercept]      フィッティングした関数の傾きslopeと切片interceptをリストで返す
#       pos[x_begin, x_end]         スキャンデータの直交座標系でのX座標の始点と終点をリストで返す
#
    def calc_fitting_curve(self, data):
        angle = data.angle_min
        x = []
        y = []
        for range in data.ranges:
            angle = angle + data.angle_increment
            position = self.getXY(range, angle)

            if not np.isnan(position[0]):
                x.append(position[0])
                y.append(position[1])
            else:
                print("range value is NaN")

        fit_line = np.polyfit(np.array(x), np.array(y), 1)
        func = list(fit_line)
        pos = [x[0], x[-1]]

        return func, pos



#   calc_function
#   一次関数の傾きと切片、x座標からy座標を求める
#   引数：calc_function(list func[slope, intercpt], list x[x_begin, x_end])
#       data    フィッティングしたい点群を含むスキャンデータ
#   返り値:float y
#       y       y=ax+bのyの値
#

    def calc_function(self, func, x):
        a = func[0]
        b = func[1]

        y = a * x + b

        return y





#   calc_marker
#   直線のmarkerトピックのデータ作成
#   引数：calc_marker( list pos[x1,y1,z1,x2,y2,z2], std_msgs/Header time)
#       pos[x1,y1,z1,x2,y2,z2]      表示する直線の始点のxyz座標,終点のxyz座標
#       time                        トピックのheadertime
#   返り値:std_msgs/Header marker_data
#         marker_data       引数で与えられた点間を結ぶ直線をrvizで表示できる形式にしたmarkerデータ


    def calc_marker(self, pos, headertime):
        marker_data = Marker()
        marker_data.header.frame_id = "laser"
        marker_data.ns = "soiya"
        marker_data.id = 0
        marker_data.header.stamp = headertime
        marker_data.action = Marker.ADD

        marker_data.pose.position.x = 0.0
        marker_data.pose.position.y = 0.0
        marker_data.pose.position.z = 0.0

        # クォータニオン
        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 0.0
        marker_data.pose.orientation.w = 1.0

        # 色設定
        marker_data.color.r = 0.0
        marker_data.color.g = 1.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        # 直線のサイズ設定
        marker_data.scale.x = 0.005
        marker_data.scale.y = 0.005
        marker_data.scale.z = 0.005

        marker_data.lifetime = rospy.Duration()
        marker_data.type = Marker.LINE_STRIP

        marker_data.points = []

        # 始点の座標
        first_point = Point()
        first_point.x = pos[0]
        first_point.y = pos[1]
        first_point.z = pos[2]
        marker_data.points.append(first_point)

        # 終点の座標
        second_point = Point()
        second_point.x = pos[3]
        second_point.y = pos[4]
        second_point.z = pos[5]
        marker_data.points.append(second_point)
        return marker_data






#   convert_3d
#   鏡で反射された部分のデータを三次元へ変換する(鏡の位置を境にZ方向へ折り返す)
#   引数：convert_3d(list func[float slope, float intercept], pos1, pos2, dir)
#           func[float slope, float intercept]  1次関数の傾きslope, 切片intercept
#           pos1                                始点x座標
#           pos2                                終点x座標
#           dir                                 スキャンデータをどちらに折り返すかのフラグ left側scanなら0,rightなら1
#
#   返り値: position[x1,y1,z1,x2,y2,z2], func_3d[a,b]
#         position      marker_data用の直線の始点と終点の座標
#         func_3d       ZX平面上での1次関数
#               a 傾き
#               b 切片
#

    def convert_3d(self, func, pos1, pos2, dir):
        offset = self.mirror_d * dir

        # 傾きを左右のデータごとに最終的にZ軸下向きになるようにする
        a = func[0] * dir

        # 切片から鏡までの距離を引く
        b = (func[1] - offset) * dir

        # 鏡まではXY平面、鏡より先はZX平面に変換
        # ZX平面は原点からY方向に鏡までの距離をオフセットした位置での平面になる
        x1 = pos1
        y1 = offset
        z1 = a * pos1 + b
        x2 = pos2
        y2 = offset
        z2 = a * pos2 + b
        position = [x1, y1, z1, x2, y2, z2]
        func_3d = [a, b]

        return position, func_3d






#   calc_pose
#   慣性座標系(測定平面)から見たlidarの傾きを計算する
#   引数：calc_pose(list func_R, list func_L)
#           func_R [float slope_R, float intercept_R]  1次関数の傾きslope, 切片intercept
#           func_L [float slope_L, float intercept_L]  1次関数の傾きslope, 切片intercept
#
#   返り値: roll, pitch
#         roll      測定平面とセンサ座標系のX軸のなす角(pitch)
#         pitch      測定平面とセンサ座標系のY軸のなす角(Roll)
#           hight       測定座標系から見たurgの高さ

    def calc_pose(self, func_R, func_L):
        Ar = func_R[0]
        Br = func_R[1]
        Al = func_L[0]
        Bl = func_L[1]

        # pitch
        Ax = (Ar + Al)/2    #測定平面を決定するために２直線の傾きの平均を取る
        #測定平面の傾きAxから、測定平面とセンサ座標系XY平面のなす角(pitch)を求める
        ytan = Ax          
        pitch = np.arctan(ytan)  #逆三角関数で解く

        # roll
        # センサ座標系y軸とそれぞれの近似直線の切片の差を２枚の鏡の距離で割ってtanを出す
        xtan = (Bl - Br) / (2 * self.mirror_d)
        roll = np.arctan(xtan)

        # hight
        
        #切片の平均を取ってセンサ座標系Z軸の地面までの距離を出す
        Bc = (Bl+Br)/2

        hight = np.cos(roll) * np.cos(pitch) * Bc


        #s = "height = " + str(hight) + ", roll = " + str(roll) + ", pitch = " + str(pitch)
        #rospy.loginfo(s)
        print(str(hight) + ", " + str(roll) + ", " + str(pitch))


        return roll, pitch, hight





#   broadcast_pose
#   tfにlidarの姿勢をブロードキャストする
#   引数：broadcast_pose(self, roll, pitch, hight)
#
#   返り値: なし
#

    def broadcast_pose(self, roll, pitch, hight):

        br = tf.TransformBroadcaster()
        br.sendTransform((0,0,hight),
                        #lidarは上下逆についているので座標を変換
                        tf.transformations.quaternion_from_euler(-roll+np.pi,-pitch , 0),
                        rospy.Time.now(),
                        "measurement_plane",
                        "laser")
    


if __name__ == '__main__':
    rospy.init_node('fitting')
    node = MirrorLiDAR()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

