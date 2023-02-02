#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import sys
import roslib
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.srv import ArmControl

teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl


#eef = rospy.Publisher('/servo/endeffector', Bool, queue_size=10)
#arm = rospy.ServiceProxy('/servo/arm', StrTrg)
#arm_debug = rospy.ServiceProxy('/servo/debug_arm', ArmControl)

class FindBag():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        self.laser_list = []  # LRFのデータが入るリスト
        self.center_range = []  # 中心から拡張するためのリスト
        self.bag_range = []  # バッグの範囲を格納するリスト
        self.center_index = 0.0  # 真ん中の要素番号を格納する変数
        self.index_sum = 0.0  # LRFの要素数の合計

    def laserCB(self, receive_msg):
        self.laser_list = list(receive_msg.ranges)

    def roundHalfUp(self, value):
        decimals = math.modf(value)[0]
        return int(value + 1) if decimals >= 0.5 else int(value)

    def laserIndex(self):
        self.index_sum = len(self.laser_list)
        self.center_index = self.roundHalfUp(self.index_sum/2 - 1)

    def average(self, input_list):
        ave = sum(input_list)/len(input_list)
        return ave

    def centerSpread(self, spread_value, left_right):
        self.center_range = self.laser_list
        if left_right == 'left':
            del self.center_range[0 : self.center_index]  # 右半分削除
            del self.center_range[-self.center_index-1+spread_value : ]  # リストの先頭から1つずつ追加
        elif left_right == 'right':
            del self.center_range[self.center_index+1 : self.index_sum]  # 左半分削除
            del self.center_range[ : -1-spread_value]  # リストの末尾から1つずつ追加
            self.center_range = list(reversed(self.center_range))
        else:
            rospy.loginfo('Not left_right value')

    def centerFind(self, left_right):
        bag_dist = []
        bag_average = 'NULL'
        self.laserIndex()
        count = 0
        while not rospy.is_shutdown():
            rospy.sleep(0.05)
            self.centerSpread(count, left_right)
            laser_average = self.average(self.center_range)
            if laser_average - 0.3 > self.center_range[-1]:
                self.bag_range.append(count)
                bag_dist.append(self.center_range[-1])
                bag_average = self.average(bag_dist)
            elif bag_average != 'NULL' and bag_average + 0.3 < self.center_range[-1]:
                print('=========================\nbreak')
                break
            else:
                pass
            print(self.bag_range)
            count += 1
        center_angle = self.bag_range[self.roundHalfUp(len(self.bag_range)/2)]
        return center_angle*0.18 if left_right == 'left' else -1*center_angle*0.18


if __name__ == '__main__':
    rospy.init_node('find_bag_node')
    fb = FindBag()
    bc = BaseControl()
    rospy.sleep(0.5)

    #arm('carry')
    #eef.publish(False)

    move_angle = fb.centerFind('right')
    rospy.sleep(0.1)
    print(f'Move angle = {move_angle}')
    bc.rotateAngle(move_angle, 0.2)
    #rospy.sleep(1.0)
    #arm_debug([0.4, 0.55])
    #rospy.sleep(0.5)
    #bc.translateDist(st.laser_list[st.center_element]-0.1)
    #rospy.sleep(0.5)
    #eef.publish(True)
    #rospy.sleep(0.5)
    #arm('carry')
    print('finish')
