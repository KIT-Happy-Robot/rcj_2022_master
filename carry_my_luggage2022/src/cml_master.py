#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#----------------------------------------------------
# Title: RCAP2021 Carry My Luggage
# Author: Yusuke Kanazawa
# Date: 2021/11/26
# Memo: 
#----------------------------------------------------
import rospy
import roslib
import time
import sys
import smach
import smach_ros
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from happymimi_msgs.srv import StrTrg

from find_bag.srv import FindBagSrv, GraspBagSrv
from happymimi_voice_msgs.srv import YesNo
from happymimi_navigation.srv import NaviLocation
from happymimi_msgs.srv import StrTrg
base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl

# tts_srv
#tts_srv = rospy.ServiceProxy('/tts', StrTrg)
tts_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)

class FindBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_success',
                                               'find_failure'])
        # Service
        self.grasp_bag = rospy.ServiceProxy('/grasp_bag_server', GraspBagSrv)
        #self.find_bag = rospy.ServiceProxy('/find_bag_server', FindBagSrv)
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/left_right_recognition', String, self.poseCB)
        # Module
        self.base_control = BaseControl()
        # Value
        self.pose_msg = 'NULL'

    def poseCB(self, receive_msg):
        self.pose_msg = receive_msg.data

    def poseDataCheck(self):
        while not self.pose_msg and not rospy.is_shutdown():
            rospy.loginfo('No pose data available ...')
            rospy.sleep(0.5)

    def execute(self, userdata):
        tts_srv('/cml/which_bag')
        self.head_pub.publish(-2.0)
        self.poseDataCheck()
        rospy.sleep(1.5)
        while not rospy.is_shutdown():
            if self.pose_msg == 'left':
                tts_srv('/cml/bag_left')
                tts_srv('/cml/_bag')
                self.grasp_bag('right', [0.25, 0.4])
                self.base_control.rotateAngle(10, 0.5)
                break
            elif self.pose_msg == 'right':
                tts_srv('/cml/bag_right')
                tts_srv('/cml/_bag')
                self.grasp_bag('left', [0.25, 0.4])
                self.base_control.rotateAngle(-10, 0.5)
                break
            else:
                pass
        return 'find_success'


class Chaser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['chaser_finish'])
        # Publisher
        self.chaser_pub = rospy.Publisher('/follow_human', String, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/find_str', String, self.findCB)
        rospy.Subscriber('/cmd_vel', Twist, self.cmdCB)
        # ServiceProxy
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Module
        self.base_control = BaseControl()
        # Value
        self.start_time = time.time()
        self.find_msg = 'NULL'
        self.cmd_sub = 0.0

    def findCB(self, receive_msg):
        self.find_msg = receive_msg.data

    def cmdCB(self, receive_msg):
        self.cmd_sub = receive_msg.linear.x

    def execute(self, userdata):
        rospy.loginfo('Executing state: CHASER')
        tts_srv("/cml/follow_you")
        self.chaser_pub.publish('start')
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            now_time = time.time() - self.start_time
            if self.cmd_sub == 0.0 and self.find_msg == 'NULL':
                self.find_msg = 'lost_stop'
                self.start_time = time.time()
            elif self.cmd_sub == 0.0 and now_time >= 5.0 and self.find_msg == 'lost_stop':
                tts_srv("/cml/car_question")
                answer = self.yesno_srv().result
                if answer:
                    self.chaser_pub.publish('stop')
                    self.base_control.rotateAngle(0, 0)
                    return 'chaser_finish'
                else:
                    tts_srv("cml/follow_cont")
            elif self.find_msg == "lost":
                self.start_time = time.time()
                self.find_msg = 'lost_after'
            elif self.find_msg == "lost_after" and now_time >= 1.0:
                tts_srv("/cml/follow_lost")
                self.find_msg = "lost_long"
            elif self.find_msg == "lost_long" and now_time >= 11.0:
                tts_srv("/cml/car_question")
                answer = self.yesno_srv().result
                if answer:
                    self.chaser_pub.publish('stop')
                    self.base_control.rotateAngle(0, 0)
                    return 'chaser_finish'
                else:
                    tts_srv("/cml/follow_cont")
                    self.find_msg = "NULL"

            elif self.cmd_sub != 0.0:
                self.find_msg = 'NULL'
            else:
                pass


class PassBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['PASS_finish',
                                               'PASS_failure'])
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # ServiceProxy
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)
        # Module
        self.base_control = BaseControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: BagPass')
        self.base_control.translateDist(-0.3)
        tts_srv('/cml/give_bag')
        self.arm_srv('give')
        self.head_pub.publish(0)
        tts_srv('/cml/return_start')
        return 'PASS_finish'


class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['return_finish'])
        # Service
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.base_control = BaseControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: RETURN')
        rospy.sleep(0.5)
        self.base_control.rotateAngle(170, 0.3)
        rospy.sleep(0.5)
        self.navi_srv('cml')
        rospy.sleep(0.5)
        tts_srv("/cml/finish_cml")
        return 'return_finish'


if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start Carry My Luggage")
    tts_srv("/cml/start_cml")
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'FIND_BAG',
                FindBag(),
                transitions = {'find_success':'CHASER',
                               'find_failure':'CHASER'})

        smach.StateMachine.add(
                'CHASER',
                Chaser(),
                transitions = {'chaser_finish':'PASS_BAG'})

        smach.StateMachine.add(
                'PASS_BAG',
                PassBag(),
                transitions = {'PASS_finish':'RETURN',
                               'PASS_failure':'PASS_BAG'})

        smach.StateMachine.add(
                'RETURN',
                Return(),
                transitions = {'return_finish':'finish_sm_top'})

    outcome = sm_top.execute()
