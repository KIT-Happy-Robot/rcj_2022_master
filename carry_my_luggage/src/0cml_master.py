#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------
# Title: RCAP2021 Carry My Luggage
# Author: Yusuke Kanazawa
# Date: 2021/11/
# Memo:
#----------------------------------------------------
import rospy
import time
import smach
import smach_ros
from std_msgs.msg import String
from happymimi_robot.srv import StrTrg
from happymimi_voice_msgs.srv import YesNo
from happymimi_navigation.srv import NaviLocation
from happymimi_recognition_msgs.srv import RecognitionList, RecognitionLocalize

# tts_srv
tts_srv = rospy.ServiceProxy('/tts', StrTrg)

class FindBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_success'],
                                   input_keys = ['find_count_in'],
                                   output_keys = ['find_count_out'])
        rospy.Subscriber('left_right_recognition', String, self.poseCB)
        self.pose_msg = 'NULL'

    def poseCB(self, receive_msg):
        self.pose_msg = receive_msg.data

    def execute(self, userdata):
        rospy.sleep(0.5)
        find_count = userdata.find_count_in
        if self.pose_msg == '0:left':
            userdata.find_count_out = find_count
            return 'find_success'
        elif self.pose_msg == '0:right':
            find_count += 1
            userdata.find_count_out = find_count
            return 'find_success'
        elif self.pose_msg == '0:false:':
            find_count += 2
            userdata.find_count_out = find_count
            return 'find_failure'
        else:
            find_count += 3
            userdata.find_count_out = find_count


class GraspOrPass(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['GOP_finish'],
                                   input_keys = ['bag_count_in'])
        # ServiceProxy
        self.recog_srv = rospy.ServiceProxy('/recognition/list', RecognitionList)
        self.localize_srv = rospy.ServiceProxy('/recognition/localize', RecognitionLocalize)

    def execute(self, userdata):
        bag_list = self.recog_srv(target_name='peperbag', sort_option='left')
        if userdata.bag_count_in == 0:  # バッグが左にあるとき
            
        else if userdata.bag_count_in == 1:  # バッグが右にあるとき

        else if userdata.bag_count_in == 2:  # バッグがどっちにあるのかわからなかったとき

        else:  # バッグをパスするとき
        return 'GOP_finish'


class Chaser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['chaser_finish'])
        # Publisher
        self.chaser_pub = rospy.Publisher('/follow_human', String, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/find_str', String, self.findCB)
        # ServiceProxy
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Value
        self.find_msg = False
        self.start_time = time.time()

    def findCB(self, receive_msg):
        self.find_msg = receive_msg.data

    def execute(self, userdata):
        self.chaser_pub.publish('start')
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            now_time = time.time() - self.start_time
            if self.find_msg == "lost" and now_time >= 5:
                tts_srv("I lost sight of you")
                tts_srv("Is this the location of the car?")
                answer = self.yesno_srv().result
                if answer:
                    return 'chaesr_finish'
                else:
                    tts_srv("Ok, continue to follow")
                    tts_srv("Please come in front of me")
                    self.find_msg = "NULL"
            elif self.find_msg == "lost":
                self.start_time = time.time()
            else:
                pass


class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['return_finish'])
        # Service
        #self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)

    def execute(self, userdata):
        print 'Return'
        rospy.sleep(0.5)
        #self.navi_srv('operator')
        return 'return_finish'


if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start Carry My Luggage")
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.find_count = 0
    with sm_top:
        smach.StateMachine.add(
                'FIND_BAG',
                FindBag(),
                transitions = {'find_success':'GRASP_OR_PASS',
                               'find_failure':'GRASP_OR_PASS'},
                remapping = {'find_count_in':'find_count',
                             'find_count_out':'find_count',})

        smach.StateMachine.add(
                'GRASP_OR_PASS',
                GraspOrPass(),
                transitions = {'GRASP_finish':'CHASER',
                               'PASS_finish':'RETUN'},
                remapping = {'bag_count_in':'find_count'})

        smach.StateMachine.add(
                'CHASER',
                Chaser(),
                transitions = {'chaser_finish':'GRASP_OR_PASS'})

        smach.StateMachine.add(
                'RETURN',
                Return(),
                transitions = {'return_finish':'finish_sm_top'})

        outcome = sm_top.execute()
