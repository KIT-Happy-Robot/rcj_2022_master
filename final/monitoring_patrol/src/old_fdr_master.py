#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
# Desc: 
# Author: Hiroto Washio
# Date: 17/02/2022
#-------------------------------------------------------------------
import time, datetime
import sys
import rospy
from std_msgs.msg import String, Float64
import smach
import smach_ros
import roslib
from happymimi_msgs.srv import StrTrg
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
#from common_action_client import *
#from common_function import *
#from mimi_common_pkg.srv import ManipulateSrv, RecognizeCount
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from happymimi_voice_msgs.srv import *
from geometry_msgs.msg import Twist
from happymimi_navigation.srv import NaviLocation
from happymimi_recognition_msgs.srv import RecognitionFind, RecognitionFindRequest, RecognitionLocalizeRequest
base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl
reco_path = roslib.packages.get_pkg_dir('recognition_processing') + '/src/'
sys.path.insert(0, reco_path)
from recognition_tools import RecognitionTools
import roslib.packages
happymimi_voice_path = roslib.packages.get_pkg_dir("happymimi_voice")+"/../config/wave_data/aram.wav"
sec_happymimi_voice_path = roslib.packages.get_pkg_dir("happymimi_voice")+"/../config/wave_data/ga9du-ecghy2.wav"
from real_time_navi.srv import RealTimeNavi
from playsound import playsound
from send_gmail.srv import SendGmail
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
rt = RecognitionTools()
#import pyaudio
#now_dt = datetime.datetime.today()

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_finish'])
        self.bc = BaseControl()
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)

    def execute(self, userdata):
        rospy.loginfo("Executing state: Start")
        tts_srv("It's time to confirm")
        self.navi_srv('first_search_point')
        return 'start_finish'

class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found_lying','found_standing',
                                            'not_found_one','not_found_two'])
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        #self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        self.real_time_navi = rospy.ServiceProxy('/realtime_navi_server', RealTimeNavi)
        self.bc = BaseControl()
        #self.rt = RecognitionTools()
        navi_counter = 0
        found_place = 'NULL'
 
    def execute(self, userdata):
        print("Executing state : SearchPerson")
        navi_counter =+ 1
        self.head_pub.publish(25)
        rospy.sleep(1.5)
        self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
        rospy.sleep(1.0)
        #self.head_pub.publish(0)
        if self.find_result == True:
            print("found a person")
            request = RecognitionLocalizeRequest()
            request.target_name = "person"
            centroid = rt.localizeObject(request).point
            person_height = centroid.z
            print person_height
            standard_z = 0.4
            if person_height >= standard_z:
                self.head_pub.publish(0)
                tts_srv("Hi!")
                target_name = 'standing_person'
                print('found a standing person')
                #now_dt = datetime.datetime.today()
                #now_time = now_dt.time()
                #print now_dt
                #print now_time
                tts_srv("What's up. Howdy?")
                return 'found_standing'
            else:
                tts_srv("Hi!,,,Oh! you lying down!")
                target_name = 'lying_person'
                self.real_time_navi('set')
                return 'found_lying'
            # 人が居ない場合
        elif self.find_result == False:
            print("found a person.")
            tts_srv("found no person.")
            if navi_counter == 1:
                tts_srv("moving to another point")
                self.navi_srv('sec_search_point')
                rospy.sleep(0.5)
                rospy.loginfo('finish moving to another place')
                return 'not_found_one'
            elif navi_counter > 1:
                print"found no person again"
                tts_srv("Found no person again. He is out of this house now")
                return 'not_found_two'

class TalkAndAlert(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['to_call','to_exit'])
        #self.yes_no_srv = rospy.ServiceProxy('/yes_no', YesNo)
        #self.rt = RecognitionTools()
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.mail_srv = rospy.ServiceProxy('/send_gmail_server', SendGmail)
    def execute(self, userdata):
        print("Executing state : TalkAndAlert")
        for i in range(3):
            tts_srv("Are you sleeping")
            #yes_no_result = self.yes_no_srv().result#####
            self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
            # 人が居なかった場合
            if self.find_result == False:
                print 'The person is away here'
                tts_srv("You are out of my eyes. You probably woke up")
                return 'to_exit'
            #　人を見つけた場合
            elif self.find_result == True:
                pass
            request = RecognitionLocalizeRequest()
            request.target_name = "person"
            centroid = rt.localizeObject(request).point
            person_height = centroid.z
            print person_height
            standard_z = 0.4
            #if person_height > standard_z or yes_no_result == True or yes_no_result == False:  #　voice
            #　見つけた人が立っていた場合
            if person_height > standard_z :
                self.head_pub.publish(0)
                print("Confirm that you are awake")
                #tts_srv("How are you?")
                #tts_srv("I'm sorry to prevent you from sleeping though, I think it's better to sleep on your facking bed")
                return 'to_exit'
                break
            else:
                pass
        #　立たない、応答がない場合
        else:
            for i in range(3):
                playsound(happymimi_voice_path)
                print 'send mail for help'
                self.mail_srv('kit.robocup.home@gmail.com',
                              'gewretvedgpzlobj',
                             ['c1100781@planet.kanazawa-it.ac.jp',
                              'c1115332@planet.kanazawa-it.ac.jp'],
                              '【KitHappyRobot】人名に関するお知らせ',
                              '家で人が意識不明の状態で倒れています。' + '至急、救急車を呼んでください。',
                             )
                tts_srv("A person is lying down and lose conciousness")
                request = RecognitionLocalizeRequest()
                request.target_name = "person"
                centroid = rt.localizeObject(request).point
                person_height = centroid.z
                print person_height
                standard_z = 0.4
                
                if person_height > standard_z:
                    self.head_pub.publish(0)
                    print("Confirm that a person stand")
                    tts_srv("Waht's up. Are you OK?")
                    tts_srv("I'm sorry to prevent you from sleeping though, but I think it's better to sleep in your bed")
                    return 'to_exit'
                    break
                else:
                    pass
            else:
                return 'to_call'

class Call(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['call_finish'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.bc = BaseControl()
    def execute(self, userdata):
        print("Executing state : Call")
        tts_srv("I'll call anyone for help")
        self.navi_srv('call_point')
        for i in range(2):
            playsound(happymimi_voice_path)
            #tts_srv("Help! A person was found unconscious in this house. Someone please call an ambulance")
            self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
            if self.find_result == True:
                #tts_srv("Find someone...Help! A person was found unconscious in this house. Please follow me.")
                self.real_time_navi('navigation')
                self.head_pub.publish(20)
                #tts_srv("I guided you to the unconscious person")
                return 'call_finish'
                break
            elif self.find_result == False:
                if i == 0:
                    self.bc.rotateAngle(90, 0.3)
                    rospy.sleep(2.0)
                elif i == 1:
                    self.bc.rotateAngle(-180,0.5)
                elif i == 2:
                    self.bc.rotateAngle(90,0.4)
                else:
                    break
        return 'call_finish'

class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['all_finish'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
    def execute(self, userdata):
        print("Start going to operator")
        tts_srv("I'll be back")
        start_time = time.time()
        stop_time = 7
        while (time.time() - start_time) <= stop_time:
            playsound(sec_happymimi_voice_path)

        self.navi_srv("start_point")
        print("finish confirming of human life safety")
        tts_srv("finish confirming")
        return 'all_finish'

if __name__ == '__main__':
    rospy.init_node('falling_down_resc')
    rospy.loginfo('Start informing people lying down')
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'Start',
                Start(),
                transitions = {'start_finish':'SearchPerson'})
        smach.StateMachine.add(
                'SearchPerson',
                SearchPerson(),
                transitions = {'found_lying':'TalkAndAlert',
                               'found_standing':'Exit',
                               'not_found_one':'SearchPerson',
                               'not_found_two':'Exit'})
        smach.StateMachine.add(
                'TalkAndAlert',
                TalkAndAlert(),
                transitions = {'to_call':'Call',
                               'to_exit':'Exit'})
        smach.StateMachine.add(
                'Call',
                Call(),
                transitions = {'call_finish':'Exit'})
        smach.StateMachine.add(
                'Exit',
                Exit(),
                transitions = {'all_finish':'finish_sm_top'})
        
    outcome = sm_top.execute()
