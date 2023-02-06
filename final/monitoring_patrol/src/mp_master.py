#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ����@�@�F ���E�����͕����P�Q�N�̖�T�T���l���畽���P�V�N�̖�P�P�Q���l�Ɩ�Q�{�ɂȂ��Ă�
# �z�菇���F ���{�݁E�a�@ > ����ҏZ�܂��̎���i���{�b�g�̓����R�X�g�I�ɂ��̊��������H�j
# �@�@�@�@�@ �S���̈�Î{�ݐ��F179,090
# �@�@�@�@�@ �S���̉��{�ݐ��F179,090

#import time, datetime
import sys, roslib
import rospy
from std_msgs.msg import String, Float64
#Action
from smach_ros import ActionServerWrapper
# MP features
from mp_master.action import MpAction
from send_gmail.srv import SendGmail*
from aed_location_server.srv import AedLocationInfo*
#from monitoring_patrol.srv import LyingHumanHeight*, AedLocationInfo*, BioDetection*, Biometric*
from monitoring_patrol.action import BodyMotionDetection, PatrolRoom
#-----------------------------
# recognition ---
from happymimi_recognition_msgs.srv import RecognitionFind, RecognitionFindRequest
from happymimi_recognition_msgs.srv import RecognitionLocalize, RecognitionLocalizeRequest
# voice/sound ---
from playsound import playsound
from happymimi_voice_msgs.srv import *
from happymimi_msgs.srv import StrTrg
tts_sc = rospy.ServiceProxy('/tts', StrTrg) #!! ( ,TTS)
happymimi_voice_path = roslib.packages.get_pkg_dir("happymimi_voice")+"/../config/wave_data/aram.wav"
sec_happymimi_voice_path = roslib.packages.get_pkg_dir("happymimi_voice")+"/../config/wave_data/ga9du-ecghy2.wav"
# MC ------------
from geometry_msgs.msg import Twist
from happymimi_navigation.srv import NaviLocation
navi_sc = rospy.ServiceProxy('/navi_location_server', NaviLocation)
from real_time_navi.srv import RealTimeNavi

HT_PATH = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, HT_PATH)
from base_control import BaseControl
bc = BaseControl()


# �X�^�[�g
# Ver1�F ��ʒu����X�^�[�g
# Ver2�F �����̊O���������肵�āA���m�̈ʒu��Patrol�ɑJ��
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_finish'])

    def execute(self, userdata):
        rospy.loginfo("Executing state: Start")
        tts_sc("Start monitering patrol")
        # ����ʒu�Ɉړ�
        navi_sc('first_search_point')
        return 'start_finish'

# Ver1�F WayPoint�ɉ����Ĉړ����A�l��T��
#       �ړ����ɕ��񏈗���PersonSerch������
#        i_tra: Start     | o_tra: Charm, Observe,
#        i_key: human_num |
# Ver2�F �����̃R�X�g�}�b�v���J�����̍��E��p���őS�Ė��߂�悤��Patrol
# �@�@�@ �����Ă�l����������->�@Charm�@�i!!!or DecideAction
#       �|��Ă���@�@�@�@�@�@->�@Observe
# Ver3�F �l���m�ɋ����E�T�C�Y������݂��āA���炩�ɈႤ���̂𑫐؂肷��
# Ver4�F ���L�ɂ��Ή�
class PatrolRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['lying','standing',
                                            'not_found_one','not_found_two'])
        self.find_sc = rospy.ServiceProxy('/recognition/find', RecognitionFind)
    def execute(self, userdata):
        print("Executing state : PatrolRoom")
        # !! 3WayPoint�ړ���Search
        while self.find_sc(RecognitionFindRequest(target_name='person')).result == False:
        
        if 
            # �l�̎p�����m
            if getPosture().result == True:
                tts_srv("Hi!")
                return "to_charm"
        
# Ver1�F�|��Ă���l�̈ӎ��̗L��
#       �ӎ�������i�����x�����x��1,2�j-> ObeyOrder(), in_order
#       �Ȃ��@�@�@-> remey
# Ver2�F�l�������Ă��邩�����mmp_mojule.biometricCheck()
# Ver3�F�����Ȃ��ꍇ�A���̑�������āA�������𔻒f
class Observe(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes= [conscious, unconscious])
        
    
        
    def execute(self, userdata):
        rospy.loginfo("Executing state : Observe")
        

        # speak "Hello"
        # wait motion reaction(3s)
        #self.motion_sc(uint8 wait_time)
        # (*2 tiomes)
        # if detect any body motion, return

# Call(), SendMail(), AedMail(), AedInfo()
# rt_navi_sc
class Rescue(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['to_call','to_exit'])

    def execute(self, userdata):
        print("Executing state : TalkAndAlert")



class Call(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['call_finish'])

    def execute(self, userdata):
        print("Executing state : Call")



# i_tra: peace
# o_tra: charm_done
# �����g Hi5
class Charm(smach.State):


# i_tra: no_human, 
#
# i_key: human_num
class Back(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['all_finish'])
        
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
    def execute(self, userdata):
        print("Start going to operator")

def mpSmach():
    top = smach.StateMachine(outcomes = ["mp_succeeded",
                                         "mp_aborted",
                                         "mp_preempted"])
    top.userdata.human_num = 0
    top.userdata.warn_level = 0
    top.userdata.action_order = ""
    with top:
        smach.StateMachine.add('Start', Start(),
                                transitions = {'start_finish':'PatrolRoom'})
        smach.StateMachine.add('PatrolRoom', PatrolRoom(),
                                transitions = {'found_lying':'TalkAndAlert',
                                            'found_standing':'Back',
                                            'not_found_one':'PatrolRoom',
                                            'not_found_two':'Back'})
        smach.StateMachine.add('Observe', Observe(),
                                transitions = {'to_call':'Call',
                                            'to_exit':'Back'})
        smach.StateMachine.add('Call', Call(),
                                transitions = {'call_finish':'Back'})
        smach.StateMachine.add('Back', Back(),
                                transitions = {'all_finish':'finish_sm_top'})
        
    #outcome = top.execute()
  
def createNode(node_name):
    rospy.init_node(node_name)
    rospy.loginfo('Ready to start **%s**', node_name)
  
# accoding to original start timer setting
#def actualMain():
#  createNode()
  
# �f���Ńp�g���[���������Ɏ��s
# 
def demoMain():
    createNode("mp_demo")
    mp_sm = mpSmach()
    # �f�� Smach���s
    mp_sm_outcome = mp_sm.top.execute()
    
    rospy.spin()

# ��ʒu�ɂ��ăp�g���[�������s�imp_mode�F�O���ň��Ԋu�̎��ԂɊ�Â������s�j
# !!�@�����ɓ����Ă��Ȃ���Ԃ���X�^�[�g�F Navi�{EnterRoom �� patrolRoom
# goal-->     �m�F����Z�l�̐ݒ萔�A
# Response--> �����A���~�A�����
# Feedback--> Smach�X�e�[�g
# !!! Aborted�ɂȂ�󋵂𒲂ׂ�
def runMpAcserver():
    createNode("mp_acserver")
    mp_sm = mpSmach()
    ASW = ActionServerWrapper('mp_master_acserver', MpAction,
                              wrapped_container  = mp_sm.top,
                              succeeded_outcomes = ['mp_succeeded'],
                              aborted_outcomes   = ['mp_aborted'],
                              preempted_outcomes = ['mp_preempted'],
                              goal_key = 'goal_msg',
                              result_key = 'result_msg')
  # Run the server in a background thread
    ASW.run_server()
    rospy.spin()

def mpAcclientTest():
  
if __name__ == '__main__':
    pass