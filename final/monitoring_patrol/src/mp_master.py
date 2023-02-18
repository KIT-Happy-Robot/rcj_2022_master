#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#import time, datetime
import rospy
import smach
from smach_ros import ActionServerWrapper
from std_msgs.msg import Bool, String, Float64
# Action
# MP features
#from mp_master.action import MpAction
import sys, roslib
mp_path = roslib.packages.get_pkg_dir("monitoring_patrol")
sys.path.imsert(0, mp_path)
from monitoring_patrol.srv import AdNaviSrv
from send_gmail.srv import SendGmail
from monitoring_patrol.srv import AedLocationInfo
from monitoring_patrol.srv import LyingHumanHeight #, BioDetection*, Biometric*
from monitoring_patrol.action import BodyMotionDetectAct #, PatrolRoom
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
from real_time_navi.srv import RealTimeNavi #!!

HT_PATH = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, HT_PATH)
from base_control import BaseControl
bc = BaseControl()


# スタート
# Ver1： 定位置へ着いてからスタート。　スタート位置に付けなければ失敗
# Ver2： 部屋の外からでも対応。未知の位置でPatrolに遷移
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["start_succeeded", "start_failed",
                                               "to_Back"],
                             input_keys = ["start_loop_lim_in"])
        self.navi_sc = rospy.ServiceProxy('/navi_location_server', NaviLocation)

    def execute(self, userdata):
        navi_count = 1
        start_loop_lim = userdata.start_loop_lim_in
        rospy.loginfo("MP: Executing state: Start")
        tts_sc("Start room patrol")
    # 部屋まで位置に移動
        # ナビを制限回数内で実行
        while not navi_count > start_loop_lim:
            navi_res = self.navi_sc("start_pos").result
            if navi_res: return "start_succeeded" #!!
            else: navi_count =+ 1
        return "start_failed"
    
        # navi_res = self.navi_sc("start_pos").result
        # if navi_res: return "start_succeeded"
        # # ナビが失敗したら、指定回数でやり直す
        # else:
        #     navi_count =+ 1
        #     # 指定回数でナビが失敗したらBackへ遷移する
        #     if navi_count > start_loop_lim: return "to_Back"
        #     else: return "start_failed"
                

# Ver1： WayPointに沿って移動し、ターゲットを探す
#       移動中に並列処理でPersonSerchしたい
#        i_tra: Start     | o_tra: Charm, Observe,
#        i_key: human_num |
# Ver2： 部屋のコストマップをカメラの左右画角内で全て埋めるようにPatrol
# 　　　 立ってる人を見つけたら->　Charm　（!!!or DecideAction
#       倒れている　　　　　　->　Observe
# Ver3： 人検知に距離・サイズ制限を設けて、明らかに違う物体を足切りする
# Ver4： 犬猫にも対応
class PatrolRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['lying','standing',
                                            'not_found_one','not_found_two'])
        self.find_sc = rospy.ServiceProxy('/recognition/find', RecognitionFind)
    #!! 首を下げて常に人を探す
    def execute(self, userdata):
        rospy.loginfo("MP: Executing state: Patrol room")
        self.search_sub = rospy.Subscriber('/mp/detect_target', #Bool,  ) #!!
        
        while not self.search_sub.data:
        
        tts_sc("Searching the target")
        # 首を下げて、探し回す
        # !! 3WayPoint移動＆Search
        while self.find_sc(RecognitionFindRequest(target_name='person')).result == False:
        
        if 
            # 人の姿勢検知
            if getPosture().result == True:
                tts_srv("Hi!")
                return "to_charm"
        
# Ver1：倒れている人の意識の有無
#       意識がある（音声警告レベル1,2）-> ObeyOrder(), in_order
#       ない　　　-> remey
# Ver2：人が動いているかを検知mp_mojule.biometricCheck()
# Ver3：動かない場合、生体測定をして、生物かを判断
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
# ご愛嬌 Hi5
class Charm(smach.State):
  pass

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
    top.userdata.start_loop_lim = 2
    top.userdata.human_num = 0
    top.userdata.warn_level = 0
    top.userdata.action_order = ""
    with top:
        smach.StateMachine.add('Start', Start(),
                                transitions = {'start_finish':'PatrolRoom',
                                               "start_failed":"Start",
                                               "to_Back":"Back"},
                                remapping = {"start_loop__lim_in":"start_loop_lim"})
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
  
# デモでパトロールをすぐに実行
# 
def demoMain():
    createNode("mp_demo")
    mp_sm = mpSmach()
    # デモ Smach実行
    mp_sm_outcome = mp_sm.top.execute()
    
    rospy.spin()

# 定位置についてパトロールを実行（mp_mode：外部で一定間隔の時間に基づいた実行）
# !!　部屋に入っていない状態からスタート： Navi＋EnterRoom → patrolRoom
# goal-->     確認する住人の設定数、
# Response--> 成功、中止、横取り
# Feedback--> Smachステート
# !!! Abortedになる状況を調べる
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
  pass

if __name__ == '__main__':
    pass