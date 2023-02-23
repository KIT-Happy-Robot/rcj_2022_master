#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import smach
from smach_ros import ActionServerWrapper
from std_msgs.msg import Bool, String, Float64
# Action
# MP features ---
#from mp_master.action import MpAction
import sys, roslib
mp_path = roslib.packages.get_pkg_dir("monitoring_patrol")
sys.path.imsert(0, mp_path)
from send_gmail.srv import SendGmail
from monitoring_patrol.srv import (AedLocationInfo,
                                   DetectLyingDown,
                                   AdNaviSrv) #, BioDetection*, Biometric*
from monitoring_patrol.action import (MpAct )
#from monitoring_patrol.action import BodyMotionDetectAct #, PatrolRoom
# recognition ---
from happymimi_recognition_msgs.srv import (RecognitionFind, RecognitionFindRequest
                                            RecognitionLocalize, RecognitionLocalizeRequest
                                            MotionDetection, MotionDetectionRequest
                                           )
# voice/sound ---
from playsound import playsound
from happymimi_voice_msgs.srv import *
from happymimi_msgs.srv import StrTrg
tts_sc = rospy.ServiceProxy('/tts', StrTrg) #!! ( ,TTS)
#happymimi_voice_path = (roslib.packages.get_pkg_dir("happymimi_voice")
#                        +"/../config/wave_data/aram.wav")
#sec_happymimi_voice_path = (roslib.packages.get_pkg_dir("happymimi_voice")
#                            +"/../config/wave_data/ga9du-ecghy2.wav")
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
# Ver2： 部屋のどこからでも外からでも対応。未知の位置でPatrolに遷移
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["start_finish", "navi_failed"],
                                 input_keys = ["start_loop_lim_in"])
        #self.navi_sc= rospy.ServiceProxy("/apps/ad_navi_server", AdNaviSrv)
        self.navi_count = 1
        
    def execute(self, userdata):
        #start_loop_lim = userdata.start_loop_lim_in
        rospy.loginfo("MP: Executing state: Start")
        tts_sc("Start patrolling")
        # 部屋まで位置に移動
        # ナビを制限回数内で実行
        while navi_count <= userdata.start_loop_lim_in:
            navi_res = navi_sc("start").result
            #navi_res = self.navi_sc("start").result
            if navi_res:
                bc(-45, 0.5)
                rospy.sleep(1.5)
                return "start_finish" #!!
            else: self.navi_count =+1; return "navi_failed"

# Ver1： WayPointに沿って移動し、ターゲットを探す
#       移動中に並列処理でPersonSerchしたい
#        i_tra: Start     | o_tra: Charm, Observe,
#        i_key: human_num |
# Ver2： 並列処理で部屋内の人探索
# 　　　 立ってる人を見つけたら->　Charm
#        倒れていたら->　Observe
# Ver3： 人検知に距離・サイズ制限を設け、明らかに違う物体を足切りする
# Ver4： 犬猫にも対応
class PatrolRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=[ 'lying_down', 'standing', 'found_nothing'],
                             input_key = ["human_num_in"],
                             output_keys=["human_num_out"])
        self.find_sc = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.adnavi_sc = rospy.ServiceProxy("/apps/ad_navi_server", AdNaviSrv)
        self.ad_navi_ac = rospy.()
        self.human_pose_sc = rospy.ServiceProxy("/person_feture/falling_down_rescue",SetFloat)
        #self.detect_target_ac = 
        self.head_pub = rospy.Publisher("/servo/head", Float64, queue_size=1)

    #!! 首を下げて常に人を探す
    def execute(self, userdata):
        rospy.loginfo("MP: Executing state: Patrol room")
        #self.search_sub = rospy.Subscriber('/mp/detect_target', #Bool,  ) #!!
        #while self.and not rospy.is_shutdown() and self.search_sub.data:
        
        # 首を下げて、探し回す
        # !! 3WayPoint移動＆Search
        self.head_pub("30")
        
        while self.adnavi_ac(RecognitionFindRequest(target_name='person')).result == False:
        #while self.find_sc(RecognitionFindRequest(target_name='person')).result == False:
            
        # 人発見
        if self.find_sc(RecognitionFindRequest(target_name='person')).result:
            # 人の姿勢検知
            #if getPosture().result == True:
            while self.human_pose_sc(SetFloatRequest()).data:
                tts_srv("Hi!")
                return "to_charm"
        
# Ver1：倒れている人の意識の有無
#       意識がある（音声警告レベル1,2）-> ObeyOrder(), in_order
#       ない　　　-> remey
# Ver2：人が動いているかを検知mp_mojule.biometricCheck()
# Ver3：動かない場合、生体測定をして、生物かを判断
class Observe(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes= ["unconscious", "to_assist", "to_charm"])

        
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
        smach.State.__init__(self,outcomes = ['rescue_finish'])

    def execute(self, userdata):
        rospy.loginfo("Executing state : Rescue")

        return "rescue_finish"


class Assist(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ["assist_finish"])

    def execute(self, userdata):
        rospy.loginfo("Executing state : Assist")
        
        return "receive_order_finish


# i_tra: peace
# o_tra: charm_done
# ご愛嬌 Hi5
class Charm(smach.State):
  def __init__(self):
        smach.State.__init__(self,outcomes = ['charm_finish'])
        
        # VOICE
        #self.order_voice_sc = rospy.ServiceProxy('', )
  def execute(self, userdata):
    rospy.loginfo("Executing state : Charm")
    
    return "charm_finish"
    
    
# i_tra: no_human, 
# i_key: human_num
class Back(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['all_finish'])
        
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
    def execute(self, userdata):
        print("Start going to operator")
        

def mpSmach():
    top = smach.StateMachine(outcomes = ["mp_finish",
                                         "mp_aborted",
                                         "mp_preempted",
                                         "guided",
                                         "assisted"])
    top.userdata.start_loop_lim = 2
    top.userdata.human_num = 0
    top.userdata.warn_level = 1
    top.userdata.order_in = 1
    top.userdata.action_order = ""
    with top:
        smach.StateMachine.add('Start', Start(),
                                transitions = {'start_finish':'PatrolRoom',
                                               "navi_failed":"Start"},
                                remapping = {"start_loop_lim_in":"start_loop_lim"})
        
        smach.StateMachine.add('PatrolRoom', PatrolRoom(),
                                transitions = {'lying_down':'Observe',
                                            'standing':'Charm',
                                            'found_notiong':'Back'})
        
        smach.StateMachine.add('Observe', Observe(),
                                transitions = {"unconcsious" : "Rescue",
                                               "to_assist" : "Assist"
                                               "to_charm" : "Charm"},
                                remapping = {"warn_level_in":"warn_level"})
        
        smach.StateMachine.add('Rescue', Rescue(),
                                transitions = {'rescue_finish':'guided'}) # MP finish
        
        smach.StateMachine.add('Assist', Assist(),
                                transitions = {'assist_finish':'assisted'} # MP finish
                                remapping = {"":""})
        
        smach.StateMachine.add('Charm', Charm(),
                                transitions = {'charm_finish':'Back'}
                                remapping = {"":""}))
        smach.StateMachine.add('Back', Back(),
                                transitions = {'all_finish':'mp_finish'}) # MP finish

    #outcome = top.execute()
  
        
class mpFuncs():
    def __init__(self):
        rospy.init_node("")
        rospy.loginfo('Ready to start **%s**', node_name)
        
    def createNode(self, node_name):
        rospy.init_node(node_name)
        rospy.loginfo('Ready to start **%s**', node_name)
        
    # デモでパトロールをすぐに実行
    # 
    def demoMain(self):
        self.createNode("mp_demo")
        MP_SMACH = mpSmach()
        # デモ Smach実行
        mp_demo_outcome = MP_SMACH.top.execute()
        
        rospy.spin()

    # 定位置についてパトロールを実行（mp_mode：外部で一定間隔の時間に基づいた実行）
    # !!　部屋に入っていない状態からスタート： Navi＋EnterRoom → patrolRoom
    # goal-->     確認する住人の設定数、
    # Response--> 成功、中止、横取り
    # Feedback--> Smachステート
    # !!! Abortedになる状況を調べる
    def mpAcserver(self):
        self.createNode("mp_acserver")
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
        

    # accoding to original start timer setting
    #def actualMp(): 



if __name__ == '__main__':
    pass
