#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import rosparam
import roslib
import smach
import smach_ros
#音声やり取り用
from fmmmod import FeatureFromVoice, FeatureFromRecog,  LocInfo, SaveInfo
from std_msgs.msg import Float64
from happymimi_msgs.srv import SimpleTrg, StrTrg
from happymimi_navigation.srv import NaviLocation, NaviCoord

file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl

# speak
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
# wave_play
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)


class ApproachGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['approach_finish'],
                             input_keys = ['g_count_in'])
        # Service
        self.gen_coord_srv = rospy.ServiceProxy('/human_coord_generator', SimpleTrg)
        self.ap_srv = rospy.ServiceProxy('/approach_person_server', StrTrg)
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.navi_coord_srv = rospy.ServiceProxy('navi_coord_server', NaviCoord)
        # Topic
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.bc = BaseControl()

    def execute(self, userdata):
        rospy.loginfo("Executing state: APPROACH_GUEST")
        # return 'approach_finish'
        guest_num = userdata.g_count_in
        guest_name = "human_" + str(guest_num)
        #guest_name = "human_0"
       #human_loc = rospy.get_param('/tmp_human_location')
        
        self.bc.rotateAngle(180,1.0)
        # tts_srv("Move to guest")
        wave_srv("/fmm/move_guest")
        
        rospy.sleep(0.5)
        self.navi_srv('fmm')

        if guest_num == 0:
            self.head_pub.publish(0)
            rospy.sleep(1.0)
            #self.bc.translateDist(1.0,0.2)
           # rospy.sleep(1.0)
           # self.bc.rotateAngle(-90, 1.0)
           # rospy.sleep(1.0)
           # self.bc.translateDist(0.1,0.2)
           # rospy.sleep(1.0)
            result = self.gen_coord_srv().result
            print(result)
            rospy.sleep(1.0)
            #result = self.ap_srv(data = human_0)
            result = self.ap_srv(data = guest_name)

        elif guest_num == 1:
            self.head_pub.publish(0)
            self.bc.rotateAngle(-60, 1.0)
            rospy.sleep(1.0)
            self.bc.translateDist(0.2,0.2)
            #rospy.sleep(2.0)
            result = self.gen_coord_srv().result
            rospy.sleep(1.0)
            #result = self.ap_srv(data = human_1)
            print(result)
            #self.bc.rotateAngle(-330,0.2)
            # self.bc.rotateAngle(-10)
            # for i in range(3):
                # result = self.gen_coord_srv().result
                # if result:
                    # break
                # else:
                    # break
                    # self.bc.rotateAngle(-10)
            result = self.ap_srv(data = guest_name)
        elif guest_num == 2:
            self.head_pub.publish(0)
            rospy.sleep(1.0)
            #self.bc.translateDist(0.5,0.2)
            #rospy.sleep(1.0)
            self.bc.rotateAngle(-90, 1.0)
            rospy.sleep(1.0)
            #self.bc.translateDist(0.1,0.2)   
            #result = self.ap_srv(data = human_1)
            #rospy.sleep(2.0)
            result = self.gen_coord_srv().result
            #human_loc = rospy.get_param('/tmp_human_location')
            #self.human_cord = human_loc[human_1]
            #self.navi_coord_srv (loc_coord = human_1)
            rospy.sleep(1.0)
            result = self.ap_srv(data = guest_name)
            if result == false:
                guest_name = human_1
                result = self.ap_srv(data = geust_name)
        else:
            pass
        #result = self.ap_srv(data = guest_name)
        #print(result)
        self.head_pub.publish(0)
        if result:
            return 'approach_finish'
        else:
            return 'approach_finish'


class FindFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_finish'],
                             input_keys = ['g_count_in'],
                             output_keys = ['future_out'])
        # Topic
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.li = LocInfo()
        self.ffv = FeatureFromVoice()
        self.ffr = FeatureFromRecog()
        self.base_control = BaseControl()    
        self.guest_name  = "null"
        self.guest_loc   = "null"
        self.gn_sentence = "null"
        self.f1_sentence = "null"
        self.f2_sentence = "null"
        self.sentence_list = []

    def execute(self, userdata):
        rospy.loginfo("Executing state: FIND_FUATURE")
        self.head_pub.publish(-20)
        # tts_srv("Excuse me. I have a question for you")
        wave_srv("/fmm/start_q")
        self.base_control.translateDist(0.3,0.2)            
        self.guest_name = self.ffv.getName()
        #print (self.guest_name)
        self.guest_loc = self.li.nearPoint("human_" + str(userdata.g_count_in))
        self.gn_sentence = self.guest_name + " is near " + self.guest_loc
        #self.base_control = BaseControl()
        # self.gn_sentence = (self.guest_name + " is near table")
        if userdata.g_count_in == 0:
            # self.f1_sentence = "Height is " + self.ffr.getHeight()
            # self.f2_sentence = "Cloth color is " + self.ffr.getClothColor()
            self.f1_sentence = "Gender is " + self.ffv.getSex(self.guest_name)            
            self.f2_sentence = "Age is " + self.ffv.getAge()
        elif userdata.g_count_in == 1:
            self.head_pub.publish(0)
            self.base_control.translateDist(-0.3,0.2)
            rospy.sleep(0.5)
            self.head_pub.publish(-15)
            rospy.sleep(0.5)
            #self.f1_sentence = "Height is about " + self.ffr.getHeight() + " cm"
            self.f1_sentence = "Skin color is " + self.ffr.getSkinColor()
            self.f2_sentence = "Cloth color is " + self.ffr.getClothColor()
            # self.f2_sentence = "Age is " + self.ffv.getAge()
        elif userdata.g_count_in == 2:
            self.f1_sentence = "Description1 is No information"
            self.f2_sentence = "Description2 is No information"
            pass
        else:
            return 'find_finish'
        # tts_srv("Thank you for your cooperation")
        wave_srv("/fmm/finish_q")
        userdata.future_out = [self.gn_sentence, self.f1_sentence, self.f2_sentence]
        return 'find_finish'


class TellFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['tell_finish'],
                             input_keys = ['g_count_in', 'future_in'],
                             output_keys = ['g_count_out'])
        # Service
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.save_srv = rospy.ServiceProxy('/recognition/save', StrTrg)
        # Topic
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Value
        self.sentence_list = []
        self.si = SaveInfo()
        self.bc = BaseControl()

    def execute(self, userdata):
        rospy.loginfo("Executing state: TELL_FUATURE")
        guest_num = userdata.g_count_in
        self.sentence_list = userdata.future_in
        # tts_srv("Move to operator")
        rospy.sleep(0.2)
        wave_srv("/fmm/move_operator")
        self.bc.rotateAngle(180, 1.0)
        rospy.sleep(0.5)
        navi_result = self.navi_srv('operator').result
        # navi_result = True
        self.head_pub.publish(-15)
        if navi_result:
            # tts_srv("I'll give you the guest information.")
            wave_srv("/fmm/start_req")
        else:
            # tts_srv("I'm sorry. I couldn't navigate to the operator's location. I will provide the features from here.")
            wave_srv("/fmm/start_req_here")
        print (self.sentence_list)
        for i in range(len(self.sentence_list)):
            tts_srv(self.sentence_list[i])
            i += 1
        # yamlにでーたを保存
        self.si.saveInfo("guest_" + str(guest_num), self.sentence_list)
        userdata.g_count_out = guest_num + 1
        # self.bc.rotateAngle(90, 0.3)
        return 'tell_finish'


class Operation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_test', 'all_finish'],
                             input_keys = ['g_count_in'])

        self.bc = BaseControl()

    def execute(self, userdata):
        rospy.loginfo("Executing state: OPERATION")
        guest_count = userdata.g_count_in
        if guest_count == 0:
            # tts_srv("Start Find My Mates")
            wave_srv("/fmm/start_fmm")
            #self.bc.rotateAngle(90, 1.0)
            return 'start_test'
        elif guest_count > 2:
            # tts_srv("Finish Find My Mates. Thank you very much")
            wave_srv("/fmm/finish_fmm")
            return 'all_finish'
        else:
            #self.bc.rotateAngle(90, 1.0)
            return 'start_test'


if __name__ == '__main__':
    rospy.init_node('fmm_master')
    rospy.loginfo("Start Find My Mates")
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.guest_count = 0
    with sm_top:
        smach.StateMachine.add(
                'OPERATION',
                Operation(),
                transitions = {'start_test':'APPROACH_GUEST',
                               'all_finish':'finish_sm_top'},
                remapping = {'g_count_in':'guest_count'})

        smach.StateMachine.add(
                'APPROACH_GUEST',
                ApproachGuest(),
                transitions = {'approach_finish':'FIND_FEATURE'},
                remapping = {'g_count_in':'guest_count'})

        smach.StateMachine.add(
                'FIND_FEATURE',
                FindFeature(),
                transitions = {'find_finish':'TELL_FEATURE'},
                remapping = {'future_out':'guest_future',
                             'g_count_in':'guest_count'})

        smach.StateMachine.add(
                'TELL_FEATURE',
                TellFeature(),
                transitions = {'tell_finish':'OPERATION'},
                remapping = {'future_in':'guest_future',
                             'g_count_in':'guest_count',
                             'g_count_out':'guest_count'})

    outcome = sm_top.execute()
