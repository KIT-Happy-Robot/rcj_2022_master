#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
# Desc:
# Author: Hiroto Washio
# Date: 24/02/2022
#-------------------------------------------------------------------
import time
import sys
import rospy
from std_msgs.msg import String, Float64
import smach
import smach_ros
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
#from common_action_client import *
#from common_function import *
#from mimi_common_pkg.srv import ManipulateSrv, RecognizeCount 
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from happymimi_navigation.srv import NaviLocation
from happymimi_voice_msgs.srv import YesNo
from happymimi_recognition_msgs.srv import RecognitionFind, RecognitionFindRequest
from happymimi_recognition_msgs.srv import RecognitionLocalizeRequest
from happymimi_msgs.srv import StrTrg
import roslib
base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl
recog_path = roslib.packages.get_pkg_dir('recognition_processing') + '/src/'
sys.path.insert(0, recog_path)
from recognition_tools import RecognitionTools
tts_srv = rospy.ServiceProxy('/tts', StrTrg)


class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found_lying','found_standing',
                                            'not_found_one','not_found_two'])
        #self.localize_srv = rospy.ServiceProxy('/recognition/localize', RecognitionLocalize)
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        self.navi_srv = rospy.ServiceProxy('/navi_location_server',NaviLocation)
        self.bc = BaseControl()
        self.rt = RecognitionTools()
        navi_counter = 0
    def execute(self, userdata):
        rospy.loginfo("start SearchPerson")
        navi_counter =+ 1
        self.head_pub.publish(22)
        rospy.sleep(3.0)
        print"down angle"
        #self.bc.rotateAngle(45, 0.3)
        #rospy.sleep(5.0)
        self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
        #rospy.sleep(5.0)
        #self.head_pub.publish(0)
        if self.find_result == True:
            print("found person")
            request = RecognitionLocalizeRequest()
            request.target_name = "person"
            centroid = self.rt.localizeObject(request).point
            person_height = centroid.z
            print person_height
            standard_value_z = 0.4
            #self.localize_srv(target_name="person")
            #localize_result = self.localize_srv(RecognitionLocalizeReqest)
            #z_value = localize_result[3]
            if person_height >= standard_value_z:
                target_name = 'standing_person'
                tts_srv("come to the operator")
                return 'found_standing'
            else :
                target_name = 'lying_person'
                return 'found_lying'
        elif self.find_result == False:
            print("find no person")
            tts_srv("find no person")
            if navi_counter == 1:
                tts_srv("start navigation")
                self.navi_srv('sec_search_point')
                #self.bc.rotateAngle(90,0.3)
                rospy.sleep(0.5)
                rospy.loginfo('finish moving to another place')
                return 'not_found_one'
            elif navi_counter > 1:
                print"found no person again"
                return 'not_found_two'


if __name__ == '__main__':
    rospy.init_node('falling_down_resc')
    rospy.loginfo('Start informing people lying down')
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
        'SearchPerson',
        SearchPerson(),
        transitions = {'found_lying':'finish_sm_top',
                       'found_standing':'finish_sm_top',
                       'not_found_one':'SearchPerson',
                       'not_found_two':'finish_sm_top'})
outcome = sm_top.execute()
