#!/usr/bin/env pythoan3
# -*- coding: utf-8 -*-
import os
import yaml
from scipy.spatial import distance
import tf
import rospy
import rosparam
import roslib
from happymimi_msgs.srv import SimpleTrg, StrTrg, StrToStr, SetFloat, SetStr
from happymimi_voice_msgs.srv import TTS, YesNo, StringToString

#from happymimi_voice_msgs.srv import StringToString
#file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
#sys.path.insert(0, file_path)
#from base_control import BaseControl 


# tts_srv
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
# wave_play
wave_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)

class FeatureFromVoice():
    def __init__(self):
        # Service
        self.feature_srv = rospy.ServiceProxy('get_feature_srv', StrToStr)
        self.yes_no_srv = rospy.ServiceProxy('/yes_no', YesNo)
        self.getsex_srv = rospy.ServiceProxy('/gender_jg',StringToString)

        # Value
        self.name = "null"
        self.age  = "null"
        self.sex  = "null"

    def yesNo(self):
        result = self.yes_no_srv().result
        return result

    def getName(self):
        self.name = "null"
        for i in range(3):
            name_res = self.feature_srv(req_data = "fmm name")
            print (name_res.res_data)
            if name_res.result:
                self.name = name_res.res_data
                tts_srv("Hi " + self.name)
                break
            elif i == 3:
                break
                # tts_srv("Sorry. I'm going to ask you one more time.")
            else:
                wave_srv("/fmm/ask_again")
                self.name = "guest"
        return self.name

    def getAge(self):
        self.age = "null"
        for i in range(3):
            age_res = self.feature_srv(req_data = "old")
            if i == 3:
                self.age = "unknown"
                break
            elif age_res.result:
                self.age = age_res.res_data
                tts_srv("Your age is" + self.age)
                # tts_srv("Is this OK? Please answer yes or no")
                wave_srv("/fmm/answer_yn")
                if self.yesNo():
                    break
                else:
                    # tts_srv("Sorry. I'm going to ask you one more time.")
                    # wave_srv("/fmm/ask_again")
                    pass
            else:
                # tts_srv("Sorry. I'm going to ask you one more time.")
                wave_srv("/fmm/ask_again")
        return self.age

    def getSex(self,req):
        self.sex = "null"
        # tts_srv("Are you a female? Please answer with yes or no")
        #wave_srv("/fmm/sex_q")
        #result = self.yes_no_srv().result
        res = self.getsex_srv(req)
        if res.result:
            self.sex=res.result_data 
        else:
            self.sex = "null"
        tts_srv("You are " + self.sex)

        #req = self.name
        #sel.sex = self.getsex_srv(req)
        #tts_srv("You are " + self.sex)
        print(self.sex)

        return self.sex

    
class FeatureFromRecog():
    def __init__(self):
        # Service
        #self.height_srv = rospy.ServiceProxy('/person_feature/height_estimation', SetFloat)
        #服
        self.cloth_srv  = rospy.ServiceProxy('/person_feature/cloth_color', SetStr)
        #ズボン
        #self.pants_srv = rospy.ServiceProxy('/person_feature/pants_color', SetStr)
        #顔
        #self.skin_srv = rospy.ServiceProxy('/person_feature/skin_color', SetStr)
        #髪
        #self.hair_srv = rospy.ServiceProxy('/person_feature/hair_color', SetStr)
        # Value
        #self.height      = "null"
        self.cloth_color = "null"
        #self.skin_color = "null"
        #self.pants_color = "null"
        #selk.hair_color = "null"
        #self.mask ="null"
        #Topic
        #self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        #self.bc = BaseControl()
    
    #def getPansColor(self):
    #    self.pants_color = "null"
    #    self.pants_color = self.pants_srv().result
        
    #    if self.pants_color == '':
    #        return "none"

    #    else:
    #        return self.pants_color


    #def getSkinColor(self):
    #    self.skin_color = "null"
    #    self.skin_color =  self.skin_srv().result
        
    #    if self.skin_color == '':
    #        return "none"
    #    else if self.skin_color == 'white':
    #        self.mask = "mask"
    #        return "none"
        
    #    else:
    #        return self.skin_color
    
    #def getMask(self):
    #    self.mask = "null"
    #    self.mask = self.skin_srv().result

    #    if self.skin_color == 'white':
    #        self.mask = "wearing a mask"
    #       return "none"
    #    else:
    #        self.mask = "not wearing a mask"


    #def getHairColo(self):
    #    self.hair_color = "null"
    #    self.hair_color = self.hair_srv().result
    #    if self.hair_color == '':
    #        return "none"
    #    else:
    #        return self.hair_color


    #def getHeight(self):
    #    self.head_pub.publish(0)
    #    self.base_control.translateDist(-0.5,0.2)
    #    
    #     height = SetFloat()
    #    height = self.height_srv()
    #    
    #    if height.data == -1:
    #        return False
    #    else:
    #        self.height = str(round(height.data))
    #        return self.height

    def getClothColor(self):
        self.cloth_color = "null"
        self.cloth_color = self.cloth_srv().result
        if self.cloth_color == '':
            return "none"
        else:
            return self.cloth_color
class LocInfo():
    def __init__(self):
        self.loc_dict   = rospy.get_param('/location')
        self.human_dict = {}
        self.loc_name_list = list(self.loc_dict.keys())
        self.loc_name      = "null"
        self.result = 0.00
        self.loc_result = "null"

    # 複数の座標のうちx, yに一番近い座標の名前を求める
    def nearPoint(self, target_name):
        self.loc_name = "null"
        self.human_dict = rospy.get_param('/tmp_human_location')
        print(self.human_dict)
        h_rpy = self.human_dict[target_name]
        h_xy = (h_rpy[0], h_rpy[1])
        for i in range(len(self.loc_name_list)):
            self.loc_name = self.loc_name_list[i]
            loc_rpy = self.loc_dict[self.loc_name]
            l_xy = (loc_rpy[0], loc_rpy[1])
            if i == 0:
                stdval = distance.euclidean(h_xy, l_xy)
            dist = distance.euclidean(h_xy, l_xy)
            #print (self.loc_name)
            #print (dist)
            if stdval > dist:
                stdval = dist
                self.loc_result = self.loc_name
        print (self.loc_result)
        return self.loc_result

class SaveInfo():
    def __init__(self):
        self.data_path = roslib.packages.get_pkg_dir("find_my_mates") + "/guest_info/"
        # Service
        # self.save_srv = rospy.ServiceProxy('/recognition/save', StrTrg)

    def saveInfo(self, name, data):
        rospy.loginfo('Save feature')
        file_name = name + ".yaml"
        with open(os.path.join(self.data_path, file_name), "w") as yf:
            yaml.dump(data, yf, default_flow_style = False)
        # self.save_srv(data = self.data_path)
