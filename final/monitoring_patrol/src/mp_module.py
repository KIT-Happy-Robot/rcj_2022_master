#!/usr/bin/env python3
# -*- coding: utf-8 --
#
#

import rospy
# 体温、心拍(BPM)
from std_msgs.msg import Float32
# Biometric Manipulation
#from happymimi_manipulation import TouchSkin 

# MP Messages -------------
# 動作検知 SC
from monitoring_patrol.srv import DetectMotion, DetectMotionRequest
# 姿勢検知 SC
from monitoring_patrol.srv import DetectPosture, DetectPostureRequest
from monitoring_patrol.srv import AedLocationInfo
#from monitoring_patrol.srv import BioDetection
from send_gmail.srv import SendGmail
#from mp_master.action import MpAction
#----------------------




# Recognition Feature
class mpRecognition():
    def __init__(self):
        # sub
        self.body_temp = 0.0
    
    def BodyTempCB(self, msg):
        self.body_temp = msg
    
    # return: lying or not lying (or standing or sittiong/squatting)
    def getHumanPosture(self):
        
        return result

    # return: bool result
    def detectMotion(self):
        
        return result
    
    # Ver2  : 
    def biometricCheck():

# 生体測定（体温、心拍）
#!! 実行時にSubscノード生成と破棄をするか否か
class biometricCheck():
    def __init__(self):
        # Service Client
        self.motion_sc = rospy.ServiceProxy("/detect/motion", DetectMotion)
        
        self.mpr = self.mpRecognition()

    # return: float16 body_temp
    def getBodyTemp(self):
        return self.mpr.body_temp
    
    # return: lying or not lying (or standing or sittiong/squatting)
    def getPosture(self):
    
    # Manipulation
    def moveArm2Body(self):
        
    
    # return: string result("move", "not_moved")
    def checkBodyMotion(self):

class mpMc():
    def __init__(self):
