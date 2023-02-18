#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#　** ターゲットを検知したら、パブリッシュし、３次元位置を取得する **

# ・ロボットの首、動きまわす台車制御は、マスター上でする
# ・ターゲットを検知したら、止まらせる
# 　→　** ターゲット検知パブリッシャー** が必要
# 　

#　FindObjectのパブリッシュ版
#  ↑の結果をbool型で常にパブリッシュするノード
#　（それか常にターゲットを検知するサーバー）

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes

import os, sys, roslib
mimi_reco_msgs_path = roslib.packages.get_pkg_dir("happymimi_recognition_msgs")
sys.path.imsert(0, mimi_reco_msgs_path)
from happymimi_recognition_msgs.srv import (RecognitionFind, 
                                            RecognitionFindRequest, 
                                            RecognitionFindResponse)
# ----------------------------------------
# mimi_reco_proc_path = roslib.packages.get_pkg_dir("recognition_processing")
# sys.path.imsert(0, os.path.join(mimi_reco_proc_path, 'src/'))
# from recognition_tools import CallDetector, RecognitionTools
# ----------------------------------------


class DetectTargetPublisher():
  bbox = []

  def __init__(self):
    rospy.loginfo('start **detect_target**')
    rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.boundingBoxCB)
    rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
    rospy.Service('/recognition/find',RecognitionFind, self.detectTarget)
      
  def detectTarget(self, request):
    rospy.loginfo('module type : Find')

    base_control = BaseControl()

    response_flg = RecognitionFindResponse()
    object_name = request.target_name
    loop_count = 0

    find_flg = bool(self.countObject(RecognitionCountRequest(object_name)).num)

    while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
        loop_count += 1

        rotation_angle = 30 - (((loop_count)%4)/2) * 60
        base_control.rotateAngle(rotation_angle, 0.5)
        rospy.sleep(3.0)

        bbox_list = self.createBboxList(RecognitionTools.bbox)
        if object_name == '':
            find_flg = bool(len(bbox_list))
        elif object_name == 'any':
            find_flg = bool(len(list(set(self.object_dict['any'])&set(bbox_list))))
        else:
            find_flg = object_name in bbox_list
    response_flg.result = find_flg
    return response_flg
  
  # def createNode(self, node_name):
  #   rospy.init_node(node_name)
  #   recognition_tools = RecognitionTools()
  #   rospy.spin()
    
  def main(self):
    rospy.init_node("detect_target")
    
    rospy.spin()
    
if __name__ == '__main__':
  DTP = DetectTargetPublisher()
  DTP.main()