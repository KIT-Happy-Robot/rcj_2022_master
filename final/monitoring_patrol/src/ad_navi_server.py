#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# just a location_name requesst is preferred
import rospy
import rosparam
import dynamic_reconfigure.client

from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import sys, roslib
mimi_navi_path = roslib.packages.get_pkg_dir("happymimi_navigation")
sys.path.insert(0, mimi_navi_path)
#sys.path.insert(0, "${HOME}/test_ws/src/dev_noe/task/rcj22/final")
#from happymimi_navigation.srv import NaviCoord, NaviCoordRequest
from monitoring_patrol.srv import AdNaviSrv, AdNaviResponse


class AdNaviServer():
    def __init__(self):
      # TOPIC
      # Publisher
      self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
      
      # SERVICE
      #from navi_coord import NaviCoordServer
      #self.NCS = NaviCoordServer()
      #rospy.loginfo("ready to **Navi-Coord-Server**")
      rospy.loginfo("ready to **Ad-Navi-Server**")
      self.an_ss = rospy.Service('/apps/ad_navi_server', AdNaviSrv, self.execute)
      #self.navi_srv = rospy.ServiceProxy('navi_coord_server', NaviCoord)
      self.dwa_c = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
      self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
      
      #self.navi_params_dict = rospy.get_param("/ad_navi_param")
      self.navi_def_params = rospy.get_param("/ad_navi_param/FastLoose")
      self.navi_fl_params = rospy.get_param("/ad_navi_param/Default")
      self.location_dict = rospy.get_param("/location_dict")
      self.target_coord = [] #!!

    def setParam(self, option = "default"): #!!
      if option == "fast_loose":
        self.dwa_c.update_configuration(self.navi_def_params)
      elif option == "default":
        self.dwa_c.update_configuration(self.navi_def_params)
        rospy.sleep(0.5)
    
    def name2Coord(self, location):
      if location in self.location_dict:
        self.target_coord = self.location_dict[location]
        rospy.loginfo("AdNavi: target location ->" +location)
        return True
      else: 
        rospy.logerr("<"+location+"> doesn't exist.")
        return False
    
    
    def execute(self, req):
      rospy.loginfo("Execute **ad_navi_server**")
      
      # Set the target coord
      if req.location_name == "":
        self.target_coord = req.coordinate
      else: # in case of recieving a location name
        # Judge name2Coord result
        if self.name2Coord(req.location_name): pass
        else: 
          rospy.loginfo("ac_navi_server: Failed to gen-coord from the location")
          return False
        
      # Set the navigation parameter according to requested option
      self.setParam(option = req.option)
      
      
      
def main():
  rospy.init_node('ad_navi_server')
  try:
    ans = AdNaviServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
