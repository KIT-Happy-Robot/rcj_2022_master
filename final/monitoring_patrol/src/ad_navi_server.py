#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# just a location_name requesst is preferred
# 首の角度は変えない
import rospy
import rosparam
import dynamic_reconfigure.client
import actionlib
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import sys
import roslib
#mimi_navi_path = roslib.packages.get_pkg_dir("happymimi_navigation")
self_path = roslib.packages.get_pkg_dir("monitoring_patrol")
sys.path.insert(0, "/${HOME}/test_ws/src/dev_noe/task/rcj22/final/")
#sys.path.insert(0, mimi_navi_path)

#sys.path.insert(0, "${HOME}/test_ws/src/dev_noe/task/rcj22/final")
self_path = roslib.packages.get_pkg_dir("monitoring_patrol") + "/src"
sys.path.insert(0, self_path)
#from happymimi_navigation.srv import NaviCoord, NaviCoordRequest
from monitoring_patrol.srv import AdNaviSrv, AdNaviSrvResponse

# 拡張版オプション付き自律移動の実行クラス
class AdNaviServer():
  def __init__(self, server_type = service):
    rospy.loginfo("Initialize **AdNaviServer**")
    # TOPIC
    # Publisher
    #self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
    
    # SERVICE
    #from navi_coord import NaviCoordServer
    #self.NCS = NaviCoordServer()
    #self.navi_srv = rospy.ServiceProxy('navi_coord_server', NaviCoord)

    rospy.loginfo("ac_navi_server: ready to **Ad-Navi-Server**")
    self.an_ss = rospy.Service('/apps/ad_navi_server', AdNaviSrv, self.execute)
    
    # ACTION
    self.move_base_ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
    self.dwa_c = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS')
    self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    
    # Param
    #self.navi_params_dict = rospy.get_param("/ad_navi_param")
    self.navi_def_params = rospy.get_param("/ad_navi_param/FastLoose")
    self.navi_fl_params = rospy.get_param("/ad_navi_param/Default")
    self.location_dict = rospy.get_param("/location_dict")
    self.target_coord = [] #!!
    # Data
    self.mb_goal = MoveBaseGoal
    
  # Requestの"option"に基づき、Naviのパラメーターを変更する関数
  def setParam(self, option = "default"): #!!
    if option == "fast_loose":
      self.dwa_c.update_configuration(self.navi_def_params)
    elif option == "default":
      self.dwa_c.update_configuration(self.navi_def_params)
      rospy.sleep(0.5)
  
  # ロケーション名の姿勢座標パラメーターを姿勢座標のリスト(self.target_coord)に格納する関数
  def name2Coord(self, location):
    if location in self.location_dict:
      self.target_coord = self.location_dict[location]
      rospy.loginfo("AdNavi: target location ->" +location)
      return AdNaviSrvResponse(result = True)
    else: 
      rospy.logerr("<"+location+"> doesn't exist.")
      return AdNaviSrvResponse(result = False)
    
  # 姿勢座標のリストからゴールを作成する関数
  def createGoal(self, coord_list):
    rospy.loginfo("ac_navi_server: Create move base goal")
    # set goal_pose
    self.goal = MoveBaseGoal()
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = coord_list[0]
    self.goal.target_pose.pose.position.y = coord_list[1]
    self.goal.target_pose.pose.orientation.z = coord_list[2]
    self.goal.target_pose.pose.orientation.w = coord_list[3]
    
  def clearCostmap(self):
    # clearing costmap
    rospy.loginfo("ac_navi_server: Clearing costmap...")
    rospy.wait_for_service('move_base/clear_costmaps')
    self.clear_costmap()
    rospy.sleep(0.5)
    
  def sendGoal(self, goal):
    rospy.loginfo("ac_navi_server: Send move base goal")
    # 首は上げない
    #self.head_pub.publish(0)
    self.move_base_ac.wait_for_server()
    self.move_base_ac.send_goal(goal)
    self.move_base_ac.wait_for_result()
    navi_act_state = self.move_base_ac.get_state()
    while not rospy.is_shutdown():
      navi_act_state = self.ac.get_state()
      if navi_act_state == 3:
        rospy.loginfo('Navigation success!!')

        return NaviLocationResponse(result = True)
      elif navi_act_state == 4:
        rospy.loginfo('Navigation Failed')
        return NaviLocationResponse(result = False)
      else:
        pass
    
  def execute(self, req):
    rospy.loginfo("Execute **ad_navi_server**")
    
    # Set the target coord list
    if req.location_name == "":
      self.target_coord = req.coordinate
    else: # in case of recieving a location_name
      # Judge name2Coord result
      if self.name2Coord(req.location_name): pass
      else: 
        rospy.loginfo("ac_navi_server: Failed to gen-coord from the location")
        return False
      
    # Set the navigation parameter according to requested Optio 
    self.setParam(option = req.option)
    
    # Create Goal
    self.createGoal(self.target_coord)
    # Send Goal
    self.sendGoal(self.goal)
      
      
      
def main():
  rospy.init_node('ad_navi_server')
  try:
    ans = AdNaviServer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
