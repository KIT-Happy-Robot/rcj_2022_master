#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# (Yumeko test spase: 36.53247090438674, 136.63104443023042)

# Geolocation accuracy

# Gmap point: 
#  - LC: 36.53163897927895, 136.6271756167569
#  - Yumeko test spase: 36.53247090438674, 136.63104443023042

# API point : latitude":"36.5968 longitude":"136.5998"

from yaml import load
import requests
import rospy
import rosparam
from std_msgs.msg import String, Float64

import sys;
sys.path.insert(0, '/home/hiroto/test_ws/src/dev_noe/task/rcj22/final')
from monitoring_patrol.srv import AedLocationInfo, AedLocationInfoResponse

class AedLocationServer():
  def __init__(self):
    rospy.loginfo('Start "AED locatioin server"')
    rospy.Service('/info/near_aed_location', AedLocationInfo, self.aedLocInfoCB)

    # accuracy of this is so bad 
    self.geocoord_dict = rosparam.get_param("/geo_coord_dict") #!!!
    self.req = ""
    self.tar_geocoord = ""
  # 
  def createURL(self, lat, lng, rad):
    url = "https://aed.azure-mobile.net/api/AEDSearch?lat="+str(lat)+"&lng="+str(lng)+"&r="+str(rad)
    return url
    
  def searchLocationName(self, name):
    if name in self.geocoord_dict:
      return True
    else: return False
  
  # return nearest AED location. Goejs 
  def aedLocInfoCB(self, req):
    rospy.loginfo('AED location server: GET Service Request')
    res = AedLocationInfoResponse()
    
    if self.searchLocationName(req.location_name): pass
    else : 
      rospy.loginfo("AED location server: The location name doesn't exist")
      return res.result == False #!!! �I
    
    # create URL
    self.tar_geocoord_list = self.geocoord_dict[req.location_name]
    url = self.createURL(self.tar_geocoord_list[0], self.tar_geocoord_list[1], req.radius)
    
    api_res = requests.get(url)
    api_data = requests.get(url).json()
    #if  api_res == 200:
    try:
      api_res.raise_for_status()
    except requests.exceptions.HTTPError:
      rospy.loginfo("AED location server: HTTPError")
      return res.result == False #!!! �I
    
    # if get success status
    rospy.loginfo("AED location server: GET SUCCESS Status Code")
    
    # confirm api_data has any elements
    if 0 < len(api_data): pass
    else:
      rospy.loginfo("AED location server: Recieved no data")
      return AedLocationInfoResponse(result = False)

    nearest_aed_dict = api_data[0]

    rospy.loginfo("AED location server: Response the recieved data")
    res.result        = True
    res.aed_num       = len(api_data)
    res.dist          = nearest_aed_dict["DIST"]
    res.location_name = nearest_aed_dict["LocationName"]
    #res.prefecture    = nearest_aed_dict["Prefecture"]
    res.city          = nearest_aed_dict["City"]
    res.address_area  = nearest_aed_dict["AddressArea"]

    res.latitude      = nearest_aed_dict["Latitude"]
    res.longitude     = nearest_aed_dict["Longitude"]
    return res
  
def main():
  rospy.init_node('aed_location_server', anonymous=True)
  try:
    ALS = AedLocationServer()
    rospy.spin()
  except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
    main()