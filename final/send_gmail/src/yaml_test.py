#!/usr/bin/env python
import rospy
import roslib
import sys
import yaml
file_path = roslib.packages.get_pkg_dir('send_gmail') + '/config/'
sys.path.insert(0, file_path)

rospy.init_node('yaml_test')

with open(file_path+'rcj_2021_final.yaml', 'r') as yml:
    address = yaml.load(yml)

print address['to']['kanazawa', 'washio']
