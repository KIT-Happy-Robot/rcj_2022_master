#!/usr/bin/env python
import rospy
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.init_node('time_pub')
    pub = rospy.Publisher('/timer', String, queue_size = 10)

