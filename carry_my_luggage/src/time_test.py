#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Time():
    def __init__(self):
        rospy.Subscriber('/timer', String, self.msgCB)
        rospy.Subscriber('/cmd_vel', Twist, self.laserCB)
        rospy.Subscriber('/yes_no', String, self.yesnoCB)
        self.start_time = time.time()
        self.yes_no = 'NULL'
        self.lost_msg = 'NULL'
        self.laser_msg = 0.0
        self.execute()

    def laserCB(self, receive_msg):
        self.laser_msg = receive_msg.linear.x

    def msgCB(self, receive_msg):
        self.lost_msg = receive_msg.data

    def yesnoCB(self, receive_msg):
        self.yes_no = receive_msg.data

    def execute(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            now_time = time.time() - self.start_time
            if self.lost_msg == "lost":
                self.start_time = time.time()
                self.lost_msg = 'lost_after'
                print("first if")
            elif self.lost_msg == "lost_after" and now_time >= 5:
                print("elif")
                rospy.wait_for_message('/yes_no', String)
                if self.yes_no == 'yes':
                    return 'yes_finish'
                else:
                    print("continue")
                    self.lost_msg = 'NULL'
            elif self.laser_msg != 0.0:
                self.lost_msg = 'NULL'
                print(self.lost_msg)
            else:
                print(now_time)

if __name__ == '__main__':
    rospy.init_node('time')
    t = Time()
    print(t)
