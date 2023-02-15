#!/usr/bin/env python

import rospy
from send_gmail.srv import SendGmail, SendGmailResponse

class ListTest():
    def __init__(self):
        rospy.Service('/send_gmail_server', SendGmail, self.execute)
        rospy.loginfo("Ready to send_gmail_server")
        # Value
        self.list = []

    def execute(self, srv_req):
        try:
            self.list = srv_req.to_address
            print self.list
            count = len(self.list)
            print 'count: ' + str(count)
            return SendGmailResponse(result = True)
        except rospy.ROSInternalException:
            rospy.logger("!!Interrupt!!")
            return SendGmailResponse(result = False)

if __name__ == '__main__':
    rospy.init_node('send_gmail_node')
    lt = ListTest()
    rospy.spin()
