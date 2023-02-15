#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#------------------------------------------------
# Title: Gamilでメールをを送信するServiceServer
# Author: Yusuke Kanazawa
# Data: 2022/03/02
# Memo: KIT-WLAP2じゃ送信できない
#       ポケットwifiかテザリングしてほしい
#-------------------------------------------------

import rospy
import roslib
import smtplib
import yaml
import sys
from email.mime.text import MIMEText
from send_gmail.srv import SendGmail, SendGmailResponse
file_path = roslib.packages.get_pkg_dir('send_gmail') + '/config/'
sys.path.insert(0, file_path)

SMTP_HOSTS = 'smtp.gmail.com'
SMTP_PORT = 587

class SendGmailServer():
    def __init__(self):
        rospy.Service('/send_gmail_server', SendGmail, self.execute)
        rospy.loginfo("Ready to send_gmail_server")
        # Value
        self.yaml = 'NULL'
        self.to_list = []
        self.to_address = []

    def load_file(self, file_name, to_address_list):
        with open(file_path+file_name+'.yaml', 'r') as yml:
            self.yaml = yaml.load(yml)
        list_count = len(to_address_list)
        if to_address_list[0] == 'all':
            self.to_list = self.yaml['to']['all']
        else:
            list_count = len(to_address_list)
            for count in range(list_count):
                self.to_list.append(self.yaml['to'][to_address_list[count]])
                rospy.sleep(0.2)
        print(self.to_list)
        return self.to_list

    def make_message(self, mail_subject, from_address, to_address, body_message):
        msg = MIMEText(body_message, "html")
        msg['Subject'] = mail_subject
        msg['From'] = from_address
        msg['To'] = to_address[0]
        return msg

    def set_smtp_client(self, smtp_host, smtp_port):
        smtp_client = smtplib.SMTP(smtp_host, smtp_port)
        smtp_client.ehlo()
        smtp_client.starttls()
        smtp_client.ehlo()
        return smtp_client

    def execute(self, srv_req):
        try:
            self.to_address = self.load_file(srv_req.file_name, srv_req.to_address)
            msg = self.make_message(srv_req.subject, self.yaml['from'][srv_req.from_address], self.to_address, srv_req.body)
            smtp = self.set_smtp_client(SMTP_HOSTS, SMTP_PORT)
            print('smtp login now ...')
            smtp.login(self.yaml['from'][srv_req.from_address], self.yaml['password'][srv_req.password])
            print('smtp login success')
            print('smtp message sending ...')
            smtp.sendmail(self.yaml['from'][srv_req.from_address], self.to_address, msg.as_string())
            print('smtp message send success')
            self.to_list[:] = []
            self.to_address[:] = []
            smtp.quit()
            return SendGmailResponse(result = True)
        except rospy.ROSInternalException:
            rospy.logger("!!Interrupt!!")
            return SendGmailResponse(result = False)

if __name__ == '__main__':
    rospy.init_node('send_gmail_node')
    sgs = SendGmailServer()
    rospy.spin()
