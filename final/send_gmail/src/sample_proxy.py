#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------

import rospy
import smtplib
from email.mime.text import MIMEText

SMTP_HOSTS = 'smtp.gmail.com'
SMTP_PORT = 587

FROM_EMAIL_ADDRESS = 'euroysk9@gmail.com'
PASSWORD = 'qgplhqyrnllcuvsi'

PROXY_HOST = 'wwwproxy.kanazawa-it.ac.jp'
PROXY_PORT = 8080

TO_EMAIL_ADDRESS = 'euroysk9@gmail.com'
MAIL_SUBJECT = 'テストメール2'
BODY_MESSAGE = 'これはテスト用のメールです'

def make_message(mail_subject, from_address, to_address, body_message):
    msg = MIMEText(body_message, "html")
    msg['Subject'] = mail_subject
    msg['From'] = from_address
    msg['To'] = to_address
    return msg

def set_smtp_client_with_proxy(smtp_host, smtp_port):
    smtp_client = smtplib.SMTP(smtp_host, smtp_port)
    smtp_client.ehlo()
    smtp_client.starttls()
    smtp_client.ehlo()
    return smtp_client

if __name__ == '__main__':
    rospy.init_node('sample_proxy')
    msg = make_message(MAIL_SUBJECT, FROM_EMAIL_ADDRESS, TO_EMAIL_ADDRESS, BODY_MESSAGE)
    smtp = set_smtp_client_with_proxy(SMTP_HOSTS, SMTP_PORT)
    print('smtp login attempt')
    smtp.login(FROM_EMAIL_ADDRESS, PASSWORD)
    print('smtp login success')
    print('smtp message send attempt')
    smtp.sendmail(FROM_EMAIL_ADDRESS, TO_EMAIL_ADDRESS, msg.as_string())
    print('smtp message send success')
    smtp.quit()
