#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pprint
from sklearn.preprocessing import MultiLabelBinarizer
from sklearn.model_selection import KFold
import make_list
from sklearn.model_selection import train_test_split
from sklearn import svm
from sklearn.linear_model import LogisticRegression
from xgboost import XGBClassifier
from happymimi_voice_msgs.srv import StringToString
import rospy

#from sklearn.linear_model import LinearRegression


class gender_judge():
    def __init__(self):
        bi_name = []
        name,gender = make_list.make_list()
        for n in name:
            li = make_list.bigram(n)
            bi_name.append(li + ["E" +li[-1]] + ["E" + li[-1][-1]])
        gender_list = [1 if i == "m" else 0 for i in gender]
        self.mlb = MultiLabelBinarizer()
        self.mlb.fit_transform(bi_name)
        self.mlb.classes_
        self.train_features = self.mlb.transform(bi_name)
        print("loading")
        self.classifier = XGBClassifier(objective='binary:logistic', n_estimators=300, learning_rate=0.2)
        print("xgboost")
        self.classifier.fit( self.train_features, gender_list )
        self.server=rospy.Service('/gender_jg',StringToString,self.main)
        rospy.loginfo("server is ready")

    def main(self,name):
        name = str(name.req).lower()
        print(name)
        bi_name = []
        li = make_list.bigram(name)
        bi_name.append(li + ["E" +li[-1]] + ["E" + li[-1][-1]])
        print(bi_name)
        name_features = self.mlb.transform(bi_name)

        date = self.classifier.predict(name_features)
        if date[0] == 1:
            return "Male",True
        elif date[0] == 0:
            return "Female",True
        else:
            return None,False



if __name__ == '__main__':
    rospy.init_node('gender_jg')
    gender_judge()
    rospy.spin()
