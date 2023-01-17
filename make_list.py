#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import os

file_path = os.path.expanduser('~/catkin_ws/src/rcj_2022_master/config/')

def make_list():
    name = []
    gender = []
    with open(file_path + 'yob2021.txt','r') as f:
        lines = f.read()
        lines = lines.lower()
        l = re.split(',|\n',lines)
        name += l[::3]
        name = [a for a in name if a != '']
        gender += l[1::3]
        gender = [a for a in gender if a != '']
    return name,gender

def bigram(text):
    return [text[i:i+2] for i in range(len(text) - 1)]

def end_name(bigram):
    return bigram[-1]



if __name__ == "__main__":
    list = make_list()
    print(list)
    for l in list[0]:
        print(bigram(l))
