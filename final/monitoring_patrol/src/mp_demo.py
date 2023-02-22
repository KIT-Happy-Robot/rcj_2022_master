#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 倒れている人に対応可能なパトロールをやるだけのFinalデモプログラム


import mp_master



if __name__=="__main__":
  mp_funcs = mp_master.MpFuncs()
  mp_funcs.createNode("mp_demo")
  mp_funcs.demoMain()