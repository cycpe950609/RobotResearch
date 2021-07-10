#!/usr/bin/env python2
# coding=utf-8
# This is a singleton of ROSLog , maybe we should move it to ROSLogger 
from ROSLogger import ROSLogger as Logger
import rospy

# 利用 python 模組全域特性建立的Singleton logger
#       ifSaveLog,ifSaveTimeline
LOG = Logger(True,False)