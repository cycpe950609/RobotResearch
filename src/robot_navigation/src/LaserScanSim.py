#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from sensor_msgs.msg import LaserScan
import rospy
from Logger import LOG

class LaserScanSim(LaserScan):
    def __init__(self):
        super(LaserScanSim,self).__init__()

        # Init LaserScanSim ranges' value to inf
        # Because be have 'min() arg is an empty sequence' when 'updateLaserScan'
        # Do we need this ?
        ranges = range(360)
        for i in range(360):
            ranges[i] = float('inf')
        self.ranges = tuple(ranges)
        self._first_init = True
        
    def updateLaserSanWithPose(self,new_pose):# Update LaserScan between two msg from /scan
        pass
    def updateLaserScan(self,msg):# Update LasrScan when msg from /scan receive, only update rnage which is larger than min_range ( 0.12 )
        self.header = msg.header
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.time_increment = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.intensities = msg.intensities
        if(self._first_init):
            self.ranges = msg.ranges
            self._first_init = False
        else:
            ranges = range(360)
            for i in range(360):
                if( msg.range_min < msg.ranges[i] < msg.range_max):
                    ranges[i] = msg.ranges[i]
                else:
                    ranges[i] = self.ranges[i]
            self.ranges = tuple(ranges)

        LOG.Log('LaserScanSim','At updateLaserScan',self)
        