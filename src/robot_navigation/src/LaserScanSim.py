#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from math import acos, cos, pi, sin, sqrt

import rospy
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

from Logger import LOG

LASERSCAN_UPDATE_MIN = 0.15
LASERSCAN_UPDATE_MAX = 3.5

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
        self._first_init_laser_scan = True
        self._first_init_pose = True
        self._last_pose = Pose()
        self._pose = Pose()
        self._last_ranges = self.ranges[:]

        # Init Value for LaserScan
        self.angle_min = 0.0
        self.angle_max = 2*pi
        self.angle_increment = pi/180
        self.time_increment = 1
        self.scan_time = 0
        self.range_min = 0
        self.range_max = 0
        self.intensities = []

        
    def _EuclideanDistance( self, p1 ,p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return sqrt( dx*dx + dy*dy + dz*dz )

    def _vector2direction(self,vect):
        x = vect[0]
        y = vect[1]
        r = sqrt(x ** 2 + y ** 2)
        theta = 0.0
        degree = 0.0
        if(r != 0 and r < float('inf') ):
            theta = acos(x/r)
        if(x > 0 and y > 0): # 第一象限
            degree = theta
        elif(x <= 0 and y > 0): # 第二象限
            degree = theta
        elif(x > 0 and y < 0): # 第四象限
            degree = 2*pi - theta
        elif(x < 0 and y < 0): # 第三象限
            degree = 2*pi - theta
        elif(x == 0 and y > 0):# y軸正向
            degree = pi/2
        elif(x < 0 and y == 0):# x軸負向
            degree = pi
        elif(x == 0 and y < 0):# y軸負向
            degree = pi*3/2
        elif(x > 0 and y == 0):# x軸正向
            degree = 0.0
        else:
            degree = 0
        return degree

    def updateLaserScanWithPose(self,new_pose):# Update LaserScan between two msg from /scan
        LOG.Log('NewPose','At updateLaserScan',new_pose)

        if(self._first_init_pose):
            self._last_pose = new_pose
            self._pose = new_pose
            self._first_init_pose = False
        else:
            self._pose = new_pose
            # Update LaserScan
            (r, p, _last_direction) = euler_from_quaternion([ self._last_pose.orientation.x,  self._last_pose.orientation.y,  self._last_pose.orientation.z, self._last_pose.orientation.w])
            (r, p, ptn_dir) = euler_from_quaternion([ new_pose.orientation.x,  new_pose.orientation.y,  new_pose.orientation.z, new_pose.orientation.w])
            LOG.LogNumber('_last_direction','At updateLaserScan',_last_direction)
            LOG.LogNumber('ptn_dir','At updateLaserScan',ptn_dir)

            degree_pt0_ptnew = ( _last_direction - ptn_dir + 2*pi )%(2*pi)
            # LOG.LogNumber('ptn_pt0_dir','At updateLaserScan',ptn_pt0_dir)
            LOG.LogNumber('degree_pt0_ptnew','At updateLaserScan',degree_pt0_ptnew)

            # Get new range , new_range[i] is calculated from self._last_range[i] 
            new_range = list(range(360))
            pt_last = Point(self._last_pose.position.x , self._last_pose.position.y,0)
            pt_new = Point(new_pose.position.x , new_pose.position.y,0)
            for i in range(360):
                r_last_a = self._last_ranges[i]
                if(r_last_a < float('inf')):
                    pt_a = Point(self._last_pose.position.x + r_last_a*cos(_last_direction + i*self.angle_increment) , self._last_pose.position.y + r_last_a*sin(_last_direction + i*self.angle_increment ),0 )
                    new_range[i] = self._EuclideanDistance(pt_a,pt_new)
                else:
                    new_range[i] = float('inf')
            # Shift range
            last_new_index = (int( round(degree_pt0_ptnew / self.angle_increment) + 1 ))%(360)
            # TODO: When Turtlebot3 rotate, LaserScanSim rotated , but LaserScan didnt
            LOG.LogNumber('self.angle_increment','At updateLaserScan',self.angle_increment)

            for i in range(last_new_index):
                new_range.append(new_range.pop(0))
            self.ranges = new_range[:]
        LOG.Log('LaserScanSim','At updateLaserScan',self)
            
    
    def updateLaserScan(self,msg):# Update LasrScan when msg from /scan receive, only update rnage which is larger than min_range ( 0.12 )
        # Update LaserScan
        self.header = msg.header
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.time_increment = msg.time_increment
        self.scan_time = msg.scan_time
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        self.intensities = msg.intensities

        # self.ranges = msg.ranges[:]

        if(self._first_init_laser_scan):
            ranges = range(360)
            for i in range(360):
                if( msg.range_min <= msg.ranges[i] < msg.range_max):
                    ranges[i] = msg.ranges[i]
                else:
                    ranges[i] = float('inf')
            self.ranges = tuple(ranges)
            self._first_init_laser_scan = False
        else:
            ranges = range(360)
            for i in range(360):
                if( LASERSCAN_UPDATE_MIN < msg.ranges[i] < LASERSCAN_UPDATE_MAX ):
                    ranges[i] = msg.ranges[i]
                else:
                    ranges[i] = self.ranges[i]
            self.ranges = tuple(ranges)
        # Update Last data
        self._last_pose = self._pose
        self._last_ranges = self.ranges[:]

        LOG.Log('LaserScanSim','At updateLaserScan',self)
        