#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from StateBase import StateBase
from math import acos,pi,sqrt,sin,cos
from Logger import LOG
import numpy as np

class fw_base(StateBase):
    def __init__(self):
        super(fw_base,self).__init__()
        self._rotate_matrix = [[1,0],[0,1]]
        self._obstacle_degree = 0.0
        self._obstacle_distance = float("inf")
        self._fw_twist = self._initTwist()

    def _getTwistfromVectorFW(self,v):
        twist = [0,0]
        x = v[0]
        y = v[1]
        r = sqrt(x ** 2 + y ** 2)
        theta = 0
        if(r != 0):
            theta = acos(x/r)
        if(v[0] >= 0 and v[1] >= 0): # 第一象限
            twist = [r,pi/2 - theta]
        elif(v[0] < 0 and v[1] >= 0): # 第二象限
            twist = [r,theta - pi/2]
        elif(v[0] >= 0 and v[1] < 0): # 第四象限
            twist = [r,-(pi/2 + theta)]
        elif(v[0] < 0 and v[1] < 0): # 第三象限
            twist = [r,pi*3/2 - theta]
        else:
            twist = [0,0]

        return twist


    def getDirection(self):
        raise NotImplementedError # This will overrideed by child class

    def getTwist(self):
        return self._fw_twist

    def updateLaserScan(self,msg):
        self._obstacle_distance = min(msg.ranges) # min
        self._obstacle_degree = ( msg.angle_min + msg.ranges.index( self._obstacle_distance )*msg.angle_increment )

        LOG.LogNumber('obstacle_distance','at getDistanceCallback',self._obstacle_distance)
        LOG.LogNumber('obstacle_degree','at getDistanceCallback',self._obstacle_degree)

        alpha = 1

        if( msg.range_min < self._obstacle_distance < msg.range_max ):
            # Avoid Obstacle
            c = 1
            e = 0
            k = (1/self._obstacle_distance)*(c/(self._obstacle_distance ** 2 + e))

            
            u = [ k * self._obstacle_distance * cos(self._obstacle_degree + pi ) , k * self._obstacle_distance * sin(self._obstacle_degree + pi) ] # k
            u_fw = alpha* np.array(self._rotate_matrix).dot( np.array([ [ u[0] ],[ u[1] ] ]) )

            u2twist = self._normalize_speed( self._getTwistfromVectorFW( [ u_fw[0][0] , u_fw[1][0] ] ) )

            self._fw_twist = self._initTwist()
            self._fw_twist.linear.x = u2twist[0]
            self._fw_twist.angular.z = u2twist[1]

class FollowWallClockwiseTwist(fw_base):
    def __init__(self):
        super(FollowWallClockwiseTwist,self).__init__()
        self._rotate_matrix = [ [0,1],[-1,0] ]
    
    def getDirection(self):
        return (self._obstacle_degree + pi/2)%(2*pi)


class FollowWallCounterClockwiseTwist(fw_base):
    def __init__(self):
        super(FollowWallCounterClockwiseTwist,self).__init__()
        self._rotate_matrix = [ [0,-1],[1,0] ]
    
    def getDirection(self):
        return (self._obstacle_degree + pi*3/2)%(2*pi)



        