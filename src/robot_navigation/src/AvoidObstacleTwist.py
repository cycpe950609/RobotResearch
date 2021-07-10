#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from StateBase import StateBase
from math import acos,pi,sqrt,cos,sin
from sensor_msgs.msg import LaserScan
from Logger import LOG
from StateBase import StateBase

class AvoidObstacleTwist(StateBase):
    def __init__(self):
        super(AvoidObstacleTwist,self).__init__()
        self._data = LaserScan()
        self._obstacle_degree = 0.0
        self._obstacle_distance = float("inf")
        self._ao_twist = self._initTwist()

    def _getTwistfromVectorAO(self,v):
        twist = [0,0]
        x = v[0]
        y = v[1]
        r = sqrt(x ** 2 + y ** 2)
        # rospy.loginfo("VectorAO : ( %lf , %lf ) ",v[0],v[1])
        theta = 0
        if(r != 0):
            theta = acos(x/r)
        if(v[0] >= 0 and v[1] >= 0): # 第一象限
            twist = [r,pi/2 - theta]
        elif(v[0] < 0 and v[1] >= 0): # 第二象限
            twist = [r,theta - pi/2]
        elif(v[0] >= 0 and v[1] < 0): # 第四象限
            twist = [-r,pi/2 - theta]
        elif(v[0] < 0 and v[1] < 0): # 第三象限
            twist = [-r,-(theta - pi/2)]
        else:
            twist = [0,0]

        # rospy.loginfo("TwistAO : ( %lf , %lf ) ",twist[0],twist[1])
        return twist


    def getDirection(self):
        return self._obstacle_degree + pi

    def getTwist(self):
        return self._ao_twist
    
    def updateLaserScan(self,msg):
        self._obstacle_distance = min(msg.ranges) # min
        self._obstacle_degree = ( msg.angle_min + msg.ranges.index( self._obstacle_distance )*msg.angle_increment )

        LOG.LogNumber('obstacle_distance','at getDistanceCallback',self._obstacle_distance)
        LOG.LogNumber('obstacle_degree','at getDistanceCallback',self._obstacle_degree)

        if( msg.range_min < self._obstacle_distance < msg.range_max ):
            # Avoid Obstacle
            c = 1
            e = 0
            k = (1/self._obstacle_distance)*(c/(self._obstacle_distance ** 2 + e))

            
            u = [ k * self._obstacle_distance * cos(self._obstacle_degree + pi ) , k * self._obstacle_distance * sin(self._obstacle_degree + pi) ] # k

            u2twist = self._normalize_speed( self._getTwistfromVectorAO(u) )
            self._ao_twist = self._initTwist()
            self._ao_twist.linear.x = u2twist[0]
            self._ao_twist.angular.z = u2twist[1]

        else:
            self._obstacle_distance = float("inf")
            self._ao_twist = self._initTwist()
        