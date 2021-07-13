#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from math import exp,cos,sin,sqrt
import rospy
from Logger import LOG
import numpy as np
from Constant import BURGER_MAX_ANG_VEL, BURGER_MAX_LIN_VEL
from geometry_msgs.msg import Twist,Point,Pose,Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class StateBase(object):
    def __init__(self):
        super(StateBase,self).__init__()
        self._pose = Pose()
        self._direction = 0.0
        self._last_direction = 0.0

    def _EuclideanDistance( self, p1 ,p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return sqrt( dx*dx + dy*dy + dz*dz )

    def _initTwist(self):
        rtv = Twist()
        rtv.linear.x = 0.0
        rtv.linear.y = 0.0
        rtv.linear.z = 0.0

        rtv.angular.x = 0.0
        rtv.angular.y = 0.0
        rtv.angular.z = 0.0
        return rtv

    def _normalize_speed(self,u):
        # 限制linear 和 anglar 的大小
    
        LINEAR_MAX = BURGER_MAX_LIN_VEL

        # next_time = self.last_time + self.last_duration
        # duration_time = next_time - rospy.Time.now()

        # last_next_direction = ( self.last_direction + u[1] )
        # now_next_direction  = ( self.direction + u[1]*duration_time.to_sec() )

        # if(u[1] >= 0):# Turn left
        #     if(now_next_direction <= last_next_direction):
        #         ANGULAR_MAX = BURGER_MAX_ANG_VEL
        #     else:# Turn left too much
        #         ANGULAR_MAX = last_next_direction - self.direction
        # else:# Turn right
        #     if(now_next_direction >= last_next_direction):#
        #         ANGULAR_MAX = BURGER_MAX_ANG_VEL
        #     else:# Turn right too much
        #         ANGULAR_MAX = self.direction - last_next_direction

        # if(u[0]*duration_time.to_sec() > BURGER_MAX_ANG_VEL):
        #     ANGULAR_MAX = BURGER_MAX_ANG_VEL
        # else:
        #     ANGULAR_MAX = BURGER_MAX_ANG_VEL*duration_time.to_sec()
        # rospy.loginfo("ANGULAR_MAX : %lf"%(ANGULAR_MAX))

        ANGULAR_MAX = BURGER_MAX_ANG_VEL

        u[1] = self._last_direction + u[1] - self._direction

        w = [0,0]
        if(u[0] > LINEAR_MAX):# Linear
            # w[1] = u[1]*float(BURGER_MAX_LIN_VEL)/u[0]
            w[1] = u[1]
            w[0] = float(LINEAR_MAX)
            # print(u)
        elif(u[0] < -LINEAR_MAX):
            # w[1] = u[1]*float(-BURGER_MAX_LIN_VEL)/u[0]
            w[1] = u[1]
            w[0] = float(-LINEAR_MAX)



        if(w[1] > ANGULAR_MAX):# Angular
            w[0] = w[0]*float(ANGULAR_MAX)/w[1]
            w[1] = float(ANGULAR_MAX)
        elif(u[1] < -ANGULAR_MAX):# Angular
            w[0] = w[0]*float(-ANGULAR_MAX)/w[1]
            w[1] = float(-ANGULAR_MAX)

        return w

    def updatePose(self,pose):
        assert type(pose) is Pose, 'Must be Pose'
        # Get Euler Angle from Quaternion
        (r, p, yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self._last_direction = self._direction
        self._direction = yaw
        self._pose = pose

    def getDirection(self):
        raise NotImplementedError

    def getTwist(self):
        raise NotImplementedError

    def getStateName(self):
        return self.__class__.__name__


    