#!/usr/bin/env python2
# coding=utf-8
from math import exp,cos,sin,sqrt
import rospy
from Logger import LOG
import numpy as np
from Constant import BURGER_MAX_LIN_VEL
from geometry_msgs.msg import Twist,Point,PoseStamped,Quaternion
from tf.transformations import quaternion_from_euler
from StateBase import StateBase

class Go2GoalTwist(StateBase):

    def __init__(self):
        super(Go2GoalTwist,self).__init__()
        self.GoalOfRobot = Point()
        self.GoalOfRobot.x = 0.0
        self.GoalOfRobot.y = 0.0

    def updateGoal(self,Goal):
        self.GoalOfRobot.x = Goal.Goal.x
        self.GoalOfRobot.y = Goal.Goal.y
        self.GoalOfRobot.z = Goal.Goal.z
        # self.GoalOfRobot = Goal

    

    def getDistance(self):
        return self._EuclideanDistance(self.GoalOfRobot , self._pose.position)

    def getDirection(self):
        return [self.GoalOfRobot.x - self._pose.position.x , self.GoalOfRobot.y - self._pose.position.y]

    def getTwist(self):

        LOG.LogFunction('getGo2GoalTwist')
        robot_post                      = np.array([
            [self._pose.position.x],
            [self._pose.position.y]
        ])

        robot_dest                      = np.array([
            [self.GoalOfRobot.x],
            [self.GoalOfRobot.y]
        ])

        epsilon = 0.1
        
        va               = np.array([
            [1,0],
            [0,1/epsilon]
        ])

        yaw = self._direction
        theta            = np.array([
            [cos(yaw) , sin(yaw)],
            [-sin(yaw) , cos(yaw)]
        ])
        
        e_g2g                       = robot_dest - robot_post

        e_g2g                       = e_g2g + 0.001

        alpha = 10.0
        e_g2g_normf = np.linalg.norm(e_g2g)
        k = BURGER_MAX_LIN_VEL*( (1 - exp( -alpha*e_g2g_normf*e_g2g_normf ))  / ( e_g2g_normf ) )

        u_g2g                       = e_g2g * k
        
        rlt                         = va.dot(theta).dot(u_g2g); 

        go2goal_twist = self._initTwist()
        go2goal_twist.linear.x = rlt[0][0]
        go2goal_twist.angular.z = rlt[1][0]

        # 計算機器人和Goal之間是否有障礙物
        # max_laser_value = -float("inf") # 機器人的方向和Goal方向的內積值最大時的距離 
        # max_inner_product = -float("inf")
        # for i in range(360):
        #     [x_r,y_r] = [ cos(self.LaserScanMsg.angle_min + i*self.LaserScanMsg.angle_increment) , sin(self.LaserScanMsg.angle_min + i*self.LaserScanMsg.angle_increment) ]
        #     [x_g,y_g] = e_g2g
        #     ip = x_r * x_g + y_r * y_g
        #     if(ip > max_inner_product):
        #         max_inner_product = ip
        #         max_laser_value = self.LaserScanMsg.ranges[i]

        # # rospy.loginfo("MAX_LASER_VALUE : %lf"%max_laser_value)
        # if( self.LaserScanMsg.range_min < max_laser_value < WarningDistance):
        #     self.ObstacleInRobot2Goal  = True
        # else:
        #     self.ObstacleInRobot2Goal = False


        return go2goal_twist

# go2goal_twist = Go2GoalTwist()