#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from math import acos, ceil, cos, floor, pi, sin, sqrt

import rospy
from geometry_msgs.msg import Point, Pose,Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion,quaternion_from_euler

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
        self._percise_last_pose  = False

        # Init Value for LaserScan
        self.angle_min = 0.0
        self.angle_max = 2*pi
        self.angle_increment = pi/180
        self.time_increment = 1
        self.scan_time = 0
        self.range_min = 0
        self.range_max = 0
        self.intensities = []

        self._last_time = rospy.Time.now()
        self._update_time = self._last_time
        self._last_twist = self._initTwist()
        
    def _initTwist(self):
        rtv = Twist()
        rtv.linear.x = 0.0
        rtv.linear.y = 0.0
        rtv.linear.z = 0.0

        rtv.angular.x = 0.0
        rtv.angular.y = 0.0
        rtv.angular.z = 0.0
        return rtv
        
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
    
    def _getLaserScan(self,new_pose): 
        # Update LaserScan
        (r, p, _last_direction_tmp) = euler_from_quaternion([ self._last_pose.orientation.x,  self._last_pose.orientation.y,  self._last_pose.orientation.z, self._last_pose.orientation.w])
        (r, p, ptn_dir_tmp) = euler_from_quaternion([ new_pose.orientation.x,  new_pose.orientation.y,  new_pose.orientation.z, new_pose.orientation.w])
        _last_direction = (_last_direction_tmp + 2*pi)%(2*pi)#range of _last_direction_tmp is -pi ~ pi, update them to 0 ~ 2pi
        ptn_dir = (ptn_dir_tmp + 2*pi)%(2*pi)#range of ptn_dir_tmp is -pi ~ pi, update them to 0 ~ 2pi
        LOG.LogNumber('_last_direction','At updateLaserScan',_last_direction)
        LOG.LogNumber('ptn_dir','At updateLaserScan',ptn_dir)

        pt0 = Point(self._last_pose.position.x + self._last_ranges[0]*cos(_last_direction) , self._last_pose.position.y + self._last_ranges[0]*sin(_last_direction) ,0)
        ptn_pt0_dir = self._vector2direction([ pt0.x - new_pose.position.x ,  pt0.y - new_pose.position.y ])

        degree_pt0_ptnew = ( ptn_pt0_dir - ptn_dir + 2*pi )%(2*pi)

        # degree_pt0_ptnew = ( _last_direction - ptn_dir + 2*pi )%(2*pi)
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
        # LOG.Log('new_range','At updateLaserScan',new_range)
        # Shift range
        # last_new_index = (int( round(degree_pt0_ptnew / self.angle_increment) + 1 ))%(360)
        angle_increment = 2*pi/360
        # last_new_index = ((degree_pt0_ptnew / self.angle_increment) + 1)%(360)# self.angle_increment is not 1 degree precisely
        last_new_index = ((degree_pt0_ptnew / angle_increment))%(360)
        # floor_lni = floor(last_new_index)
        # ceil_lni = ceil(last_new_index)
        # TODO: When Turtlebot3 rotate, LaserScanSim rotated , but LaserScan didnt
        LOG.LogNumber('last_new_index','At updateLaserScan',last_new_index)

        base_idx = int(round(last_new_index)%(360))
        floor_idx = int(floor(last_new_index)%(360))
        LOG.LogNumber('base_idx','At updateLaserScan',base_idx)
        LOG.LogNumber('floor_idx','At updateLaserScan',floor_idx)
        final_range = list(range(360))
        delta_floor_center = last_new_index - floor(last_new_index)
        delta_ceil_center = ceil(last_new_index) - last_new_index
        if(base_idx == floor_idx):
            LOG.LogFunction('Base_Equal_Floor')
            for i in range(360):#用內插法計算距離
                f_idx = i
                c_idx = (i+1)%(360)
                # final_range[(i - base_idx + 360)%(360)] = delta_floor_center*new_range[c_idx] + delta_ceil_center*new_range[f_idx]
                final_range[i] = delta_floor_center*new_range[c_idx] + delta_ceil_center*new_range[f_idx]
        else:
            LOG.LogFunction('Base_Not_Equal_Floor')
            for i in range(360):#用內插法計算距離
                f_idx = (i-1+360)%(360)
                c_idx = i
                # final_range[(i - base_idx + 360)%(360)] = delta_floor_center*new_range[c_idx] + delta_ceil_center*new_range[f_idx]
                final_range[i] = delta_floor_center*new_range[c_idx] + delta_ceil_center*new_range[f_idx]

        for i in range(base_idx):
            final_range.append(final_range.pop(0))
        # for i in range(360):#用內插法計算距離
            # f_lni   = (floor_lni + i)%(360)
            # c_lni   = (ceil_lni + i)%(360)
            # lni     = (last_new_index + i)%(360)
            
            # delta_floor_center = lni - f_lni
            # delta_ceil_center = c_lni - lni
            # if(floor_idx == base_idx):
            #     final_range[i] = delta_floor_center*new_range[int(c_lni)] + delta_ceil_center*new_range[int(f_lni)]
            # else:
            #     final_range[(i-1+360)%(360)] = delta_floor_center*new_range[int(c_lni)] + delta_ceil_center*new_range[int(f_lni)]

        # TODO: There is nan and large number in range. WHY? 
        return final_range

    def updateLaserScanWithPose(self,new_pose):#:Pose):# Update LaserScan between two msg from /scan
        LOG.Log('NewPose','At updateLaserScan',new_pose)

        now_time = rospy.Time.now()


        if(self._percise_last_pose == False):
            delta_last_LaserScanUpdate = (self._update_time - self._last_time).to_nsec()
            delta_LaserScanUpdate_now = (now_time - self._update_time).to_nsec()
            LOG.LogNumber('delta_last_LaserScanUpdate','At updateLaserScan',delta_last_LaserScanUpdate)
            LOG.LogNumber('delta_LaserScanUpdate_now','At updateLaserScan',delta_LaserScanUpdate_now)
            #用內插法估計座標
            self._last_pose.position.x = (delta_last_LaserScanUpdate*new_pose.position.x + delta_LaserScanUpdate_now*self._last_pose.position.x)/(delta_last_LaserScanUpdate + delta_LaserScanUpdate_now)
            self._last_pose.position.y = (delta_last_LaserScanUpdate*new_pose.position.y + delta_LaserScanUpdate_now*self._last_pose.position.y)/(delta_last_LaserScanUpdate + delta_LaserScanUpdate_now)
            #用內插法估計方向，由於BURGER_MAX_ANG_VEL小於PI，方向在_last_pose及new_pose夾角較小的那邊
            (r, p, _last_direction_tmp) = euler_from_quaternion([ self._last_pose.orientation.x,  self._last_pose.orientation.y,  self._last_pose.orientation.z, self._last_pose.orientation.w])
            (r, p, ptn_dir_tmp) = euler_from_quaternion([ new_pose.orientation.x,  new_pose.orientation.y,  new_pose.orientation.z, new_pose.orientation.w])
            last_update_direction = (_last_direction_tmp + 2*pi)%(2*pi)#range of _last_direction_tmp is -pi ~ pi, update them to 0 ~ 2pi
            new_direction = (ptn_dir_tmp + 2*pi)%(2*pi)#range of ptn_dir_tmp is -pi ~ pi, update them to 0 ~ 2pi

            last_direction = (delta_last_LaserScanUpdate*new_direction + delta_LaserScanUpdate_now*last_update_direction)/(delta_last_LaserScanUpdate + delta_LaserScanUpdate_now)
            
            LOG.LogNumber('LastTime','At updateLaserScan',self._last_time.to_sec())
            LOG.LogNumber('UpdateTime','At updateLaserScan',self._update_time.to_sec())
            LOG.LogNumber('NowTime','At updateLaserScan',now_time.to_sec())

            LOG.LogNumber('LastUpdateDirection','At updateLaserScan',last_update_direction)
            LOG.LogNumber('LastDirection','At updateLaserScan',last_direction)
            LOG.LogNumber('NewDirection','At updateLaserScan',new_direction)         
            
            if(abs(last_update_direction - new_direction) > pi):
                last_direction = (last_direction + pi)%(2*pi)

            # #Convert 0 ~ 2pi to -pi ~ pi
            # if(last_direction > pi):
            #     last_direction = -(2*pi - last_direction)

            (self._last_pose.orientation.x,self._last_pose.orientation.y,self._last_pose.orientation.z,self._last_pose.orientation.w) = quaternion_from_euler(r,p,last_direction)
            
            self._update_time = now_time
            self._percise_last_pose = True
            # LOG.Log('UpdatePose','At updateLaserScan',self._last_pose)


        self._last_time = now_time
        self._pose = new_pose
        if(self._first_init_pose):
            self._last_pose = new_pose
            # self._pose = new_pose
            self._first_init_pose = False
            self._update_time = rospy.Time.now()
        else:
            # self._pose = new_pose
            self._update_time = rospy.Time.now()
            final_range = self._getLaserScan(new_pose)
            self.ranges = final_range[:]

        LOG.LogNumber('MaxRange','At updateLaserScan',max(self.ranges))

        LOG.Log('LaserScanSim','At updateLaserScan',self)
        # pass
            
    def updateTwist(self,twist):#:Twist):
        self._last_twist = twist

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
            # Get new_pose
            now_time = rospy.Time.now()
            delta_LaserScanUpdate_now = (now_time - self._update_time).to_sec()
            new_pose = Pose()

            (r, p, _last_direction_tmp) = euler_from_quaternion([ self._last_pose.orientation.x,  self._last_pose.orientation.y,  self._last_pose.orientation.z, self._last_pose.orientation.w])
            last_update_direction = (_last_direction_tmp + 2*pi)%(2*pi)#range of _last_direction_tmp is -pi ~ pi, update them to 0 ~ 2pi

            if(self._last_twist.angular.z == 0):
                new_pose.position.x = self._last_pose.position.x + self._last_twist.linear.x * sin( last_update_direction ) * delta_LaserScanUpdate_now
                new_pose.position.y = self._last_pose.position.y + self._last_twist.linear.x * cos( last_update_direction ) * delta_LaserScanUpdate_now 
                new_pose.position.z = self._last_pose.position.z 
            else:
                linear_div_angular_velocity = self._last_twist.linear.x / self._last_twist.angular.z
                angular_velocity_mul_time   = self._last_twist.angular.z * delta_LaserScanUpdate_now
                new_pose.position.x = self._last_pose.position.x + linear_div_angular_velocity * sin( last_update_direction + angular_velocity_mul_time )
                new_pose.position.y = self._last_pose.position.y - linear_div_angular_velocity * cos( last_update_direction + angular_velocity_mul_time ) + linear_div_angular_velocity
                new_pose.position.z = self._last_pose.position.z 

            #用內插法估計方向，由於BURGER_MAX_ANG_VEL小於PI，方向在_last_pose及new_pose夾角較小的那邊
            
            new_direction = last_update_direction + self._last_twist.angular.z * delta_LaserScanUpdate_now

            if(abs(last_update_direction - new_direction) > pi):
                new_direction = (new_direction + pi)%(2*pi)

            (new_pose.orientation.x,new_pose.orientation.y,new_pose.orientation.z,new_pose.orientation.w) = quaternion_from_euler(r,p,new_direction)

            # ranges = range(360)
            ranges = self._getLaserScan(new_pose)
            for i in range(360):
                if( LASERSCAN_UPDATE_MIN < msg.ranges[i] < LASERSCAN_UPDATE_MAX ):
                    ranges[i] = msg.ranges[i]
                # else:
                #     ranges[i] = self.ranges[i]
            self.ranges = tuple(ranges)
        # Update Last data
        self._last_pose = self._pose
        self._update_time = rospy.Time.now()
        self._percise_last_pose = False

        self._last_ranges = self.ranges[:]


        # LOG.Log('LaserScanSim','At updateLaserScan',self)
        
