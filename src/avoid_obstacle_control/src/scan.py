#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler,euler_from_quaternion

from math import sqrt,sin,cos

DangerDistance = 2

rospy.init_node('scan_values')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

direction = 0

def EuclideanDistance(vec): #NOTE: vec must be a 2 element list 
    return sqrt( vec[0]**2 + vec[1]**2 )

def DistanceCallback(msg):
    print ( "First Value %f , Last Value %f" )%( msg.ranges[0], msg.ranges[359] )
    front_distance = msg.ranges[0]
    if(front_distance > msg.range_min and front_distance < DangerDistance):
        # Avoid Obstacle
        c = 1
        e = 0
        k = (1/front_distance)*(c/(front_distance ** 2 + e))
        u = [ - k * front_distance * cos(direction) , - k * front_distance * sin(direction) ] # k
        print u
        twist = Twist()
        twist.linear.x = u[0]
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = u[1]

        pub.publish(twist)

def OdometryCallback(msg):
    # print msg
    # TODO: Convert Quata.. to EulerAngle
    (r, p, y) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    direction = y
    print "Direction : " 
    print direction

dis_sub = rospy.Subscriber('/scan', LaserScan, DistanceCallback)
odo_sub = rospy.Subscriber('/odom',Odometry,OdometryCallback)
rospy.spin()
