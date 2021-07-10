#!/usr/bin/env python2
# coding=utf-8
# We must add this line , otherwise python will raise error in runtime(default encoding is ASCII)
from geometry_msgs.msg import PoseStamped,Quaternion,Pose
from tf.transformations import quaternion_from_euler
import rospy

class DirectionArrow(object):
    def __init__(self,Name):
        super(DirectionArrow,self).__init__()
        self.direct_pub = rospy.Publisher('TEST', PoseStamped, queue_size=10)

    def updatePose(self,pose):
        assert type(pose) is Pose, 'Must be Pose'
        self._pose = pose

    def publish(self,direction):
        arrow = PoseStamped()
        arrow.header.frame_id = 'odom'
        arrow.header.stamp = rospy.Time.now()
        arrow.pose.position = self._pose.position

        (x,y,z,w) = quaternion_from_euler(0, 0, direction )
        arrow.pose.orientation = Quaternion(x,y,z,w)
        self.direct_pub.publish(arrow)