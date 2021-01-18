#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <cstdio>

#define msg_odom msg
//Callback when Pose msg receive
void getPoseCallback( const geometry_msgs::Pose::ConstPtr& msg )
{
    printf("Receive position	    : ( %lf , %lf , %lf )", msg_odom->position.x, msg_odom->position.y, msg_odom->position.z);
    printf("Receive orientation	    : ( %lf , %lf , %lf , %lf )", msg_odom->orientation.x, msg_odom->orientation.y, msg_odom->orientation.z ,msg_odom->orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_stable_control");

    ros::NodeHandle nh;//Used to communicate with roscore

    ros::Subscriber odom_subscriber = nh.subscribe("odom",100,getPoseCallback);

    ros::spin();

    return 0;
}

