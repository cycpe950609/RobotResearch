// Note :
//  How to get odom ? 
//	https://wiki.ros.org/evarobot_odometry/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20Odometry

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
//#include "nav_msgs/Odometry.h"
#include <cstdio>
#include <cmath>

#define BURGER_MAX_LIN_VEL  0.22
#define BURGER_MAX_ANG_VEL  2.84

// 誤差多少以內算是到達目的地
#define DistanceError 0.2
#define msg_odom msg

// Function define 
float EuclideanDistance( float px1, float py1, float px2, float py2 );
void Go2Goal( float px, float py );
void setTurtlebotTwistCallback(const geometry_msgs::Twist& msg);
void getPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
// ===============

ros::Subscriber odom_subscriber;

ros::Publisher  new_twist;	   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_stable_control");

    ros::NodeHandle nh;//Used to communicate with roscore

    odom_subscriber = nh.subscribe("odom",100,getPoseCallback);//TODO: subscribed node's name
    new_twist	    = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    ros::spin();

    printf("Node is startup");

    return 0;
}

//Callback when new Twist of Turtlebot updated
void setTurtlebotTwistCallback(const geometry_msgs::Twist& msg)
{
     
}

//Callback when Pose msg receive
void getPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    /*
    static int count = 0;
    count++;
    if(count == 10)
    {
	count = 0;
	ROS_INFO("===================================================");
    	ROS_INFO("Seq: [%d]", msg->header.seq);
    	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    }
    */
    geometry_msgs::Twist twist;

    twist.linear.x = BURGER_MAX_LIN_VEL;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    twist.angular.x = 0.0; 
    twist.angular.y = 0.0; 
    twist.angular.z = 0.0;

    new_twist.publish(twist);

}

float EuclideanDistance( float px1, float py1, float px2, float py2 )
{

    float dx = px2 - px1;
    float dy = py2 - py1;
    return sqrt( dx * dx + dy * dy );
}

