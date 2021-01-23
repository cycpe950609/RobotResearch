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

#include <actionlib/server/simple_action_server.h>
#include "motion_stable_control/Go2GoalAction.h"
#include <string>

#define BURGER_MAX_LIN_VEL  0.22
#define BURGER_MAX_ANG_VEL  2.84

// 誤差多少以內算是到達目的地
#define DistanceError 0.2
//#define msg_odom msg

// Function define 
double EuclideanDistance( geometry_msgs::Point ,geometry_msgs::Point );
void Go2Goal( float px, float py );
void setTurtlebotTwistCallback(const geometry_msgs::Twist& msg);
void getPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
void execCallback(const motion_stable_control::Go2GoalGoalConstPtr &goal);
// ===============
// Action Server
bool isGo2Goal = false;
// ros::NodeHandle *nh;
typedef actionlib::SimpleActionServer<motion_stable_control::Go2GoalAction> Server;
Server *g2gAS;

// 用作动作名称
std::string action_name;
// 声明用于发布的反馈及结果
motion_stable_control::Go2GoalFeedback feedback;
motion_stable_control::Go2GoalResult result;


// ===============
// Turtlebot3 properties
geometry_msgs::Point GoalOfRobot;
// ===============

ros::Subscriber odom_subscriber;
ros::Publisher  new_twist;	   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_stable_control");

    // nh = new ros::NodeHandle();//Used to communicate with roscore
    ros::NodeHandle nh;

    odom_subscriber = nh.subscribe("odom",100,getPoseCallback);//TODO: subscribed node's name
    new_twist	    = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    //Go2Goal Action
    g2gAS = new Server(nh,"Go2GoalAction",boost::bind(&execCallback, _1),false);//TODO : do we must have this ?
    g2gAS->start();

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
    
    static int count = 0;
    count++;
    if(count == 10)
    {
        count = 0;
        ROS_INFO("===================================================");
    	ROS_INFO("Seq: [%d]", msg->header.seq);
    	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
        ROS_INFO("===================================================");

    }
    
    if(isGo2Goal)
    {
        ROS_INFO("Is Going to Goal now");
        //Feedback
        feedback.Position = msg->pose.pose.position;
        g2gAS->publishFeedback(feedback);
        // Update Twist
        geometry_msgs::Twist twist;

        if(EuclideanDistance(GoalOfRobot , msg->pose.pose.position) <= DistanceError) // Arrive goal
        {
            ROS_INFO("Go2Goal Success");
            isGo2Goal = false;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;

            twist.angular.x = 0.0; 
            twist.angular.y = 0.0; 
            twist.angular.z = 0.0;
            g2gAS->setSucceeded();
        }
        else
        {
            ROS_INFO("Update Twist");
            twist.linear.x = BURGER_MAX_LIN_VEL;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;

            twist.angular.x = 0.0; 
            twist.angular.y = 0.0; 
            twist.angular.z = 0.0;
        }
    	new_twist.publish(twist);
    }

}

double EuclideanDistance( geometry_msgs::Point p1 , geometry_msgs::Point p2 )
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt( dx*dx + dy*dy + dz*dz );
}

void execCallback(const motion_stable_control::Go2GoalGoalConstPtr &goal)
{
    isGo2Goal = true;
    GoalOfRobot.x = goal->Goal.x;
    GoalOfRobot.y = goal->Goal.y;
    GoalOfRobot.z = goal->Goal.z;

    while(isGo2Goal);

    //Test
    // feedback.Position.x = 15; 
    // feedback.Position.y = 16; 
    // feedback.Position.z = 17; 
    // g2gAS->publishFeedback(feedback);

}
