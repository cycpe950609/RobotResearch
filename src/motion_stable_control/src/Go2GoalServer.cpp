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
#include "Matrix.h"
#include <mutex>
#include <condition_variable>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf/tf.h>

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
std::condition_variable g_cv;
std::mutex g_mutex;
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

    ROS_INFO("Motion_Stable_Control Server Start");

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
    
    // static int count = 0;
    // count++;
    // if(count == 10)
    // {
    //     count = 0;
    //     ROS_INFO("===================================================");
    // 	ROS_INFO("Seq: [%d]", msg->header.seq);
    // 	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // 	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // 	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    //     ROS_INFO("===================================================");

    // }
    
    if(isGo2Goal)
    {
        // ROS_INFO("Is Going to Goal now");
        //Feedback
        feedback.Position = msg->pose.pose.position;
        g2gAS->publishFeedback(feedback);
        // Update Twist
        geometry_msgs::Twist twist;

        // ROS_INFO("Position %lf %lf",msg->pose.pose.position.x,msg->pose.pose.position.y);
        // ROS_INFO("DistanceError %lf " , EuclideanDistance(GoalOfRobot , msg->pose.pose.position) );

        if(EuclideanDistance(GoalOfRobot , msg->pose.pose.position) <= DistanceError) // Arrive goal
        {
            ROS_INFO("Go2Goal Success");
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;

            twist.angular.x = 0.0; 
            twist.angular.y = 0.0; 
            twist.angular.z = 0.0;
            g2gAS->setSucceeded();

            g_mutex.lock();
            isGo2Goal = false;
            g_mutex.unlock();
            g_cv.notify_all();

        }
        else
        {
            if (g2gAS->isPreemptRequested() || !ros::ok())
            {
                // Notify action cancellation
                ROS_INFO("motion_stable_control : Preempted");
                // Action cancellation and consider action as failure and save to variable
                g2gAS->setPreempted();
                g_mutex.lock();
                isGo2Goal = false;
                g_mutex.unlock();
                g_cv.notify_all();
                return;
            }
            //
            // ROS_INFO("TEST1"); 
            Matrix<double> robot_post_tmp   = { msg->pose.pose.position.x , msg->pose.pose.position.y };
            Matrix<double> robot_post(2,1);
            robot_post                      = robot_post_tmp.Transpose();
            // robot_post.PrintMatrix();
            // ROS_INFO("TEST2"); 
            Matrix<double> robot_dest_tmp   = {GoalOfRobot.x,GoalOfRobot.y};
            Matrix<double> robot_dest(2,1);
            robot_dest                      = robot_dest_tmp.Transpose();

            // ROS_INFO("TEST3"); 
            double epsilon = 0.1;
            Matrix<double> va               = {{1,0},{0,1/epsilon}};
            // ROS_INFO("TEST4"); 
            // Matrix<double> theta            = { { cos(-msg->pose.pose.orientation.z) , sin(-msg->pose.pose.orientation.z) },
            //                                     { -sin(-msg->pose.pose.orientation.z) , cos(-msg->pose.pose.orientation.z) }
            // };

            //Get Euler Angle from Quaternion
            tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

            // ROS_INFO("Quaternion %lf , %lf , %lf , %lf",msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            // ROS_INFO("EulerAngle %lf , %lf , %lf",roll , pitch , yaw);
            ROS_INFO("EulerYaw %lf " , yaw);

            // tf2::Quaternion quat_tf;
            // tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
            
            // tf2Scalar angle = quat_tf.getAngle();

            // Matrix<double> theta            = { { cos(msg->pose.pose.orientation.z) , sin(msg->pose.pose.orientation.z) },
            //                                     { -sin(msg->pose.pose.orientation.z) , cos(msg->pose.pose.orientation.z) }
            // };
            // Matrix<double> theta            = { { cos(-yaw) , sin(-yaw) },
            //                                     { -sin(-yaw) , cos(-yaw) }
            // };
            Matrix<double> theta            = { { cos(yaw) , sin(yaw) },
                                                { -sin(yaw) , cos(yaw) }
            };
            
            // ROS_INFO("TEST5"); 
            Matrix<double> e_g2g(2,1);
            // ROS_INFO("TEST5a"); 
            e_g2g                       = robot_dest - robot_post;
            // e_g2g                       = robot_post - robot_dest;
            // ROS_INFO("TEST5b"); 
            e_g2g                       = e_g2g + 0.001;
            // ROS_INFO("TEST5c"); 
            
            // e_g2g.PrintMatrix();

            const double alpha = 10.0;
            // double k = BURGER_MAX_LIN_VEL*( 1 - (exp( -alpha*e_g2g.normf()*e_g2g.normf() ) ) / (e_g2g.normf()) );
            double k = BURGER_MAX_LIN_VEL*( (1 - exp( -alpha*e_g2g.normf()*e_g2g.normf() ))  / (e_g2g.normf()) );

            // ROS_INFO("TEST6"); 
            Matrix<double> u_g2g(2,1);
            u_g2g                       = e_g2g * k;

            // u_g2g.PrintMatrix();

            // ROS_INFO("TEST7"); 
            Matrix<double> rlt(2,1);
            Matrix<double> rlt2(2,2);
            rlt2                        = va.dot(theta);
            // ROS_INFO("TEST8"); 
            rlt                         = va.dot(theta).dot(u_g2g); 
            // ROS_INFO("TEST9"); 
            // ROS_INFO("Print VA ===");
            // va.PrintMatrix();
            // ROS_INFO("Print theta ===");
            // theta.PrintMatrix();
            // ROS_INFO("Print u_g2g ===");
            // u_g2g.PrintMatrix();
            // ROS_INFO("Print rlt2 ===");
            // rlt2.PrintMatrix();
            // ROS_INFO("Print rlt ===");
            // rlt.PrintMatrix();
            // ROS_INFO("================");


            twist.linear.x = rlt[0][0];
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
    
            twist.angular.x = 0.0; 
            twist.angular.y = 0.0; 
            twist.angular.z = rlt[1][0];
        }
        new_twist.publish(twist);
        // ROS_INFO("Update Twist");

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

    ROS_INFO("Get goal ( %lf , %lf , %lf )",goal->Goal.x , goal->Goal.y , goal->Goal.z);

    std::unique_lock<std::mutex> lock(g_mutex);
    while(isGo2Goal == true)
    {
        g_cv.wait(lock);
    }

    ROS_INFO("Go to Goal Success");
    //Test
    // feedback.Position.x = 15; 
    // feedback.Position.y = 16; 
    // feedback.Position.z = 17; 
    // g2gAS->publishFeedback(feedback);

}
