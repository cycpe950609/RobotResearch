#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <actionlib/client/simple_action_client.h>
#include "motion_stable_control/Go2GoalAction.h"
#include "nav_msgs/Odometry.h"

#include <math.h>

#include <mutex>
#include <condition_variable>

#include <string>
#include <functional>

using namespace visualization_msgs;
// ===============
// Turtlebot3 properties
geometry_msgs::Point GoalOfRobot;
// ===============

// void AddMarker(const tf::Vector3& position,std::string name ,std::string text,InteractiveMarkerControl interactive_mode,std::function<Marker(InteractiveMarker)> makeMarkerCB);
void AddMarker(const tf::Vector3& position,std::string name ,std::string text,uint8_t interactive_mode,std::function<Marker(InteractiveMarker&)> makeMarkerCB);
Marker makeBox( InteractiveMarker &msg ,double ratioX,double ratioY,double ratioZ ,double alpha);
Marker makeLabel( InteractiveMarker &msg , std::string );
double EuclideanDistance( geometry_msgs::Point p1 , geometry_msgs::Point p2 );
// GoalOfRobot
std::condition_variable goal_cv;
std::mutex goal_mutex;

bool isGo2Goal = false;
actionlib::SimpleActionClient<motion_stable_control::Go2GoalAction> *ac;

// Server
std::condition_variable server_cv;
std::mutex server_mutex;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

// 声明用于发布的反馈及结果
motion_stable_control::Go2GoalFeedback feedback;
motion_stable_control::Go2GoalResult result;

ros::Subscriber odom_subscriber;

interactive_markers::MenuHandler menu_handler;

void frameCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("frameCallback triggered");
  typedef std::string stg;
  tf::Vector3 position;
  auto pos = msg->pose.pose.position;
 
  position = tf::Vector3( pos.x - 0.65, pos.y, pos.z);//TODO
  // AddMarker(position,"turtlebot_marker",stg("Turtlebot3\n") + 
  //                                       stg("( ") + std::to_string(msg->pose.pose.position.x) + stg(" , ") + std::to_string(msg->pose.pose.position.y) + stg(" )\n") +
  //                                       stg("Distance : ") + std::to_string(EuclideanDistance(msg->pose.pose.position,GoalOfRobot )),
  //                                       InteractiveMarkerControl::NONE,makeLabel );
  
  goal_mutex.lock();

  auto txt =  stg("Turtlebot3\n") + 
              stg("( ") + std::to_string(msg->pose.pose.position.x) + stg(" , ") + std::to_string(msg->pose.pose.position.y) + stg(" )\n") +
              stg("Distance : ") + std::to_string(EuclideanDistance(msg->pose.pose.position,GoalOfRobot )) + stg("\n") + 
              stg( ( isGo2Goal ) ? "Going to Goal" : "At Goal :D" )
              ;
  goal_mutex.unlock();
  goal_cv.notify_all();

  AddMarker(position,"turtlebot_marker",txt,InteractiveMarkerControl::NONE,std::bind(makeBox,std::placeholders::_1,0.1,0.1,0.1,0.01) );

  server_mutex.lock();
  server->applyChanges();
  server_mutex.unlock();
  server_cv.notify_all();
  // static uint32_t counter = 0;

  // static tf::TransformBroadcaster br;

  // tf::Transform t;

  // ros::Time time = ros::Time::now();

  // t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
  // t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  // br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

  // t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  // t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  // br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

  // counter++;
}

Marker makeBox( InteractiveMarker &msg ,double ratioX,double ratioY,double ratioZ ,double alpha)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * ratioX;
  marker.scale.y = msg.scale * ratioY;
  marker.scale.z = msg.scale * ratioZ;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = alpha;

  return marker;
}

Marker makeLabel( InteractiveMarker &msg , std::string text )
{
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.x = msg.scale * 0.3;
  marker.scale.y = msg.scale * 0.3;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 1;
  marker.text = text;

  return marker;
}


// 当action完成后会调用次回调函数一次
void finishCallback(const actionlib::SimpleClientGoalState& state,const motion_stable_control::Go2GoalResultConstPtr& result)
{
    ROS_INFO("Yay! Arrive Goal");
    // ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void execCallback()
{
    ROS_INFO("Start moving to GOAL");
}

// 收到feedback后调用的回调函数
void feedbackCallback(const motion_stable_control::Go2GoalFeedbackConstPtr& feedback_)
{
    // ROS_INFO(" New Position : %lf , %lf , %lf ", feedback_->Position.x , feedback_->Position.y , feedback_->Position.z );
}



void updateMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  // ROS_INFO("Update Marker called");
}

double EuclideanDistance( geometry_msgs::Point p1 , geometry_msgs::Point p2 )
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt( dx*dx + dy*dy + dz*dz );
}


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  static bool moved = false;

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    {
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      moved = true;
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      // Update Position

      tf::Vector3 position;
      position = tf::Vector3( feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      typedef std::string stg;
      AddMarker(position,"goal_marker",stg("Goal Marker\n") + 
                                            stg("( ") + std::to_string(feedback->pose.position.x) + stg(" , ") + std::to_string(feedback->pose.position.y) + stg(" )\n"),// +
                                            //stg("Distance : ") + std::to_string(EuclideanDistance(msg->pose.pose.position,feedback->pose.position))
                                            InteractiveMarkerControl::MOVE_PLANE,std::bind(makeBox,std::placeholders::_1,0.3,0.3,0.1,0.5) );

      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    {
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;
    }

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      if(moved)
      {
        geometry_msgs::Pose pose = feedback->pose;
        // ROS_INFO("New Pose : " + std::to_string(pose.position.x) + " " << std::to_string(pose.position.y) );
        // ROS_INFO("TESTER");
        if(isGo2Goal)
        {
          ac->cancelGoal();
          // g_mutex.lock();
          isGo2Goal = false;
          // g_mutex.unlock();
          // g_cv.notify_all();
        }

        // send a goal to the action
        motion_stable_control::Go2GoalGoal goal;
        goal.Goal.x = pose.position.x;
        goal.Goal.y = pose.position.y;
        goal.Goal.z = 0;
          // ac.sendGoal(goal,&finishCallback,&execCallback,&feedbackCallback);

        goal_mutex.lock();
        GoalOfRobot.x = goal.Goal.x;
        GoalOfRobot.y = goal.Goal.y;
        GoalOfRobot.z = goal.Goal.z;
        goal_mutex.unlock();
        goal_cv.notify_all();
              
        ac->sendGoal(goal,
                    boost::bind(&finishCallback, _1, _2),
                    boost::bind(&execCallback),
                    boost::bind(&feedbackCallback, _1)
        );

        // g_mutex.lock();
        isGo2Goal = true;
        // g_mutex.unlock();
        // g_cv.notify_all();

        server_mutex.lock();
        server->setPose( feedback->marker_name, pose );
        server_mutex.unlock();
        server_cv.notify_all();

        //server->applyChanges();
        moved = false;
      }
      break;
    }
  }

  server_mutex.lock();
  server->applyChanges();
  server_mutex.unlock();
  server_cv.notify_all();
}

// void AddMarker(const tf::Vector3& position,std::string name ,std::string text,InteractiveMarkerControl interactive_mode,std::function<Marker(InteractiveMarker)> makeMarkerCB)
void AddMarker(const tf::Vector3& position,std::string name ,std::string text,uint8_t interactive_mode,std::function<Marker(InteractiveMarker&)> makeMarkerCB)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "odom";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = name;
  int_marker.description = text;

  InteractiveMarkerControl control;

  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.interaction_mode = interactive_mode;//InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeMarkerCB(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server_mutex.lock();
  server->insert(int_marker);
  server_mutex.unlock();
  server_cv.notify_all();
}
void makeTurtlebotDescription(const tf::Vector3& position)
{
  // AddMarker(position,"turtlebot_marker","",InteractiveMarkerControl::NONE,std::bind(makeLabel,std::placeholders::_1,"Turtlebot3"));
  AddMarker(position,"turtlebot_marker","",InteractiveMarkerControl::NONE,std::bind(makeBox,std::placeholders::_1,0,0,0,0.1));
  //server->setCallback("turtlebot_marker", &processFeedback);
}

void makeGoalMarker( const tf::Vector3& position)
{
  AddMarker(position,"goal_marker","Goal Marker\n",InteractiveMarkerControl::MOVE_PLANE,std::bind(makeBox,std::placeholders::_1,0.3,0.3,0.1,0.5));

  server_mutex.lock();
  server->setCallback("goal_marker", &processFeedback);
  server_mutex.unlock();
  server_cv.notify_all();

  // set different callback for POSE_UPDATE feedback
  // server->setCallback(int_marker.name, &updateMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_goal_gui_rviz");
  ros::NodeHandle n;

  // create the action client
  // true causes the client to spin its own thread
  ac = new actionlib::SimpleActionClient<motion_stable_control::Go2GoalAction>(n,"Go2GoalAction",false);
  // ac->waitForServer(); //will wait for infinite time

  ROS_INFO("Go2Goal Server Started");
  
  odom_subscriber = n.subscribe("odom",100,frameCallback);//NOTE: Use as a timer

  // // create a timer to update the published transforms
  // ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("set_goal_gui","",false) );

  ros::Duration(0.1).sleep();

  // menu_handler.insert( "Stop go to goal", &processFeedback );


  tf::Vector3 position;
 
  position = tf::Vector3( 0, 0, 0);
  makeGoalMarker( position );
  //make roboot's description, eg: position , distance
  makeTurtlebotDescription( position );
 
  server_mutex.lock();
  server->applyChanges();
  server_mutex.unlock();
  server_cv.notify_all();

  




  ros::spin();

  server.reset();

  delete ac;
}