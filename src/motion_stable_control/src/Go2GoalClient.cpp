#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
#include "motion_stable_control/Go2GoalAction.h"

// 当action完成后会调用次回调函数一次
void finishCallback(const actionlib::SimpleClientGoalState& state,const motion_stable_control::Go2GoalResultConstPtr& result)
{
    ROS_INFO("Yay! Arrive Goal");
    ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void execCallback()
{
    ROS_INFO("Start moving to GOAL");
}

// 收到feedback后调用的回调函数
void feedbackCallback(const motion_stable_control::Go2GoalFeedbackConstPtr& feedback_)
{
    ROS_INFO(" New Position : %lf , %lf , %lf ", feedback_->Position.x , feedback_->Position.y , feedback_->Position.z );
}



int main(int argc, char **argv)
{
    //TODO : Finish Client Test    
    ros::init(argc, argv, "test_go2goal");

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<motion_stable_control::Go2GoalAction> ac("Go2GoalAction",true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    motion_stable_control::Go2GoalGoal goal;
    goal.Goal.x = 95;
    goal.Goal.y = 0;
    goal.Goal.z = 0;
    // ac.sendGoal(goal,&finishCallback,&execCallback,&feedbackCallback);
    
    ac.sendGoal(goal,
                boost::bind(&finishCallback, _1, _2),
                boost::bind(&execCallback),
                boost::bind(&feedbackCallback, _1)
    );

    ros::spin();

    return 0;
}


// const boost::shared_ptr<const motion_stable_control::Go2GoalActionFeedback_<std::allocator<void> > >&
// const boost::shared_ptr<const motion_stable_control::Go2GoalFeedback_<std::allocator<void> > >