#ifndef MOTION_STABLE_CONTROL_GO2GOAL
#define MOTION_STABLE_CONTROL_GO2GOAL

#include <ros/ros.h>	
#include <actionlib/server/simple_action_server.h>
#include "motion_stable_control/Go2GoalAction.h"
#include <string>

typedef struct Point
{
     double x;
     double y;
     double z;
}Point;

class Go2GoalAction
{
    public:
	Go2GoalAction(std::string name): g2gAS(nh, name, boost::bind(&Go2GoalAction::execCallback, this, _1), false), action_name(name)
	{
	    g2gAS.start();
	}
	~Go2GoalAction();
	Point Goal() const;

    private:
	Point goal;

    protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<motion_stable_control::Go2GoalAction> g2gAS;

	// 用作动作名称
	std::string action_name;
	// 声明用于发布的反馈及结果
	motion_stable_control::Go2GoalFeedback feedback;
	motion_stable_control::Go2GoalResult result;

	// CallBack
	void execCallback(const motion_stable_control::Go2GoalGoalConstPtr &goal);


};


#endif
