#ifndef AGENT_H_
#define AGENT_H_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "multiagent_planning/plan_info.h"

class Agent{
private:
	int agent_id;
	multiagent_planning::path_info start_pose;
	ros::NodeHandle nh;
	ros::Publisher pose_publisher;
	ros::ServiceServer update_goal_service;
	ros::ServiceClient planner_client;
	bool update_goal_callback(multiagent_planning::plan_info::Request &request,
		multiagent_planning::plan_info::Response &response);
public:
	Agent(const int& serial_id,const std::vector<int>& start_pose);
};

#endif

