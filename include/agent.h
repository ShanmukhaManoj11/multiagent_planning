#ifndef AGENT_H_
#define AGENT_H_

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "multiagent_planning/plan_info.h"

class Agent{
private:
	int agent_id; //agent id or serial id unique to agent
	multiagent_planning::path_info start_pose; //intial position of the agent from which path is to be planned to a given goal pose
	ros::NodeHandle nh; //ros node handle
	ros::Publisher pose_publisher; //ros publisher publishing current position of the agent
	ros::ServiceServer update_goal_service; //ros service resposnible for updating goal information
	ros::ServiceClient planner_client; //ros client to the planner service to be called when the goal information is updated
	bool update_goal_callback(multiagent_planning::plan_info::Request &request,multiagent_planning::plan_info::Response &response); //callback function associated with the update_goal_service
public:
	Agent(const int& serial_id,const std::vector<int>& start_pose); //constructor of the agent object
}; //agent class

#endif

