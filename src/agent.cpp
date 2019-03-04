#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "multiagent_planning/plan_info.h"
#include "agent.h"

bool Agent::update_goal_callback(multiagent_planning::plan_info::Request &request,
	multiagent_planning::plan_info::Response &response){
	multiagent_planning::plan_info srv;
	if(request.serial_id!=agent_id){
		ROS_INFO("serial id mismatch");
		return false;
	}
	srv.request.serial_id=agent_id;
	srv.request.goal=request.goal;
	if(planner_client.call(srv)){
		response=srv.response;
		return true;
	}
	else{
		ROS_ERROR("Failed to call service");
		return false;
	}
}

Agent::Agent(const int& serial_id,const std::vector<int>& start_pose):
agent_id(serial_id), nh("~"){
	pose_publisher=nh.advertise<multiagent_planning::path_info>("/agent_feedback",1);
	planner_client=nh.serviceClient<multiagent_planning::plan_info>("/get_plan");
	update_goal_service=nh.advertiseService("/update_goal",&Agent::update_goal_callback,this);
	this->start_pose.x=start_pose[0];
	this->start_pose.y=start_pose[1];
	this->start_pose.theta=start_pose[2];
	ros::Rate rate(10);
	while(ros::ok()){
		pose_publisher.publish(this->start_pose);
		ros::spinOnce();
		rate.sleep();
	}
}