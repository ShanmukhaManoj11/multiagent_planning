#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "multiagent_planning/plan_info.h"
#include "agent.h"

/*
callback associated with the update_goal_service
given a service request of type mutliagent_planning::plan_info::Request, update goal info from the corresponding message field
then call the planner client to generate the sequence of xy points corresponding to the shortest path to the updated goal node
*/
bool Agent::update_goal_callback(multiagent_planning::plan_info::Request &request,
	multiagent_planning::plan_info::Response &response){
	multiagent_planning::plan_info srv ;
	if(request.serial_id!=agent_id){ //check to see if the serial ids match
		ROS_INFO("serial id mismatch");
		return false;
	}
	srv.request.serial_id=agent_id; //composing the srv message
	srv.request.goal=request.goal;
	if(planner_client.call(srv)){ //calling the planner client to generate the path to the new goal node
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
	pose_publisher=nh.advertise<multiagent_planning::path_info>("/agent_feedback",1); //publish current pose on topic /agent_feedback
	planner_client=nh.serviceClient<multiagent_planning::plan_info>("/get_plan"); //client to the /get_plan service
	update_goal_service=nh.advertiseService("/update_goal",&Agent::update_goal_callback,this); //initialize /update_goal service
	this->start_pose.x=start_pose[0]; //populate agent start pose
	this->start_pose.y=start_pose[1];
	this->start_pose.theta=start_pose[2];
	ros::Rate rate(10); //ros spin rate
	while(ros::ok()){
		pose_publisher.publish(this->start_pose); //publish current pose on the /agent_feedback topic
		ros::spinOnce();
		rate.sleep();
	}
}