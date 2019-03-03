#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "agent.h"

Agent::Agent(const int& serial_id,const std::vector<int>& start_pose):
agent_id(serial_id), nh("~"){
	pose_publisher=nh.advertise<multiagent_planning::path_info>("/agent_feedback",1);
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