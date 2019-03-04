#ifndef AGENT_H_
#define AGENT_H_

#include "ros/ros.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

void create_path_msg_to_publish(const multiagent_planning::plan_info::Response &response,
	nav_msgs::Path& msg){
	msg.header.stamp=ros::Time::now();
	msg.header.frame_id="/world";
	std::vector<geometry_msgs::PoseStamped> poses;
	geometry_msgs::PoseStamped pose_msg;
	for(auto p: response.path){
		pose_msg.pose.position.x=p.x;
		pose_msg.pose.position.y=p.y;
		if(p.theta==0){
			pose_msg.pose.orientation.z=0; //sin(th/2)
			pose_msg.pose.orientation.w=1; //cos(th/2)
		}
		else if(p.theta==90){
			pose_msg.pose.orientation.z=1.0/sqrt(2.0);
			pose_msg.pose.orientation.w=1.0/sqrt(2.0);
		}
		else if(p.theta==180){
			pose_msg.pose.orientation.z=1;
			pose_msg.pose.orientation.w=0;
		}
		else{ //p.theta=270
			pose_msg.pose.orientation.z=1.0/sqrt(2.0);
			pose_msg.pose.orientation.w=-1.0/sqrt(2.0);
		}
		poses.push_back(pose_msg);
	}
	msg.poses=poses;
}

#endif