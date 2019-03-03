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

int main(int argc,char** argv){
	if(argc<5){
		ROS_INFO("missing args in launch file, need to specify serial_id, goal_x, goal_y and goal_theta");
		return 1;
	}
	ros::init(argc,argv,"planner_client");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	ros::Publisher path_publisher=nh.advertise<nav_msgs::Path>("/planned_path",1);
	ros::ServiceClient client=nh.serviceClient<multiagent_planning::plan_info>("/get_plan");
	multiagent_planning::plan_info srv;
	srv.request.serial_id=std::stoul(argv[1]);
	srv.request.goal={(uint)std::stoul(argv[2]),(uint)std::stoul(argv[3]),(uint)std::stoul(argv[4])};
	if(client.call(srv)){
		for(auto p: srv.response.path){
			ROS_INFO("(%d,%d,%d,%d)",p.x,p.y,p.theta,p.time);
		}
		nav_msgs::Path path_msg;
		create_path_msg_to_publish(srv.response,path_msg);
		int msg_count=0;
		while(ros::ok() && msg_count<10){
			path_publisher.publish(path_msg);
			ros::spinOnce();
			rate.sleep();
			msg_count++;
		}
	}
	else{
		ROS_ERROR("Failed to call service");
		return 1;
	}
	return 0;
}