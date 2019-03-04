#include "ros/ros.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils.h"

int main(int argc,char** argv){
	if(argc<5){
		ROS_INFO("missing args in launch file, need to specify serial_id, goal_x, goal_y and goal_theta");
		return 1;
	}
	ros::init(argc,argv,"agent_update_goal_request_client");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	ros::Publisher path_publisher=nh.advertise<nav_msgs::Path>("/planned_path",1);
	ros::ServiceClient update_goal_client=nh.serviceClient<multiagent_planning::plan_info>("/update_goal");
	multiagent_planning::plan_info srv;
	srv.request.serial_id=std::stoul(argv[1]);
	srv.request.goal={(uint)std::stoul(argv[2]),(uint)std::stoul(argv[3]),(uint)std::stoul(argv[4])};
	for(int t=0;t<5;t++){
		rate.sleep();
	}
	if(update_goal_client.call(srv)){
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