#include "ros/ros.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"

int main(int argc,char** argv){
	if(argc!=4){
		ROS_INFO("usage: planner_client serial_id goal_x goal_y");
		return 1;
	}
	ros::init(argc,argv,"planner_client");
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<multiagent_planning::plan_info>("/get_plan");
	multiagent_planning::plan_info srv;
	srv.request.serial_id=std::stoul(argv[1]);
	srv.request.goal={std::stoi(argv[2]),std::stoi(argv[3]),0};
	if(client.call(srv)){
		for(auto p: srv.response.path){
			ROS_INFO("(%d,%d,%d)",p.x,p.y,p.time);
		}
	}
	else{
		ROS_ERROR("Failed to call service");
		return 1;
	}
	return 0;
}