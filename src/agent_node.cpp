#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "agent.h"

/*
agent node provides the update goal service
agent node is a client to the get plan service which generates path to the updated goal node
*/
int main(int argc,char** argv){
	if(argc<5){ //check if the correct number of arguments are provided
		ROS_INFO("missing args in launch file, need to specify serial_id, start_x, start_y and start_theta");
		return 1;
	}
	int serial_id=std::stoul(argv[1]);
	std::vector<int> start_pose({std::stoi(argv[2]),std::stoi(argv[3]),std::stoi(argv[4])});
	ros::init(argc,argv,"agent_node");
	Agent a(serial_id,start_pose); //start the agent
	return 0;
}