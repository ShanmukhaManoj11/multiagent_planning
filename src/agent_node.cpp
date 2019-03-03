#include "ros/ros.h"
#include "multiagent_planning/path_info.h"
#include "agent.h"

int main(int argc,char** argv){
	if(argc<5){
		ROS_INFO("missing args in launch file, need to specify serial_id, start_x, start_y and start_theta");
		return 1;
	}
	int serial_id=std::stoul(argv[1]);
	std::vector<int> start_pose({std::stoi(argv[2]),std::stoi(argv[3]),std::stoi(argv[4])});
	ros::init(argc,argv,"agent_node");
	Agent a(serial_id,start_pose);
	return 0;
}