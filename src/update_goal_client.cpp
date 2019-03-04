#include "ros/ros.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils.h"

/*
update goal client to the /update_goal server - sending agent serial id and new goal position as request to generate the plan as response
*/
int main(int argc,char** argv){
	if(argc<5){ //check for correct arguments set
		ROS_INFO("missing args in launch file, need to specify serial_id, goal_x, goal_y and goal_theta");
		return 1;
	}
	ros::init(argc,argv,"agent_update_goal_request_client");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	ros::Publisher path_publisher=nh.advertise<nav_msgs::Path>("/planned_path",1); //publish planned path as nav_msgs::Path to visualize in rviz 
	ros::ServiceClient update_goal_client=nh.serviceClient<multiagent_planning::plan_info>("/update_goal"); //initialize client to the /update_goal service
	multiagent_planning::plan_info srv; //build the service request
	srv.request.serial_id=std::stoul(argv[1]);
	srv.request.goal={(uint)std::stoul(argv[2]),(uint)std::stoul(argv[3]),(uint)std::stoul(argv[4])};
	for(int t=0;t<5;t++){ //wait 5 time steps to assure the stat of required nodes from launch file
		rate.sleep();
	}
	if(update_goal_client.call(srv)){ //call to the /update_goal service
		for(auto p: srv.response.path){
			ROS_INFO("(%d,%d,%d,%d)",p.x,p.y,p.theta,p.time);
		}
		nav_msgs::Path path_msg;
		create_path_msg_to_publish(srv.response,path_msg); //convert sequence of xy-points in the response path to nav_msgs::Path message to be published
		int msg_count=0;
		while(ros::ok() && msg_count<10){ //send nav_msgs::Path messages for 10 time steps to make sure rviz receives those messages
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