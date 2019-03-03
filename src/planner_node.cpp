#include "ros/ros.h"
#include "planner.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"

class PlannerServer{
	Planner P;
public:
	PlannerServer(): P(){};
	bool plan(multiagent_planning::plan_info::Request &request,
		multiagent_planning::plan_info::Response &response){
		ROS_INFO("request: serial id=%d, goal=%d,%d,%d",request.serial_id,request.goal[0],request.goal[1],request.goal[2]);
		std::vector<std::pair<int,int>> path;
		int agent_id=request.serial_id;
		std::pair<int,int> start({0,0});
		std::pair<int,int> goal({request.goal[0],request.goal[1]});
		bool status=P.plan(agent_id,start,goal,path);
		if(status){
			int time=0;
			std::vector<multiagent_planning::path_info> res_path;
			multiagent_planning::path_info msg;
			for(auto p: path){
				msg.x=p.first;
				msg.y=p.second;
				msg.time=time;
				res_path.push_back(msg);
				time=time+10;
			}
			response.path=res_path;
		}
		P.display_world_snapshot();
		return status;
	}
};


int main(int argc,char** argv){
	ros::init(argc,argv,"planner_server");
	ros::NodeHandle nh;
	PlannerServer pServer;

	ros::ServiceServer planner_service=nh.advertiseService("/get_plan",&PlannerServer::plan,&pServer);
	ros::spin();

	return 0;
	/*Planner P;
	std::vector<std::pair<int,int>> path1;
	std::vector<std::pair<int,int>> path2;
	int agent_id=1;
	std::pair<int,int> start({2,0});
	std::pair<int,int> goal({2,5});
	bool status=P.plan(agent_id,start,goal,path1);
	agent_id=2;
	start=std::pair<int,int>({0,3});
	goal=std::pair<int,int>({6,3});
	status=P.plan(agent_id,start,goal,path2);
	P.display_world_snapshot();
	for(auto p: path1){
		std::cout<<"("<<p.first<<","<<p.second<<") ";
	}
	std::cout<<std::endl;
	for(auto p: path2){
		std::cout<<"("<<p.first<<","<<p.second<<") ";
	}
	std::cout<<std::endl;*/
}