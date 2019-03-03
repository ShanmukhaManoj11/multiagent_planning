#include "ros/ros.h"
#include "planner.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"

class PlannerServer{
	Planner P;
	void create_path_from_xy_points(const std::vector<std::pair<int,int>>& xy_path,const int& goal_theta,std::vector<multiagent_planning::path_info>& response_path){
		if(!response_path.empty()) response_path.clear();
		multiagent_planning::path_info msg;
		int n_xy_points=xy_path.size();
		std::pair<int,int> p=xy_path[0];
		std::pair<int,int> prev_p;
		msg.x=p.first;
		msg.y=p.second;
		msg.theta=0;
		msg.time=0;
		response_path.push_back(msg);
		prev_p=p;
		for(int i=1;i<n_xy_points;i++){
			p=xy_path[i];
			if(p.first<prev_p.first){
				msg.theta=180;
				msg.time=msg.time+10;
				response_path.push_back(msg);
			}
			else if(p.second>prev_p.second){
				msg.theta=90;
				msg.time=msg.time+10;
				response_path.push_back(msg);
			}
			else if(p.second<prev_p.second){
				msg.theta=270;
				msg.time=msg.time+10;
				response_path.push_back(msg);
			}
			msg.x=p.first;
			msg.y=p.second;
			msg.time=msg.time+10;
			response_path.push_back(msg);
			prev_p=p;
		}
		if(msg.theta!=goal_theta){
			msg.theta=goal_theta;
			msg.time=msg.time+10;
			response_path.push_back(msg);
		}
	}
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
			std::vector<multiagent_planning::path_info> res_path;
			create_path_from_xy_points(path,request.goal[2],res_path);
			response.path=res_path;
		}
		P.display_world_snapshot();
		return status;
	}
};

int main(int argc,char** argv){
	ros::init(argc,argv,"planner_server");
	ros::NodeHandle nh("~");
	PlannerServer pServer;
	ros::ServiceServer planner_service=nh.advertiseService("/get_plan",&PlannerServer::plan,&pServer);
	ros::spin();
	return 0;
}