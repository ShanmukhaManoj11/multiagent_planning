#include "ros/ros.h"
#include "planner.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"

class PlannerServer{
	Planner P;
	multiagent_planning::path_info* agent_pose;
public:
	PlannerServer(): P(){
		agent_pose=new multiagent_planning::path_info[2];
	}
	bool plan(multiagent_planning::plan_info::Request &request,
		multiagent_planning::plan_info::Response &response){
		ROS_INFO("request: serial id=%d, goal=%d,%d,%d",request.serial_id,request.goal[0],request.goal[1],request.goal[2]);
		std::vector<std::pair<int,int>> path;
		int agent_id=request.serial_id;
		std::pair<int,int> start({agent_pose[agent_id-1].x,agent_pose[agent_id-1].y});
		std::pair<int,int> goal({request.goal[0],request.goal[1]});
		bool status=P.plan(agent_id,start,goal,path);
		if(status){
			std::vector<multiagent_planning::path_info> res_path;
			P.create_path_from_xy_points(agent_id,path,request.goal[2],res_path);
			response.path=res_path;
		}
		P.display_world_snapshot();
		return status;
	}
	void agent_pose_callback1(const multiagent_planning::path_info& msg){
		agent_pose[0].x=msg.x;
		agent_pose[0].y=msg.y;
		agent_pose[0].theta=msg.theta;
	}
	void agent_pose_callback2(const multiagent_planning::path_info& msg){
		agent_pose[1].x=msg.x;
		agent_pose[1].y=msg.y;
		agent_pose[1].theta=msg.theta;
	}
};

int main(int argc,char** argv){
	ros::init(argc,argv,"planner_server");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	PlannerServer pServer;
	ros::ServiceServer planner_service=nh.advertiseService("/get_plan",&PlannerServer::plan,&pServer);
	ros::Subscriber agent_pose_sub1=nh.subscribe("/agent1/agent_feedback",1,&PlannerServer::agent_pose_callback1,&pServer);
	ros::Subscriber agent_pose_sub2=nh.subscribe("/agent2/agent_feedback",1,&PlannerServer::agent_pose_callback2,&pServer);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}