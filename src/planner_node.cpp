#include "ros/ros.h"
#include "planner.h"
#include "multiagent_planning/plan_info.h"
#include "multiagent_planning/path_info.h"

/*
Planner server dealing with mutliple agents - provide /get_plan service
Assumption: only 2 agents - only 2 subscribers are defined in the main that listen to the agents current position on the /agent<i>/agent_feedback topic for i=1,2
**TODO: modify to incorporate mutiple agents
*/
class PlannerServer{
	Planner P; //planner
	multiagent_planning::path_info* agent_pose; //array of agent poses of type multiagent_planning::path_info
public:
	/*
	constructor for the server
	*/
	PlannerServer(): P(){
		agent_pose=new multiagent_planning::path_info[2]; //allocating memory for 2 agents
	}
	/*
	callback associated with the /get_plan service
	*/
	bool plan(multiagent_planning::plan_info::Request &request,
		multiagent_planning::plan_info::Response &response){
		ROS_INFO("request: serial id=%d, goal=%d,%d,%d",request.serial_id,request.goal[0],request.goal[1],request.goal[2]);
		std::vector<multiagent_planning::path_info> path;
		int agent_id=request.serial_id;
		std::pair<int,int> start({agent_pose[agent_id-1].x,agent_pose[agent_id-1].y}); //start node xy position
		std::pair<int,int> goal({request.goal[0],request.goal[1]}); //goal node xy position
		bool status=P.plan(agent_id,start,agent_pose[agent_id-1].theta,goal,request.goal[2],path); //stores the sequence of (x,y,theta) points on the path to the goal node in the variable "path"
		if(status){
			response.path=path;
		}
		P.display_world_snapshot();
		return status;
	}
	/*
	callback for subscriber to the /agent1/agent_feedback topic to get the agent1's start position for planning
	*/
	void agent_pose_callback1(const multiagent_planning::path_info& msg){
		agent_pose[0].x=msg.x;
		agent_pose[0].y=msg.y;
		agent_pose[0].theta=msg.theta;
	}
	/*
	callback for subscriber to the /agent2/agent_feedback topic to get the agent2's start position for planning
	*/
	void agent_pose_callback2(const multiagent_planning::path_info& msg){
		agent_pose[1].x=msg.x;
		agent_pose[1].y=msg.y;
		agent_pose[1].theta=msg.theta;
	}
}; //Planner server

int main(int argc,char** argv){
	ros::init(argc,argv,"planner_server");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	PlannerServer pServer;
	ros::ServiceServer planner_service=nh.advertiseService("/get_plan",&PlannerServer::plan,&pServer); //advertising "/get_plan" service
	ros::Subscriber agent_pose_sub1=nh.subscribe("/agent1/agent_feedback",1,&PlannerServer::agent_pose_callback1,&pServer); //subscriber to /agent1/agent_feedback topic
	ros::Subscriber agent_pose_sub2=nh.subscribe("/agent2/agent_feedback",1,&PlannerServer::agent_pose_callback2,&pServer); //subscriber to /agent2/agent_feedback topic
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}