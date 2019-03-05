#include <iostream>
#include <queue>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include "planner.h"
#include "multiagent_planning/path_info.h"

/*
Planner constructor
*/
Planner::Planner():
n_nodes(11){
	//initialize roadmap data structure with each (x,y) cell initialized to the cell_info {0,0,0}
	for(int y=0;y<n_nodes;y++){
		std::vector<cell_info> roadmap_row;
		for(int x=0;x<n_nodes;x++){
			cell_info ci;
			ci.agent_id=0;
			ci.time_of_arrival=0;
			ci.time_of_occupancy=0;
			roadmap_row.push_back(ci);
		}
		roadmap.push_back(roadmap_row);
	}
}

/*
utility function to display currrent world snapshot
displays 2d grid where each entry is filled with a pair - (agent_id, time at which that agent occupies the node) 
*/
void Planner::display_world_snapshot(){
	if(n_nodes<=0) return;
	int nrows=roadmap.size();
	int ncols=roadmap[0].size();
	for(int y=nrows-1;y>=0;y--){
		for(int x=0;x<ncols;x++){
			std::cout<<roadmap[y][x].agent_id<<","<<roadmap[y][x].time_of_arrival<<","<<roadmap[y][x].time_of_occupancy<<" ";
		}
		std::cout<<std::endl;
	}
}

/*
given a (x,y) node, returns the list of (4 connected) neighbor nodes
*/
std::vector<std::pair<int,int>> Planner::get_neighbors_from_roadmap(const std::pair<int,int>& n){
	std::vector<std::pair<int,int>> neighbors;
	int x=n.first;
	int y=n.second;
	if(x<0 || x>=n_nodes || y<0 || y>=n_nodes) return neighbors;
	if(x-1>=0) neighbors.push_back(std::pair<int,int>({x-1,y}));
	if(x+1<n_nodes) neighbors.push_back(std::pair<int,int>({x+1,y}));
	if(y-1>=0) neighbors.push_back(std::pair<int,int>({x,y-1}));
	if(y+1<n_nodes) neighbors.push_back(std::pair<int,int>({x,y+1}));
	return neighbors;
}

/*
utility data structure to fill nodes in a priority queue
*/
struct pq_node{
	std::pair<int,int> cell; //(x,y) location
	double cost_to_reach; //cost to reach from start node
	double key; //cost to reach from start node + heuristic cost to reach goal
	int time; //time at which the node would be reached if it were in the plan
};

/*
custom comparision object for the pq_node structre to construct min-heap from stl priority queue data structure
*/
class pq_node_comp{
public:
	bool operator()(const pq_node& a,const pq_node& b){
		if(a.key==b.key) return a.cost_to_reach>b.cost_to_reach;
		return a.key>b.key;
	}
};

/*
utility function for estimating heurristic cost to goal using euclidian distance
*/
double heuristic_cost_to_goal_euclidian(const std::pair<int,int>& n,const std::pair<int,int>& goal){
	return 10*sqrt((n.first-goal.first)*(n.first-goal.first)+(n.second-goal.second)*(n.second-goal.second));
}

/*
utility function for estimating heuristic cost to goal using manhattan distance
*/
double heuristic_cost_to_goal_manhattan(const std::pair<int,int>& n,const std::pair<int,int>& goal){
	return 10*(abs(n.first-goal.first)+abs(n.second-goal.second));
}

/*
utility finction to check if the goal node is reached
*/
bool goal_reached(const std::pair<int,int>& n,const std::pair<int,int>& goal){
	return n.first==goal.first && n.second==goal.second;
}

/*
utility function to check if a node in current snapshot of the roadmap is a goal to any agent
*/
bool Planner::is_goal_to_an_agent(const std::pair<int,int>& node){
	int x=node.first;
	int y=node.second;
	if(roadmap[y][x].agent_id==0) return false; 
	/*
	if atleast one neighbor to the node with same agent_id will be reached at a later point in time than the current node then it is not a goal node
	*/
	int n=0;
	std::vector<std::pair<int,int>> neighbors=get_neighbors_from_roadmap(node);
	for(auto neighbor: neighbors){ 
		int x1=neighbor.first;
		int y1=neighbor.second;
		if(roadmap[y1][x1].agent_id==roadmap[y][x].agent_id && roadmap[y1][x1].time_of_arrival>roadmap[y][x].time_of_arrival){
			n++;
		}
	}
	return (n==0); 
}

/*
utility function to check if the given goal node is a valid node
A node can be a valid goal node, if it not a goal to any agent at the current time snapshot of the roadmap and it is in the bounds of the world
*/
bool Planner::is_valid_goal(const std::pair<int,int>& goal){
	int x=goal.first;
	int y=goal.second;
	if(x<0 || x>=n_nodes || y<0 || y>=n_nodes) return false; //out of bounds check
	return !(is_goal_to_an_agent(goal));
}

/*
returns rotation needed to move from src (x1,y1) to dst (x2,y2)
	a. TH=0 deg if x2==x1+1 && y2==y1 (else)
	b. TH=90 deg if x2==x1 && y2==y1+1 (else)
	c. TH=180 deg if x2==x1-1 && y2==y1 (else)
	d. TH=270 deg if x2==x1 and y2==y1-1
*/
int dst_theta_from_src(const std::pair<int,int>& src,const int& src_theta,const std::pair<int,int>& dst){
	int x1=src.first, y1=src.second, x2=dst.first, y2=dst.second;
	if(x2>x1 && y2==y1) return 0;
	else if(x2<x1 && y2==y1) return 180;
	else if(y2>y1 && x2==x1) return 90;
	else if(y2<y1 && x2==x1) return 270;
	else return src_theta;
}

void Planner::build_path_update_roadmap(const int& agent_id,const std::pair<int,int>& goal,const int& goal_theta,const int& goal_time,const int& start_theta,
	std::map<std::vector<int>,std::vector<int>>& parent,std::vector<multiagent_planning::path_info>& path){
	if(!path.empty()) path.clear();
	//build path
	multiagent_planning::path_info path_point;
	path_point.x=goal.first;
	path_point.y=goal.second;
	path_point.theta=goal_theta;
	path_point.time=goal_time;
	path.push_back(path_point);
	std::vector<int> prnt=parent[std::vector<int>({goal.first,goal.second,goal_time})];
	while(parent.find(prnt)!=parent.end()){
		path_point.x=prnt[0];
		path_point.y=prnt[1];
		path_point.theta=0;
		path_point.time=prnt[2];
		path.push_back(path_point);
		prnt=parent[prnt];
	}
	path_point.x=prnt[0];
	path_point.y=prnt[1];
	path_point.theta=start_theta;
	path_point.time=prnt[2];
	path.push_back(path_point);
	std::reverse(path.begin(),path.end());
	//update roadmap cell with path information
	int n_path_points=path.size();
	roadmap[path[0].y][path[0].x].agent_id=agent_id;
	roadmap[path[0].y][path[0].x].time_of_arrival=path[0].time;
	roadmap[path[0].y][path[0].x].time_of_occupancy=0;
	for(int i=1;i<n_path_points-1;i++){
		std::pair<int,int> src({path[i-1].x,path[i-1].y});
		std::pair<int,int> dst({path[i].x,path[i].y});
		path[i].theta=dst_theta_from_src(src,path[i-1].theta,dst);
		if(roadmap[path[i].y][path[i].x].agent_id==agent_id){
			roadmap[path[i].y][path[i].x].time_of_occupancy+=10;
		}
		else{
			roadmap[path[i].y][path[i].x].agent_id=agent_id;
			roadmap[path[i].y][path[i].x].time_of_arrival=path[i].time;
			roadmap[path[i].y][path[i].x].time_of_occupancy=0;
		}
	}
	roadmap[path[n_path_points-1].y][path[n_path_points-1].x].agent_id=agent_id;
	roadmap[path[n_path_points-1].y][path[n_path_points-1].x].time_of_arrival=path[n_path_points-1].time;
	roadmap[path[n_path_points-1].y][path[n_path_points-1].x].time_of_occupancy=1000;
}

/*
A* planning algorithm for finding shortest path to a goal node taking into account paths of other agents in the road map
Assumption: the circular agent can take only following steps,
	1. move to adjacent nodes connected by 4 edges, and each move on an edge costs 10 units and takes 10 seconds
	2. can rotate at the same xy position with moving and this move takes 10 seconds
	2. can wait on the current node without choosing moves 1 or 2
Brief: when a node is being processed, it's neighbor nodes on the roadmap are checked if in the next time there is any agent occupying them.
	if yes, then current agent waits until the node is no longer occupied
*/
bool Planner::Astar_planner(const int& agent_id,const std::pair<int,int>& start,const int& start_theta,const std::pair<int,int>& goal,const int& goal_theta,std::vector<multiagent_planning::path_info>& path){
	if(!path.empty()) path.clear();
	if(!is_valid_goal(goal)){ //check if given goal point is valid
		std::cout<<"Not a valid goal point"<<std::endl;
		return true;
	}
	std::vector<std::vector<bool>> visited(n_nodes,std::vector<bool>(n_nodes,false)); 
	std::map<std::vector<int>,std::vector<int>> parent; //map data structure to store parents to the potential closed nodes
	std::priority_queue<pq_node,std::vector<pq_node>,pq_node_comp> pq; //priority queue (min heap) for the open nodes
	pq_node n;
	n.cell=start;
	n.cost_to_reach=0.0;
	n.key=0.0;
	n.time=0;
	pq.push(n); 
	while(!pq.empty() && n.time<1000){ //setting a maximum bound to time to find path (1000) as a sanity check - return false if path is not found in less than 1000 time units
		n=pq.top(); //extract top node with the minimum key value
		pq.pop();
		int x=n.cell.first;
		int y=n.cell.second;
		visited[y][x]=true;
		if(goal_reached(n.cell,goal)){ //if goal reached build path backwards from the goal node using the map containing parent info for each node
			build_path_update_roadmap(agent_id,n.cell,goal_theta,n.time,start_theta,parent,path);
			return true;
		}
		std::vector<std::pair<int,int>> neighbors=get_neighbors_from_roadmap(n.cell);
		for(auto neighbor: neighbors){ //for each neighbor to the extracted node
			x=neighbor.first;
			y=neighbor.second;
			if(!visited[y][x]){
				/*
				if,
					1. the neighbor cell (x,y) will not be occupied by any agent in future (i.e., agent_id of the cell x,y in the roadmap is 0) OR
					2. current node time + 10 (time to reach the neighbor node from current node) is < time of arrival of any agent to that cell at (x,y) OR
						current node time + 10 > time_of_arrival + time of occupancy of other agent on that cell (x,y) (i.e. time at which that agent leaves the cell)
				then, there is no threat for current agent to move to the node
				else, the agent shall wait at the current node, and a wait cost of 5 units is added to the node's key value to be pushed to the queue
				*/
				if(roadmap[y][x].agent_id==0 || (n.time+10<roadmap[y][x].time_of_arrival || n.time+10>roadmap[y][x].time_of_arrival+roadmap[y][x].time_of_occupancy)){
					pq_node v;
					v.cell=neighbor;
					v.cost_to_reach=n.cost_to_reach+10;
					v.key=v.cost_to_reach+heuristic_cost_to_goal_euclidian(neighbor,goal);
					v.time=n.time+10;
					pq.push(v);
					parent[std::vector<int>({v.cell.first,v.cell.second,v.time})]=std::vector<int>({n.cell.first,n.cell.second,n.time});
				}
				else{
					pq_node v;
					v.cell=n.cell;
					v.cost_to_reach=n.cost_to_reach;
					v.key=n.key+5; //add waiting cost (5 units)
					v.time=n.time+10;
					pq.push(v);
					parent[std::vector<int>({v.cell.first,v.cell.second,v.time})]=std::vector<int>({n.cell.first,n.cell.second,n.time});
				}
			}
		}
	}
	std::cout<<"Path to goal not found!!"<<std::endl;
	return false;
}

/*
wrapper for the A* planner
*/
bool Planner::plan(const int& agent_id,const std::pair<int,int>& start,const int& start_theta,const std::pair<int,int>& goal,const int& goal_theta,std::vector<multiagent_planning::path_info>& path){
	bool status=Astar_planner(agent_id,start,start_theta,goal,goal_theta,path);
	return status;
}