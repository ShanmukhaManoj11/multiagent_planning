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
roadmap(std::vector<std::vector<std::pair<int,int>>>(11,std::vector<std::pair<int,int>>(11,std::pair<int,int>({0,0})))),n_nodes(11){
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
			std::cout<<roadmap[y][x].first<<","<<roadmap[y][x].second<<" ";
		}
		std::cout<<std::endl;
	}
}

/*
function to convert path with xy-points to sequence of (x,y,theta) points
*/
void Planner::create_path_from_xy_points(const int& agent_id,const std::vector<std::pair<int,int>>& xy_path,const int& start_theta,const int& goal_theta,std::vector<multiagent_planning::path_info>& response_path){
	if(!response_path.empty()) response_path.clear();
	multiagent_planning::path_info msg;
	int n_xy_points=xy_path.size();
	std::pair<int,int> p=xy_path[0];
	std::pair<int,int> prev_p;
	msg.x=p.first;
	msg.y=p.second;
	msg.theta=start_theta;
	msg.time=0;
	response_path.push_back(msg);
	roadmap[msg.y][msg.x].first=agent_id;
	roadmap[msg.y][msg.x].second=msg.time;
	prev_p=p;
	for(int i=1;i<n_xy_points;i++){
		p=xy_path[i];
		if(p.first>prev_p.first && msg.theta!=0){
			msg.theta=0;
			msg.time=msg.time+10;
			response_path.push_back(msg);
			roadmap[msg.y][msg.x].first=agent_id;
			roadmap[msg.y][msg.x].second=msg.time;
		}
		else if(p.first<prev_p.first && msg.theta!=180){
			msg.theta=180;
			msg.time=msg.time+10;
			response_path.push_back(msg);
			roadmap[msg.y][msg.x].first=agent_id;
			roadmap[msg.y][msg.x].second=msg.time;
		}
		else if(p.second>prev_p.second && msg.theta!=90){
			msg.theta=90;
			msg.time=msg.time+10;
			response_path.push_back(msg);
			roadmap[msg.y][msg.x].first=agent_id;
			roadmap[msg.y][msg.x].second=msg.time;
		}
		else if(p.second<prev_p.second && msg.theta!=270){
			msg.theta=270;
			msg.time=msg.time+10;
			response_path.push_back(msg);
			roadmap[msg.y][msg.x].first=agent_id;
			roadmap[msg.y][msg.x].second=msg.time;
		}
		msg.x=p.first;
		msg.y=p.second;
		msg.time=msg.time+10;
		response_path.push_back(msg);
		roadmap[msg.y][msg.x].first=agent_id;
		roadmap[msg.y][msg.x].second=msg.time;
		prev_p=p;
	}
	if(msg.theta!=goal_theta){
		msg.theta=goal_theta;
		msg.time=msg.time+10;
		response_path.push_back(msg);
		roadmap[msg.y][msg.x].first=agent_id;
		roadmap[msg.y][msg.x].second=msg.time;
	}
}

/*
given a (x,y) node, returns the list of neighbor nodes
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
	std::pair<int,int> cell;
	double cost_to_reach;
	double key;
	int time;
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
A* planning algorithm for finding shortest path to a goal node taking into account paths of other agents in the road map
Assumption: the circular agent can take only following steps,
	1. move to adjacent nodes connected by 4 edges, and each move on an edge costs 10 units and takes 10 seconds
	2. can rotate at the same xy position with moving and this move takes 10 seconds
	2. can wait on the current node without choosing moves 1 or 2
Brief: when a node is being processed, it's neighbor nodes on the roadmap are checked if in the next time there is any agent occupying them.
	if yes, then current agent waits until the node is no longer occupied
*/
bool Planner::Astar_planner(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path){
	if(!path.empty()) path.clear();
	std::vector<std::vector<bool>> visited(n_nodes,std::vector<bool>(n_nodes,false));
	std::map<std::vector<int>,std::vector<int>> parent;
	std::priority_queue<pq_node,std::vector<pq_node>,pq_node_comp> pq;
	pq_node n;
	n.cell=start;
	n.cost_to_reach=0.0;
	n.key=0.0;
	n.time=0;
	pq.push(n);
	while(!pq.empty()){
		n=pq.top();
		pq.pop();
		int x=n.cell.first;
		int y=n.cell.second;
		visited[y][x]=true;
		if(goal_reached(n.cell,goal)){
			path.push_back(goal);
			std::vector<int> prnt=parent[std::vector<int>({n.cell.first,n.cell.second,n.time})];
			while(parent.find(prnt)!=parent.end()){
				path.push_back(std::pair<int,int>({prnt[0],prnt[1]}));
				prnt=parent[prnt];
			}
			path.push_back(std::pair<int,int>({prnt[0],prnt[1]}));
			std::reverse(path.begin(),path.end());
			return true;
		}
		std::vector<std::pair<int,int>> neighbors=get_neighbors_from_roadmap(n.cell);
		for(auto neighbor: neighbors){
			x=neighbor.first;
			y=neighbor.second;
			if(!visited[y][x]){
				if(roadmap[y][x].second!=n.time+10){
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
					v.key=n.key;
					v.time=n.time+10;
					pq.push(v);
					parent[std::vector<int>({v.cell.first,v.cell.second,v.time})]=std::vector<int>({n.cell.first,n.cell.second,n.time});
				}
			}
		}
	}
	return false;
}

/*
wrapper for the A* planner
*/
bool Planner::plan(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path){
	bool status=Astar_planner(agent_id,start,goal,path);
	return status;
}