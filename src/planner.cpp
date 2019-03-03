#include <iostream>
#include <queue>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include "planner.h"

Planner::Planner():
roadmap(std::vector<std::vector<int>>(11,std::vector<int>(11,0))),n_nodes(11){
}

void Planner::display_world_snapshot(){
	if(n_nodes<=0) return;
	int nrows=roadmap.size();
	int ncols=roadmap[0].size();
	for(int y=nrows-1;y>=0;y--){
		for(int x=0;x<ncols;x++){
			std::cout<<roadmap[y][x]<<" ";
		}
		std::cout<<std::endl;
	}
}

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

struct pq_node{
	std::pair<int,int> cell;
	double cost_to_reach;
	double key;
};

class pq_node_comp{
public:
	bool operator()(const pq_node& a,const pq_node& b){
		if(a.key==b.key) return a.cost_to_reach>b.cost_to_reach;
		return a.key>b.key;
	}
};

double heuristic_cost_to_goal_euclidian(const std::pair<int,int>& n,const std::pair<int,int>& goal){
	return 10*sqrt((n.first-goal.first)*(n.first-goal.first)+(n.second-goal.second)*(n.second-goal.second));
}

double heuristic_cost_to_goal_manhattan(const std::pair<int,int>& n,const std::pair<int,int>& goal){
	return 10*(abs(n.first-goal.first)+abs(n.second-goal.second));
}

bool goal_reached(const std::pair<int,int>& n,const std::pair<int,int>& goal){
	return n.first==goal.first && n.second==goal.second;
}

bool Planner::Astar_planner(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path){
	if(!path.empty()) path.clear();
	std::vector<std::vector<bool>> visited(n_nodes,std::vector<bool>(n_nodes,false));
	std::map<std::pair<int,int>,std::pair<int,int>> parent;
	std::priority_queue<pq_node,std::vector<pq_node>,pq_node_comp> pq;
	pq_node n;
	n.cell=start;
	n.cost_to_reach=0.0;
	n.key=0.0;
	pq.push(n);
	while(!pq.empty()){
		n=pq.top();
		pq.pop();
		int x=n.cell.first;
		int y=n.cell.second;
		visited[y][x]=true;
		if(goal_reached(n.cell,goal)){
			path.push_back(goal);
			roadmap[goal.second][goal.first]=agent_id;
			std::pair<int,int> prnt=parent[goal];
			while(parent.find(prnt)!=parent.end()){
				path.push_back(prnt);
				roadmap[prnt.second][prnt.first]=agent_id;
				prnt=parent[prnt];
			}
			path.push_back(prnt);
			roadmap[prnt.second][prnt.first]=agent_id;
			std::reverse(path.begin(),path.end());
			return true;
		}
		std::vector<std::pair<int,int>> neighbors=get_neighbors_from_roadmap(n.cell);
		for(auto neighbor: neighbors){
			x=neighbor.first;
			y=neighbor.second;
			if(!visited[y][x]){
				pq_node v;
				v.cell=neighbor;
				v.cost_to_reach=n.cost_to_reach+10;
				v.key=v.cost_to_reach+heuristic_cost_to_goal_euclidian(neighbor,goal);
				pq.push(v);
				parent[neighbor]=n.cell;
			}
		}
	}
	return false;
}

bool Planner::plan(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path){
	bool status=Astar_planner(agent_id,start,goal,path);
	return status;
}