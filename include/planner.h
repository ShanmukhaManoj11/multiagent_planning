#ifndef PLANNER_H_
#define PLANNER_H_

#include <iostream>
#include <vector>

class Planner{
private:
	std::vector<std::vector<int>> roadmap;
	int n_nodes;
	bool Astar_planner(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path);
public:
	Planner();
	void display_world_snapshot();
	std::vector<std::pair<int,int>> get_neighbors_from_roadmap(const std::pair<int,int>& n);
	bool plan(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path);
};

#endif

