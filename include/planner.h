#ifndef PLANNER_H_
#define PLANNER_H_

#include <iostream>
#include <vector>
#include "multiagent_planning/path_info.h"

class Planner{
private:
	std::vector<std::vector<std::pair<int,int>>> roadmap;
	int n_nodes;
	bool Astar_planner(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path);
public:
	Planner();
	void display_world_snapshot();
	std::vector<std::pair<int,int>> get_neighbors_from_roadmap(const std::pair<int,int>& n);
	bool plan(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path);
	void create_path_from_xy_points(const int& agent_id,const std::vector<std::pair<int,int>>& xy_path,const int& start_theta,const int& goal_theta,std::vector<multiagent_planning::path_info>& response_path);
};

#endif

