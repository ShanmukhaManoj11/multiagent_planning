#ifndef PLANNER_H_
#define PLANNER_H_

#include <iostream>
#include <vector>
#include "multiagent_planning/path_info.h"

class Planner{
private:
	std::vector<std::vector<std::pair<int,int>>> roadmap; //roadmap - 2d world graph with nodes and corresponding 4 connected edges
	//roadmap[j][i] = {agent_id,time when agent_id occupies the node (x=i, y=j)}
	int n_nodes; //n_nodesxn_nodes is the size of the roadmap
	bool Astar_planner(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path); //A* path planner to compute shortest distance path to a goal node
public:
	Planner(); //constructor for the planner class
	void display_world_snapshot(); //function to display a snapshot of the world
	std::vector<std::pair<int,int>> get_neighbors_from_roadmap(const std::pair<int,int>& n); //get corresponding 4 neighbor nodes(up, right, down and left) to a given node n
	bool plan(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path); //"plan" function that wraps the A* or any other plan algorithms finding shortest distance path to a goal node
	void create_path_from_xy_points(const int& agent_id,const std::vector<std::pair<int,int>>& xy_path,const int& start_theta,const int& goal_theta,std::vector<multiagent_planning::path_info>& response_path); //given sequence of xy-points on the planned path, convert these points to (x,y,theta)
}; //Planner class

#endif

