#ifndef PLANNER_H_
#define PLANNER_H_

#include <iostream>
#include <vector>
#include "multiagent_planning/path_info.h"

/*
cell_info data structure whic corrseponds to the individual cell information of the roadmap data structure (details given in the Planner class)
Assumptions: agent_id or serial ids start from 1 and 0 id denotes not filled by any agent
cell_info data structure contains following fields,
	1. agent_id - if 0 then the correspinding cell at (x,y) location in the 2D roadmap will not be a part of path of an agent
	2. time_of_arrival - time at which agent with serial id agent_id arrives at the corresponding node
	3. time_of_occupancy - time for which the agent occupies the cell after it arrives. - this field is set to a high number (say 1000) if the cell is the goal location of the corresponding agent
*/
struct cell_info{
	int agent_id;
	int time_of_arrival;
	int time_of_occupancy;
};

/*
definition of Planner class to initialize a planner object. This object stores the current information of the world in the variable roadmap
which is a 2 dimensional array of cells implemented as vector of vectors with each cell represented by the cell_info data struct (defined above)
Main function: plan - creates the shortest path to the given goal location

Planner class implements A* on 2D (x,y) grid and generates sequence of (x,y,theta) points as a path
The implementation is based on following assumptions:
	1. agent only moves along the edges from a node to one of it's 4 connected neighbors OR waits at the same node 
	2. in-cell rotations are "not" considered as seperate moves
	given src pose (x1,y1,theta1) and goal cell (x2,y2), robot first rotates in-cell to a heading angle (theta) that is consistent with its direct movement to cell (x2,y2)
	this move to the next node is assumed to have cost 10
	a direct move to neighbor cell can be the following sequence: "in-cell rotation -> move to neighbor cell -> in-cell rotation [optional]"
	3. priority for generating plans is first-come-first, i.e. given 2 agents agent1 and agent2, if agent2 first call the /update_goal service, then service should consider collisions with
	agent2s path while creating the plan for agent1 whose request comes later
	4. to avoid potential collisions, agents can wait in a cell - but waiting cost 5 units (posing cost on wait helps in avoiding forever wait scenarios like high priority agents goal is on the potential shortest path of other low prioirty agent)
*/
class Planner{
private:
	std::vector<std::vector<cell_info>> roadmap; //roadmap - 2d world graph with nodes and corresponding 4 connected edges
	//roadmap[j][i] = {agent_id,time when agent_id occupies the node (x=i, y=j)}
	int n_nodes; //n_nodesxn_nodes is the size of the roadmap
	bool is_goal_to_an_agent(const std::pair<int,int>& node); //function to check if a given cell in the current snapshot of the roadmap is a goal to an agent
	bool is_valid_goal(const std::pair<int,int>& goal); //function to check if a given cell is a valid goal
	std::vector<std::pair<int,int>> get_neighbors_from_roadmap(const std::pair<int,int>& n); //get corresponding 4 neighbor nodes(up, right, down and left) to a given node n
	void build_path_update_roadmap(const int& agent_id,const std::pair<int,int>& goal,const int& goal_theta,const int& goal_time,const int& start_theta,
		std::map<std::vector<int>,std::vector<int>>& parent,std::vector<multiagent_planning::path_info>& path); //builds path backwards from goal given a map containing parent nodes information and updates the world/ roadmap with the created path for further use
	bool Astar_planner(const int& agent_id,const std::pair<int,int>& start,const int& start_theta,const std::pair<int,int>& goal,const int& goal_theta,std::vector<multiagent_planning::path_info>& path); //A* path planner to compute shortest distance path to a goal node
public:
	Planner(); //constructor for the planner class
	void display_world_snapshot(); //function to display a snapshot of the world
	bool plan(const int& agent_id,const std::pair<int,int>& start,const int& start_theta,const std::pair<int,int>& goal,const int& goal_theta,std::vector<multiagent_planning::path_info>& path); //"plan" function that wraps the A* or any other plan algorithms finding shortest distance path to a goal node
}; //Planner class

#endif

