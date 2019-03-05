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

Planner class implements A* on 2D (x,y) points and the generated sequence of (x,y) points are further processd to create a path which is a sequence of (x,y,theta)
The processing (conversion to (x,y,theta) sequence) is based on following assumptions:
	1. agent only moves along the edges from one node to one of it's 4 connected neighbors OR waits at the same node OR turns to 0, 90, 180 or 270 degrees theta on the same node
	2. if (x,y) is a point at index i in the path of sequence of xy points and (X,Y,TH) is the corresponding processed point; (x1,y1) is the next point that is i+1 th point in the sequence, 
	then a direct move to (x1,y1) from the (X,Y,TH) is possible when,
		a. TH==0 deg and x1==X+1 and y1==Y (OR)
		b. TH=90 deg and x1==X and y1==Y+1 (OR)
		c. TH=180 deg and x1==X1-1 and y1==Y (OR)
		d. TH=270 deg and x1==X and y1==Y-1
	and the next point to be added to the x-y-theta sequence will be (X1,Y1,TH) where x1=X1 and y1=Y1
	Otherwise an intermediate (X,Y,TH1) is added to the x-y-theta sequence where the agent rotates to angle TH1 from TH so that it first aligns with the point (x1,y1) for a direct move,
	and then point (X1,Y1,TH1) is added to the x-y-theta sequence.
This processing step is implemented in the member function "create_path_from_xy_points". This function also udates the roadmap data structure with the cell information of each cell
This function also updates the cells in the roadmap so that the occupancy of cells in time is available for the future paths
*/
class Planner{
private:
	std::vector<std::vector<cell_info>> roadmap; //roadmap - 2d world graph with nodes and corresponding 4 connected edges
	//roadmap[j][i] = {agent_id,time when agent_id occupies the node (x=i, y=j)}
	int n_nodes; //n_nodesxn_nodes is the size of the roadmap
	bool is_goal_to_an_agent(const std::pair<int,int>& node);
	bool is_valid_goal(const std::pair<int,int>& goal);
	bool Astar_planner(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path); //A* path planner to compute shortest distance path to a goal node
public:
	Planner(); //constructor for the planner class
	void display_world_snapshot(); //function to display a snapshot of the world
	std::vector<std::pair<int,int>> get_neighbors_from_roadmap(const std::pair<int,int>& n); //get corresponding 4 neighbor nodes(up, right, down and left) to a given node n
	bool plan(const int& agent_id,const std::pair<int,int>& start,const std::pair<int,int>& goal,std::vector<std::pair<int,int>>& path); //"plan" function that wraps the A* or any other plan algorithms finding shortest distance path to a goal node
	void create_path_from_xy_points(const int& agent_id,const std::vector<std::pair<int,int>>& xy_path,const int& start_theta,const int& goal_theta,std::vector<multiagent_planning::path_info>& response_path); //given sequence of xy-points on the planned path, convert these points to (x,y,theta)
}; //Planner class

#endif

