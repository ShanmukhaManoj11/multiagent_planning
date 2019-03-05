# multiagent_planning
Multi-agent path planning

Planner class implements A* on 2D (x,y) grid and generates sequence of (x,y,theta) points as a path

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/4connected_grid.png)

The implementation is based on following assumptions:
1. agent only moves along the edges from a node to one of it's 4 connected neighbors OR waits at the same node 
2. in-cell rotations are "not" considered as seperate moves
	given src pose (x1,y1,theta1) and goal cell (x2,y2), robot first rotates in-cell to a heading angle (theta) that is consistent with its direct movement to cell (x2,y2)
	A move to the next node is assumed to have cost 10
	A direct move to neighbor cell can be the following sequence: 
	"in-cell rotation -> move to neighbor cell -> in-cell rotation [optional]"

3. priority for generating plans is first-come-first, i.e. given 2 agents - agent1 and agent2, if agent2 first calls the /update_goal service, then planner should consider collisions with agent2s path while creating the plan for agent1 whose request comes later

4. to avoid potential collisions, agents can wait in a cell - but waiting costs 5 units (posing cost on waiting helps in avoiding forever wait scenarios like high priority agents goal is on the potential shortest path of other low prioirty agent)

### System architecture

1. Planner

	**planner_node** provides a **/get_plan** service which recieves request containing serial id and goal information and creating the path to the goal location for the agent with the recieved serial id

2. Agent

	a. **agent_node** contains a client to the **/get_plan** service sending appropriate service requests to planner.

	b. **agent_node** provides **/update_goal** service (name remapped to local namespace in launch file) which upon recieving the request of appropriate type (with serial id and new goal location) makes a call to the **/get_plan** service via the client generating the path to the goal location. 

	c. **agent_node** publishes agents current position on the topic **/agent_feedback** (remapped to local namespace)

3. Update goal client

	**update_goal_client** is client node for the **/update_goal** service (remapped to corresponding local namespace based on the agent_node identity) which provides request containing serial id and goal information. The recieved response contains the planned path. This path is published on **/planned_path** topic (remapped to local namespace based on the agent_node identity) which is subscribed by rviz to display the path on the world

Remapping of the topic and service names to local name spaces based on agent identities facilitiates the parallel existance and communication of multiple agents 

### Setting up the code

Assuming the ROS workspace already exists (Creating and building a ROS workspace can be found at ros wiki), place the multiagent_planning module in the src/ directory under the ros workspace. 3 different planning scenarios are provided as launch files and can be run as follows,

```bash
ros_ws$ source devel/setup.bash 
ros_ws$ catkin_make
ros_ws$ roslaunch multiagent_planning planner_server_agent_test.launch
```

### Results

#### Scenario 1: 2 agents with overlapping paths with no potential collision

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario1_rviz.png)

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario1_stdout_log.png)

#### Scenario 2: 2 agents with overlapping paths and one agent has to wait to avoid collision with other

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario2_rviz.png)

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario2_stdout_log.png)

#### Scenario 3: 2 agents with overlapping paths and one agent has to wait to avoid collision with other

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario3_rviz.png)

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario3_stdout_log.png)

#### Scenario 4: 2 agents where an agent has to take a longer path as one agent has its goal location on the potential shortest path of the former agent

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario4_rviz.png)

![alt_text](https://github.com/ShanmukhaManoj11/multiagent_planning/blob/master/images/scenario4_stdout_log.png)