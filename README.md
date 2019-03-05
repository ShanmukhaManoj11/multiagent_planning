# multiagent_planning
Multi-agent path planning

Planner class implements A* on 2D (x,y) grid and generates sequence of (x,y,theta) points as a path
The implementation is based on following assumptions:
1. agent only moves along the edges from a node to one of it's 4 connected neighbors OR waits at the same node 
2. in-cell rotations are "not" considered as seperate moves
given src pose (x1,y1,theta1) and goal cell (x2,y2), robot first rotates in-cell to a heading angle (theta) that is consistent with its direct movement to cell (x2,y2)
A move to the next node is assumed to have cost 10
A direct move to neighbor cell can be the following sequence: 
	"in-cell rotation -> move to neighbor cell -> in-cell rotation [optional]"
3. priority for generating plans is first-come-first, i.e. given 2 agents - agent1 and agent2, if agent2 first calls the /update_goal service, then planner should consider collisions with agent2s path while creating the plan for agent1 whose request comes later
4. to avoid potential collisions, agents can wait in a cell - but waiting costs 5 units (posing cost on waiting helps in avoiding forever wait scenarios like high priority agents goal is on the potential shortest path of other low prioirty agent)

