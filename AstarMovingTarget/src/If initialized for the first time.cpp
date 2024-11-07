If not initialized:
	Start timer
	For each timestep of the target
		Store target position and timestep in goal_traj
		If timestep > total timestep / 2
			Create and add node to open_list
	ComputeHeuristics()
	Clear variables
	Push starting node of robot to open_list
	ComputeAstarPath()

If path exists
	Command robot to move to next position in path

If robot reached target
	Clear data structures

Update action_ptr 
