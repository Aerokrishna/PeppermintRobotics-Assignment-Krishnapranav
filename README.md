# Assignment

https://github.com/user-attachments/assets/e92f76cc-014c-4996-85c7-08a7057edd26

## Problem Statement
Implement A star algorithm to find a path from the start point to the goal point on a given map.
Consider a 2D map below where 0 represents free space, and 1 represents an obstacle. 

## Solution 
The implementation is done in Python. The script contains comments for each class, function and logic wherever required. This is the pseudocode for the Astar path planning algorithm based on which the implementation is done. 

```python
# each coordinate of the map is considered a node
# open -> set of nodes which have been explored
# closed -> set of nodes which have been evaluated or visited
# add start node to open
# loop
    # current = node in OPEN with the lowest fcost (minheap)
    # remove the current node from OPEN
    # add current node to CLOSED
    # if current is target then return
    # for each neighbour of the current node 
        # if neighbour is invalid(obstacle or out of bounds) : skip neighbour
        # if new path to neighbor is shorter (update the fcost) or neighbour is unexplored (not in open) : 
        # update fcost, set parent of neighbour to current if neighbour not in open add in open
```

## Implementation
The script contains a class Astar which needs to be initialized with the occupancy grid map which is a 2D matrix. This map is produced as a result of mapping the real world environment. The obstacle pixels are marked as 1 and the free space is 0. Once the class has been initialized the astar function is called, which takes the start and goal coordinates (in the map frame) as arguments. This function returns the path which is a list of coordinates like (x,y) from start to the goal point. The start and the goal point can be passed as arguments while running the script. 

## Usage
To run the script with a provided map, you clone the repository and navigate inside it.
```bash
git clone https://github.com/Aerokrishna/PeppermintRobotics-Assignment-Krishnapranav.git
cd PeppermintRobotics-Assignment-Krishnapranav
```

Then you can run the python script by passing the start and goal coordinates as arguments.
```bash
python3 astar.py --start 0 0 --goal 4 4
```

You will see an output on your terminal.

To use this in your project or script, you need to define your map. Which is a 2D array containing 0 for free space in your map and 1 for obstacles. You then initialize the class with the map. Then you can call the astar() function with your start and goal points wherever required.
```bash
astar_solver = AStar(grid_map)

# start and goal are tuples of (x,y)
path = astar_solver.astar(start, goal)
```





