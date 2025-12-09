# PSEUDOCODE : ASTAR

# open -> set of nodes which have been explored
# closed -> set of nodes which have been evaluated or visited
# add start node to OPEN
# loop
    # current = node in OPEN with the lowest fcost (minheap)
    # remove the current node from OPEN
    # add current node to CLOSED
    # if current is target then return
    # for each neighbor of the current node 
        # if neighbor is invalid(obstacle or in closed) : skip
        # if new path to neighbor is shorter (update the fcost) or neighbor is unexplored (not in open) : 
        # update fcost, set parent of neighbor to current if neighbor not in open add in open

import heapq
import argparse

"""
Stores info  each node
g = cost from start
f = total cost = g + h (where h is cost to goal)
parent = previous node_id in the path (used for back tracking later)
"""
class NodeInfo:
    
    def __init__(self, g, f, parent):
        self.g = g
        self.f = f
        self.parent = parent

"""
Class which will be initialized with the map (occupancy grid map, a 2d array containing 1 for obstacles and 0 for free space)
"""
class AStar:
    def __init__(self, map):
        self.map = map

        # size of the map
        self.rows = len(map)
        self.cols = len(map[0])

        # structures for astar

        # open set stores the node_id of the nodes which have been explored but not visited
        # allows constant time search whenever required
        self.OPEN_SET = set()    

        # closed set stores the node_id of the nodes which have been visited
        # allows constant time search whenever required
        self.CLOSED_SET = set()       

        # fcost stored as a minheap sorted by fcost
        # so that we can access the node_id with the least fcost in constant time
        self.FCOST = []           

        # hashmap which contains node_id : NodeInfo, stores the cost as well as the parent node which will be used for tracing the path
        self.NODES = {}               

        # moves four directions
        self.moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    # node_id is the index of the element in the map when flattened into an array
    # gives us a mapping between the coordinates in the map (row,column) to the node_id and viceversa

    # Name : rc_to_id
    # input : row, column (indices of the map)
    # output : node_id
    # functionality : maps the coordinatea in map to node_id
    def rc_to_id(self, r, c):
        return r * self.cols + c
    
    # Name : id_to_rc
    # input : node_id
    # output : row, column (indices of the map)
    # functionality : maps the node_id to coordinates in map  
    def id_to_rc(self, node_id):
        return node_id // self.cols, node_id % self.cols
    
    # Name : is_valid
    # input : row, column
    # output : bool(whether valid or not)
    # functionality : function to check whether the node is valid or not, 
    # it is invalid L if the node is an obstacle, if the node is out of the map
    def is_valid(self, r, c):

        # check if the index of the node is out of bounds wrt to map,  
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols:
            return False
        
        # check if the index of the node is an obstacle
        if self.map[r][c] == 1:   
            return False
        return True

    # Name : heuristic
    # input : row,column, goal_row, goal_column
    # output : heuristic
    # functionality : returns euclidian distance between two nodes
    def heuristic(self, r, c, goal_r, goal_c):
        return ((r-goal_r)**2 + (c-goal_c)**2)**0.5

    # backtrack to reconstruct the path, takes goal_id as argument
    # Name : reconstruct_path
    # input : goal_id
    # output : array of coordinates
    # functionality : constructs the path from start to the goal point using the node's parent node id
    def reconstruct_path(self, goal_id):
        path = []
        curr = goal_id

        # till you reach the start node
        while curr is not None:
            r, c = self.id_to_rc(curr)
            path.append((r, c))
            curr = self.NODES[curr].parent  

        # reverse the path so that path is from start to goal
        return path[::-1]

    # Name : astar
    # input : start_point : (r,c), goal_point : (r,c) r,c are coordinates or indices of the map array
    # output : array of coordinates
    # functionality : finds the shortest point from start to goal point 
    def astar(self, start_point, goal_point):

        start_r, start_c = start_point
        goal_r, goal_c = goal_point

        # get the id of start and goal
        start_id = self.rc_to_id(start_r, start_c)
        goal_id = self.rc_to_id(goal_r, goal_c)

        # add start to OPEN_SET and add it in NODES 
        h = self.heuristic(start_r, start_c, goal_r, goal_c) # hcost
        self.NODES[start_id] = NodeInfo(g=0, f=h, parent=None)
        heapq.heappush(self.FCOST, (h, start_id)) # store the fcost and node id
        self.OPEN_SET.add(start_id)

        # loop till FCOST heap is empty which means all the nodes have been visited
        while self.FCOST:

            # current node = node with lowest f_cost
            f_cost, current_id = heapq.heappop(self.FCOST)

            # check if the current id is in the OPEN SET (primarily because we dont remove values from heap when we update OPEN_SET)
            if current_id not in self.OPEN_SET:
                continue
            
            # remove the current node from OPEN SET and mark it as visited
            self.OPEN_SET.remove(current_id)
            self.CLOSED_SET.add(current_id)

            # if the current node is the goal node reconstruct the path and return the path
            if current_id == goal_id:
                return self.reconstruct_path(goal_id)

            # get the coordinates from id
            curr_r, curr_c = self.id_to_rc(current_id)

            # get its gcost
            curr_g = self.NODES[current_id].g

            # check the neighbors
            for dr, dc in self.moves:

                # dr, dc are delta indices, get the new node coordinates nr and nc
                nr, nc = curr_r + dr, curr_c + dc

                # check if valid, if not skip
                if not self.is_valid(nr, nc):
                    continue
                
                # get the neighbor id
                neighbor_id = self.rc_to_id(nr, nc)

                # check if the node has already been visited
                if neighbor_id in self.CLOSED_SET:
                    continue

                # compute new gcost and fcost
                tentative_g = curr_g + 1 # distance from the current node is 1 
                tentative_f = tentative_g + self.heuristic(nr, nc, goal_r, goal_c)

                # if the neighbor is not in openset, meaning it has not been explored
                if neighbor_id not in self.OPEN_SET:
                    # store info about this node
                    self.NODES[neighbor_id] = NodeInfo(tentative_g, tentative_f, current_id)
                    #push its fcost into the heap and add it to the openset
                    heapq.heappush(self.FCOST, (tentative_f, neighbor_id))
                    self.OPEN_SET.add(neighbor_id)

                # if it has already been explored
                else:
                    # check if the new gcost is lesser than the previously stored gcost
                    if tentative_g < self.NODES[neighbor_id].g:
                        # update the gcost, fcost and its parent
                        self.NODES[neighbor_id].g = tentative_g
                        self.NODES[neighbor_id].f = tentative_f
                        self.NODES[neighbor_id].parent = current_id

                        # push updated fcost to the heap
                        heapq.heappush(self.FCOST, (tentative_f, neighbor_id))

        # no path obtained
        return None


# usge
def main():
    parser = argparse.ArgumentParser(description="Run A* path planning on a 2D grid map.")
    # start x y
    parser.add_argument("--start", "-s", nargs=2, type=int, required=True, help="Start coordinate: x y")
    # -goal x y
    parser.add_argument("--goal", "-g", nargs=2, type=int, required=True,help="Goal coordinate: x y")

    args = parser.parse_args()
    # convert it inro tuple
    start = tuple(args.start)   # (x, y)
    goal = tuple(args.goal)     # (x, y)

    # map
    grid_map = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    # Create A* instance and run
    astar_solver = AStar(grid_map)
    path = astar_solver.astar(start, goal)

    print("Path:", path)


if __name__ == "__main__":
    main()
