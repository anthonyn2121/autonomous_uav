from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World
from proj1_3.code.occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    g_index = np.array(goal_index)

    def getNeighbors(curr_index, map):
        neighbors = []
        diff = [-1, 0, 1]
        for i in diff:
            for j in diff:
                for k in diff:
                    neighbor_index = [curr_index[0] + i, curr_index[1] + j, curr_index[2] + k]

                    if (map.is_valid_index(neighbor_index) and not map.is_occupied_index(neighbor_index)):
                        neighbors.append(tuple(neighbor_index))

        neighbors.remove(curr_index)
        return neighbors

    def getPath(start_index, goal_index, parent, map):
        path_index = [goal_index]
        while path_index[-1] != start_index:
            path_index.append(parent[path_index[-1]])
        path_index = path_index[::-1]

        path = []
        for i in path_index:
            if i == start_index:
                    #and i == goal_index:
                # path.append(OccupancyMap.index_to_metric_negative_corner(map, i))
                path.append(start)
            elif i == goal_index:
                path.append(goal)
            else:
                path.append(OccupancyMap.index_to_metric_center(map, i))

        path = np.array(path)
        return path

    # Initialize variables
    open = [] #list of nodes we can still step to
    isVisited = {}
    g = {} #cost
    parent = {}

    graph = ((i,j,k) for i in range((occ_map.map.shape[0])) for j in range((occ_map.map.shape[1])) for k in range((occ_map.map.shape[2])))
    for node in graph:
        isVisited[node] = False
        g[node] = np.inf
        parent[node] = 0


    g[start_index] = 0
    heappush(open,([g[start_index],start_index]))
    iterations = 0
    while(isVisited[goal_index] == False):
        curr_index = heappop(open)[1]
        if (isVisited[curr_index] == False):
            iterations += 1
            isVisited[curr_index] = True
            curr_neighbors = getNeighbors(curr_index, occ_map)
            for v in curr_neighbors:
                neighbor_cost = g[curr_index] + np.linalg.norm(np.array(curr_index) - np.array(v))
                if (astar):
                    heuristic = np.linalg.norm(g_index - np.array(v))
                else:
                    heuristic = 0
                if (neighbor_cost < g[v]):
                    heappush(open, ([neighbor_cost + heuristic, v]))
                    g[v] = neighbor_cost
                    parent[v] = curr_index


    # build path
    if (isVisited[goal_index]):
        path = getPath(start_index, goal_index, parent, occ_map)
        print("Path found")
        # print(f"{iterations} nodes visited")
        print(f"{len(path)} nodes visited")
    else:
        print("Path not found")

    return path
