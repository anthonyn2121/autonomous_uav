from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World
from proj1_2.code.occupancy_map import OccupancyMap  # Recommended.


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
    goal_metric = occ_map.index_to_metric_center(goal_index)
    g_index = np.array(goal_index)

    # Initialize lists used by Dijkstra/A*
    open_list = []
    visited_list = np.zeros(shape=occ_map.map.shape)
    g = np.full(occ_map.map.shape, np.inf)
    f = np.full(occ_map.map.shape, np.inf)
    parent = np.zeros(shape=occ_map.map.shape, dtype=(float, 3))


    # Push the start position into the queue
    g[start_index] = 0
    heappush(open_list, tuple([g[start_index], 0, start_index]))
    # Dijkstra loop
    count = 0
    open_count = 0
    while(not visited_list[goal_index] and len(open_list)):
        curr_index = heappop(open_list)[2]
        # print("current index: ", curr_index, " open_list: ", len(open_list))
        if (not visited_list[curr_index]):
            count += 1
            visited_list[curr_index] = 1
            curr_neighbors = getNeighbors(curr_index, occ_map)
            for neighbor_index in curr_neighbors:
                neighbor_cost = g[curr_index] + np.linalg.norm(
                    np.array(curr_index) - np.array(neighbor_index))
                if (astar):
                    heuristic_cost = np.linalg.norm(
                        g_index - np.array(neighbor_index))
                else:
                    heuristic_cost = 0
                if (neighbor_cost < g[neighbor_index]):
                    g[neighbor_index] = neighbor_cost
                    parent[neighbor_index] = curr_index
                    open_count += 1
                    heappush(open_list, tuple(
                        [neighbor_cost + heuristic_cost, open_count, neighbor_index]))
        # else:
        #     print(curr_index, " already visited")

    path = None
    # construct the path
    if (visited_list[goal_index]):
        path = getPath(start_index, goal_index, parent, occ_map)
        path = np.vstack((start, path))
        path = np.vstack((path, goal))
        print("Path found")
        # print(path)
        print(f"{count} nodes visited")
    else:
        print("Path not found")

    return path


def getNeighbors(curr_index, map):
    neighbors = []
    diff = [-1, 0, 1]
    for i in diff:
        for j in diff:
            for k in diff:
                neighbor_index = [curr_index[0]+i,
                                  curr_index[1]+j, curr_index[2]+k]

                if(map.is_valid_index(neighbor_index)):
                    if (not map.is_occupied_index(neighbor_index)):
                        neighbors.append(tuple(neighbor_index))

    neighbors.remove(curr_index)
    return neighbors


def getPath(start_index, goal_index, parent, map):
    path = np.array([]).reshape(0, 3)
    #curr_index = tuple(parent[goal_index].astype(int))
    curr_index = goal_index
    while(not np.array_equal(curr_index, start_index)):
        path = np.vstack((map.index_to_metric_center(curr_index), path))
        curr_index = tuple(parent[curr_index].astype(int))
    path = np.vstack((map.index_to_metric_center(start_index), path))
    return path
