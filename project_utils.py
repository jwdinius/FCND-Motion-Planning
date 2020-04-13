from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
import matplotlib.pyplot as plt

import networkx as nx
from bresenham import bresenham

from udacidrone.frame_utils import global_to_local

from planning_utils import create_grid

# set globals
COLLINEARITY_EPS = 1e0
GOAL_THRESH = 1.0

def three_tuple(inp):
    """
    input type definition for goal command-line argument
    args:
        inp - comma-separated string with lon,lat,alt - e.g. example valid input is "-122.397,37.7953,3.0"
    returns:
        3-tuple with lon, lat, alt
    raises:
        argparse.ArgumentTypeError if input can't be parsed as 3-tuple
    """
    try:
        lon, lat, alt = map(float, inp.split(","))
        return lon, lat, alt
    except:
        raise argparse.ArgumentTypeError("three_tuple expects three floats separated by commas")

def save_path(grid, wp_ne, start_ne, goal_ne, figname='path.png'):
    """
    takes in grid, waypoints, start, and goal positions
    and saves plot containing the data
    args:
        grid - occupancy grid
        wp_ne - (north, east) waypoint data with map offset applied
        start_ne - (north, east) of starting point with map offset applied
        goal_ne - (north, east) of goal point with map offset applied
        figname - filename to save figure to (default = "path.png")
    returns:
        None
    """
    plt.figure()
    plt.imshow(grid, origin='lower', cmap='Greys')

    # plot edges
    edges = [(p, q) for p,q in zip(wp_ne[:-1], wp_ne[1:])]
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    # plot vertices
    for p in wp_ne:
        plt.plot(p[1], p[0], 'b.')

    # plot start and goal points
    plt.plot(start_ne[1], start_ne[0], 'rx')
    plt.plot(goal_ne[1], goal_ne[0], 'gx')
    
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.savefig(figname)

def show_grid(grid):
    fig = plt.figure()
    plt.imshow(grid, origin='lower', cmap='Greys')
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

def get_object_centers(data, map_offset, alt, safety_distance):
    """
    get obstacle centers
    args:
        data - obstacle data (read-in from colliders.csv)
        map_offset - (north, east) map offset
        alt - drone altitude
        safety_distance - safety distance from obstacle
    returns:
        obstacle centers list
    """
    north_offset, east_offset = map_offset
    centers = []
    for d in data:
        obs_north, obs_east, obs_alt, _, _, res_alt = d
        if obs_alt + res_alt + safety_distance > alt:
            centers.append([obs_north - north_offset, obs_east - east_offset])
    return centers

def find_open_edges(graph, grid):
    """
    finds connections between vertices in the graph (edges) that do not intersect with obstacles
    args:
        graph - Voronoi graph
        grid - occupancy data
    returns:
        list of edges that do not intersect with obstacles
    """
    edges = []
    # ridge_vertices come from Voronoi graph object
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        # find cells in graph connected by vertices
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        # initialize intersection flag to false
        intersection = False

        for c in cells:
            # check if c is off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                intersection = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]]:
                intersection = True
                break

        if not intersection:
            # append edge
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))
    return edges

def create_graph_from_edges(edges):
    """
    create a graph from edges
    args:
        edges - list of graph edges
    returns:
        NetworkX graph object
    """
    G = nx.Graph()
    for e in edges:
        v1, v2 = e
        # weight the edge by the length of the edge
        dist = np.linalg.norm(np.array(v1) - np.array(v2))
        G.add_edge(v1, v2, weight=dist)
    return G

def create_graph(data, alt, safety_distance):
    """
    returns a graph over open space given occupancy grid
    args:
        data - occupancy data
        alt - drone altitude
        safety_distance - safety distance from obstacle
    returns:
        graph structure over unobstructed space
    """
    # get occupancy grid and offsets.
    grid, north_offset, east_offset = create_grid(data, alt, safety_distance)

    # find object centers
    centers = get_object_centers(data, (north_offset, east_offset), alt, safety_distance)

    # create Voronoi object
    voronoi = Voronoi(centers)

    # find clear edges
    edges = find_open_edges(voronoi, grid)

    # return graph and offsets
    return (create_graph_from_edges(edges), north_offset, east_offset)

def graph_a_star(graph, h, start, goal):
    """
    A* search on NetworkX graph
    args:
        graph - graph structure over unobstructed space
        h - search heuristic to speed-up search
        start - start position
        goal - position
    returns:
        subgraph connecting start and goal points
    """
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node)

    path = []
    path_cost = None
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])

    return path[::-1], path_cost

def closest_point(graph, point):
    """
    find closest point on the graph to an input point
    args:
        graph - NetworkX graph
        point - 3-dim point to check against
    returns:
        graph vertex closest to input point
    """
    curr_point = (point[0], point[1])
    closest_point = None
    dist = None
    for p in graph.nodes:
        d = np.linalg.norm(np.array(p) - np.array(curr_point))
        if not dist or d < dist:
            closest_point = p
            dist = d
    return closest_point

def heuristic_l1(position, goal_position):
    """
    compute L1 norm between two points
    args:
        position - first point
        goal_position - second point
    returns:
        L1 distance between two input points
    """
    return np.abs(position[0] - goal_position[0]) + np.abs(position[1] - goal_position[1])

def prune_waypoints(wp, epsilon):
    """
    prune waypoints based on collinearity 
    args:
        wp - list of waypoints
        epsilon - pruning factor
    returns:
        list of pruned waypoints
    """
    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinear(p1, p2, p3):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return float(np.abs(det)) < epsilon

    # initialize pruned path with input path
    pruned_wp = [p for p in wp]
    i = 0
    while i < len(pruned_wp) - 2:
        p1 = point(pruned_wp[i])
        p2 = point(pruned_wp[i+1])
        p3 = point(pruned_wp[i+2])

        # if the 3 points are collinear remove the middle one.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinear(p1, p2, p3):
            pruned_wp.remove(pruned_wp[i+1])
        else:
            i += 1
    return pruned_wp

def calculate_waypoints(global_start, global_goal, global_home, data, alt, safety_distance):
    """
    calculate waypoints in local coordinates for path between global start and global goal positions given
    a desired altitude and a desired distance to keep safe from obstacles
    args:
        global_start - start position in global coordinates
        global_goal - goal position in global coordinates
        global_home - home position in global coordinates
        data - occupancy grid data
        alt - drone altitude
        safety_distance - safety distance from obstacle
    returns:
        list of path waypoints
    """
    # calculate graph and offsets from occupancy data
    graph, north_offset, east_offset = create_graph(data, alt, safety_distance)
    map_offset = np.array([north_offset, east_offset, .0])

    # compute local start position with offset
    local_position = global_to_local(global_start, global_home) - map_offset

    # find closest point to the graph for start position
    graph_start = closest_point(graph, local_position)

    # compute local goal position with offset
    local_goal = global_to_local(global_goal, global_home) - map_offset

    # if goal is close to starting point, just return a single waypoint and be done with it
    if np.linalg.norm(local_goal[:2] - local_position[:2]) < GOAL_THRESH:
        p = local_goal[:2]
        alt = -local_goal[2]
        h = np.arctan2(local_goal[1] - local_position[1], local_goal[0] - local_position[0]) 
        return [[int(p[0] + north_offset), int(p[1] + east_offset), alt, h]]
        
    # find closest point to the graph for goal position
    graph_goal = closest_point(graph, local_goal)

    # find path (if one exists)
    path, _ = graph_a_star(graph, heuristic_l1, graph_start, graph_goal)
    
    # return empty path if valid path could not be found for some reason
    if len(path) == 0:
        return path
    
    # need to add final goal position back (because it's not necessarily on the graph)
    path.append(local_goal)

    # prune path based on collinearity checks
    path = prune_waypoints(path, COLLINEARITY_EPS)

    # set heading based on difference between waypoints
    des_heading = [np.arctan2(p2[1] - p1[1], p2[0] - p1[0]) for p1,p2 in zip(path[:-1], path[1:])]
    des_heading.append(des_heading[-1])  # need to add final heading command manually

    # add back offset data and round 
    return [[int(p[0] + north_offset), int(p[1] + east_offset), alt, h] for p,h in zip(path, des_heading)]
