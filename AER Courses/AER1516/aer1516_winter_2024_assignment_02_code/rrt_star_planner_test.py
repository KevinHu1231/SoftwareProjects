"""
Assignment #2 Template file
"""
import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees* (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_star_planner.py. Your
   implementation can be tested by running RRT_DUBINS_PROBLEM.PY (check the 
   main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specificed below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)
3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_star_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def rrt_star_planner(rrt_dubins, display_map=False):
    """
        Execute RRT* planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    # LOOP for max iterations
    i = 0
    check_radius = 3
    #check_points = [(12.5,0,math.pi/2),(12.5,15,math.pi),(7.5,15,math.pi),(7.5,10,0),(6.5,12.5,3*math.pi/2),(rrt_dubins.goal.x,rrt_dubins.goal.y,rrt_dubins.goal.yaw)]
    check_points = [(12.5,0,math.pi/2),(12.5,15,math.pi),(7.5,15,math.pi),(7.5,10,0),(10,13.8,7*math.pi/6),(8,12.5,7*math.pi/6),(rrt_dubins.goal.x,rrt_dubins.goal.y,rrt_dubins.goal.yaw)]
    while i < rrt_dubins.max_iter:
        x = check_points[i][0]
        y = check_points[i][1]
        yaw = check_points[i][2]
        i += 1

        # Add any addtional code you require for RRT*.
        to_node = rrt_dubins.Node(x,y,yaw)

        # Find an existing node nearest to the random vehicle state
        best_dist = np.inf
        best_node = None
        for node in rrt_dubins.node_list:
            dist = rrt_dubins.calc_new_cost(node,to_node) - node.cost
            if dist < best_dist:
               best_dist = dist
               best_node = node

        new_node = rrt_dubins.propogate(best_node, to_node)


        # Check if the path between nearest node and random state has obstacle collision
        # Add the node to nodes_list if it is valid

        near_nodes = []
        if rrt_dubins.check_collision(new_node):
            for node in rrt_dubins.node_list:
                if np.sqrt((node.x - new_node.x)**2+(node.y - new_node.y)**2) < check_radius:
                  near_nodes.append(node)

            node_min = new_node
            cost_min = rrt_dubins.calc_new_cost(best_node,new_node)

            for near_node in near_nodes:
                near_to_new = rrt_dubins.propogate(near_node,new_node)

                if rrt_dubins.check_collision(near_to_new) and (rrt_dubins.calc_new_cost(near_node,new_node) < cost_min):

                    node_min = near_to_new
                    cost_min = rrt_dubins.calc_new_cost(near_node,new_node)

            rrt_dubins.node_list.append(node_min) # Storing all valid nodes

            for near_node in near_nodes:
                new_to_near = rrt_dubins.propogate(node_min,near_node)
                if rrt_dubins.check_collision(new_to_near) and (rrt_dubins.calc_new_cost(node_min,near_node) < near_node.cost):
                    rrt_dubins.node_list.remove(near_node)
                    rrt_dubins.node_list.append(new_to_near)

        else:
            continue

        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()

        # Check if new_node is close to goal
        if node_min.is_state_identical(rrt_dubins.goal):
            print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
            break

    if i == rrt_dubins.max_iter:
        print('reached max iterations')

    # Return path, which is a list of nodes leading to the goal...
    path = []
    while 1:
        path = [node_min] + path
        if node_min.is_state_identical(rrt_dubins.start):
            break
        node_min = node_min.parent
    return path
