"""Example utility module.

Please use a file like this one to add extra functions.

"""

def exampleFunction():
    """Example of user-defined function.

    """
    x = -1
    return x

import numpy as np
import random
import threading
import time
import copy

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#random.seed(10)
np.random.seed(10)

# Node class for RRT*
class Node():
    def __init__(self, state):
        self.parent = None
        self.children = []
        self.cost = 0
        self.state = state

# Class for Cylinder Obstacle
class CylinderObstacle():
    def __init__(self, height, radius, x, y):
        self.height = height
        self.radius = radius
        self.x = x
        self.y = y

# Class for gate cylinder under frame
class GateObstacle():
    def __init__(self, height, radius, frame, orient, x, y):
        self.height = height
        self.radius = radius
        self.x = x
        self.y = y
        self.frame = frame
        self.horizontal_orient = orient 

# Class for gate frame
class FrameObstacle():
    def __init__(self,height,length,width,orient,x,y,z):

        self.horizontal_orient = orient
        self.height = height

        if orient: #Boolean for orientation of gate
            self.length = length
            self.width = width
        else:
            self.length = width
            self.width = length

        self.x = x
        self.y = y
        self.z = z

# Class for RRT*
class RRT():
    def __init__(self, start, end, gate_orient, gate, nodes = [], lines = [], obstacles = [], path = None, path_1 = [], path_2 = [], path_lines = [], bias = True, dimension = 3, plot = True):
        self.start = start # Start and end positions
        self.end = end

        self.plot = plot #Used for plotting in Python, not used in simulation
        self.fig = None
        self.figure = None

        self.end_state_1 = None # End state positions of gate
        self.end_state_2 = None
        self.end_state = None
        self.horizontal_orient = gate_orient
        self.end_state_dist = 0.3 # no less than 0.15 allowed, endpoint distance from center point of gate

        self.closest_node = None # Closest node to goal position
        self.closest_node_dist = np.inf

        self.nodes = nodes # List of Nodes
        self.lines = lines

        self.path = path # Current path
        self.path_lines = path_lines
        self.path_cost = np.inf

        self.path_improved = False # Check if path improved this iteration

        self.rewire_radius = 1 # Hyperparameters
        self.run = True
        self.bias = bias
        self.bias_weight = 0.1
        self.safety_dist = 0.15
        self.obstacle_uncertainty = 0.2
        self.goal_radius = 0.1
        self.goal_vicinity = 0.2
        self.dimension = dimension

        self.obstacles = obstacles # List of Obstacles

        self.next_state = False # Check whether or not to expand to goal next iteration

        self.lower_bound = np.array([-1.3,-3.0,0]) # Bounds of generated nodes
        self.upper_bound = np.array([2.2,2.1,1.3])

        self.gate = gate # Location of gate

        self.init_RRT() # Initialize RRT nodes and obstacles

    def init_RRT(self):
        self.start = Node(self.start) # Create Nodes

        if self.horizontal_orient == True: # Create endpoint nodes
            self.end_state_1 = Node(self.end + np.array([0, self.end_state_dist ,0]))
            self.end_state_2 = Node(self.end - np.array([0, self.end_state_dist ,0]))
        elif self.horizontal_orient == False:
            self.end_state_1 = Node(self.end + np.array([self.end_state_dist, 0, 0]))
            self.end_state_2 = Node(self.end - np.array([self.end_state_dist, 0, 0]))
        else:
            self.end_state_1 = Node(self.end)
            self.end_state_2 = Node(self.end - np.array([100, 100, 100]))

        self.nodes = [self.start] # Add starting node to node list

        dist_1 = self.dist(self.start,self.end_state_1)
        dist_2 = self.dist(self.start,self.end_state_2)
        
        if dist_1 < dist_2: # Update closest node to goal location
            self.end_state = self.end_state_1
            self.update_closest_node(self.start,dist_1)
        else:
            self.end_state = self.end_state_2
            self.update_closest_node(self.start,dist_2)

        self.init_obstacles() # Initialize obstacles

        if self.plot == True: # Initialize plotting if plotting turned on
            self.init_plot()

    def init_obstacles(self): # Define obstacles and their locations and add to obstacle list
        extra_radius = self.safety_dist + self.obstacle_uncertainty

        obs_1 = CylinderObstacle(1.3,0.12+extra_radius,-1,0)
        obs_2 = CylinderObstacle(1.3,0.12+extra_radius,1.5,0)
        obs_3 = CylinderObstacle(1.3,0.12+extra_radius,0.5,-1)
        obs_4 = CylinderObstacle(1.3,0.12+extra_radius,1.5,-2.5)

        gate_1 = GateObstacle(0.8,0.1+self.safety_dist,0.4,True,-0.5,1.5)
        gate_2 = GateObstacle(0.8,0.1+self.safety_dist,0.4,False,0,0.2)
        gate_3 = GateObstacle(0.8,0.1+self.safety_dist,0.4,True,2,-1.5)
        gate_4 = GateObstacle(0.8,0.1+self.safety_dist,0.4,False,0.5,-2.5)

        frame_4 = FrameObstacle(0.6,0.8,0.4,True,-0.5,1.5,1) 
        frame_3 = FrameObstacle(0.6,0.8,0.4,False,0,0.2,1) 
        frame_2 = FrameObstacle(0.6,0.8,0.4,True,2,-1.5,1) 
        frame_1 = FrameObstacle(0.6,0.8,0.4,False,0.5,-2.5,1) 

        self.obstacles = [obs_1,obs_2,obs_3,obs_4,gate_1,gate_2,gate_3,gate_4,frame_1,frame_2,frame_3,frame_4] 

    def check_collision(self, path): # Check for collision of any obstacle in obstacle list
        for obstacle in self.obstacles:
            if isinstance(obstacle, CylinderObstacle) or isinstance(obstacle, GateObstacle):
                collision_status = self.check_collision_cylinder(path,obstacle) # Check for collision with cylinder
            elif isinstance(obstacle, FrameObstacle):
                collision_status = self.check_collision_frame(path,obstacle) # Check for collision with frame
            if collision_status == True:
                return False
        return True
    
    def check_collision_cylinder(self, path, obstacle): #Check for collision with cylinder
        start = path.parent.state
        end = path.state
        n = end - start # Normal vector in direction of travel
        a = n[0]**2 + n[1]**2 # Coefficients of quadratic equation defined in report
        b = 2*(start[0]*n[0] + start[1]*n[1] - n[0]*obstacle.x - n[1]*obstacle.y)
        c = start[0]**2 + obstacle.x**2 + start[1]**2 + obstacle.y**2 - 2*start[0]*obstacle.x - 2*start[1]*obstacle.y - obstacle.radius**2

        discriminant = b**2 - 4*a*c # Discriminant calculation

        if discriminant > 0:
            t1 = (-b + np.sqrt(discriminant)) / (2*a) # Solutions to quadratic equation
            t2 = (-b - np.sqrt(discriminant)) / (2*a)

            # Check if any of the solutions are within the valid range
            
            # Check if line intersects infinite cylinder under certain height

            if (0 <= t1 <= 1):
                if ((start + t1*n)[2] <= obstacle.height):
                    return True
            
            if (0 <= t2 <= 1):
                if ((start + t1*n)[2] <= obstacle.height):
                    return True
                
        # Check if line intersects end cap
        t = (obstacle.height - start[2])/n[2]
        # Check when line is at same height as obstacle 
        if (0 <= t <= 1):
            x = start[0] + t*n[0]
            y = start[1] + t*n[1]
            
            # Check if line intersects end cap at collision time
            if np.linalg.norm(np.array([x,y]) - np.array([obstacle.x,obstacle.y])) < obstacle.radius:
                return True
    
        return False
    
    
    def check_collision_frame(self, path, obstacle): # Check for collision with frame
        start = path.parent.state
        end = path.state

        #Vectors for height, length, width and center of obstacle
        height_vec = np.array([0,0,obstacle.height])
        center_vec = np.array([obstacle.x,obstacle.y,obstacle.z])

        #Length defined here as x length and Width defined here as y length
        length_vec = np.array([obstacle.length,0,0])
        width_vec = np.array([0,obstacle.width,0])

        # Define boundaries of each rectangle
        bounds1 = [center_vec-width_vec/2-height_vec/2-length_vec/2, center_vec-width_vec/2+height_vec/2+length_vec/2]
        bounds2 = [center_vec+width_vec/2-height_vec/2-length_vec/2, center_vec+width_vec/2+height_vec/2+length_vec/2]
        bounds3 = [center_vec-width_vec/2-height_vec/2-length_vec/2, center_vec+width_vec/2+height_vec/2-length_vec/2]
        bounds4 = [center_vec-width_vec/2-height_vec/2+length_vec/2, center_vec+width_vec/2+height_vec/2+length_vec/2]

        # Check for collision with any of the four rectangles
        if self.line_intersect_rectangle(length_vec,height_vec,bounds1[0],bounds1,start,end) == True:
            return True
        
        if self.line_intersect_rectangle(length_vec,height_vec,bounds2[0],bounds2,start,end) == True:
            return True
        
        if self.line_intersect_rectangle(width_vec,height_vec,bounds3[0],bounds3,start,end) == True:
            return True
        
        if self.line_intersect_rectangle(width_vec,height_vec,bounds4[0],bounds4,start,end) == True:
            return True
        
        return False
    
    def line_intersect_rectangle(self,rect_vec1,rect_vec2,rect_pt,bounds,start,end): # Check for collision with rectangle
        # Calculate the normal vector of the rectangle's plane
        normal = np.cross(rect_vec1, rect_vec2)
        normal = normal/np.linalg.norm(normal)

        # Calculate the direction vector of the line segment
        line_dir = end - start
        line_len = np.linalg.norm(line_dir)
        D = line_dir/line_len

        # Calculate the distance from the line_start to the rectangle plane
        a = np.dot((rect_pt - start),normal) / np.dot(D,normal)

        # Check if the intersection point is on the line segment
        if (a < 0) or (a > line_len):
            return False
        
        # Calculate the intersection point
        intersection_point = np.round(start + a * D, 3)
        if (bounds[0][0] <= intersection_point[0]) and (bounds[0][1] <= intersection_point[1]) and (bounds[0][2] <= intersection_point[2]) and (bounds[1][0] >= intersection_point[0]) and (bounds[1][1] >= intersection_point[1]) and (bounds[1][2] >= intersection_point[2]):
            return True
        else:
            return False
    
    def plan(self): # Function to find path for RRT*
        it = 0 # Iteration counter
        while self.run == True:

            if self.next_state == True: # Expand to goal if previous state close to goal
                self.next_state = False
                new_node = self.end_state
            else:
                new_node = self.generate_rand_state() # Generate random state via biased sampling

                if self.path == None: # If random state close to goal either expand to goal or expand to goal next iteration
                    distance = self.dist(new_node,self.end_state)
                    if (self.goal_radius < distance) and (distance < self.goal_vicinity):
                        self.next_state = True

                    elif distance < self.goal_radius:
                        new_node = self.end_state
            
            best_node, nearest_neighbors, in_radius = self.find_nearest_neighbors(new_node) # Find nearest neighbors of current node
            new_node = self.create_path(best_node,new_node) # Create path between current node and nearest neighbor

            if self.check_collision(new_node): # Check for collision of newly created path
                node_min = new_node # Track lowest cost path
                cost_min = new_node.cost

                # Update lowest cost node
                for near_node in nearest_neighbors: # Check neighbors within the rewire radius for a lower cost path
                    #print("Update lowest cost node")
                    near_to_new = self.create_path(near_node,new_node)
                    if self.check_collision(near_to_new) and (near_to_new.cost < cost_min): # If path has no collision and is lower cost update path
                        node_min = near_to_new
                        cost_min = near_to_new.cost
                
                # Add node to tree
                node_min.parent.children.append(node_min) # Update children of parents
                self.update_closest_node(node_min,distance) # Update closest node to goal
                self.nodes.append(node_min)
                if self.plot == True:
                    line = self.plot_line(node_min)
                    self.lines.append(line)
                
                # Update nodes in rewire radius
                if in_radius == True:
                    nearest_neighbors.append(best_node)

                # Rewire neighbors if there is a new shortest path from newly added node
                for near_node in nearest_neighbors:
                    new_to_near = self.create_path(node_min,near_node) 
                    if self.check_collision(new_to_near) and (new_to_near.cost < near_node.cost): # If path has no collision and is lower cost create new path

                        # Remove old node from tree
                        new_to_near.children = near_node.children # Update children from old node to new node
                        near_node.parent.children.remove(near_node) # Remove node from previous parent's child list

                        near_node_idx = self.nodes.index(near_node)
                        self.nodes.remove(near_node) # Remove from node list
                        
                        if self.plot == True:
                            line = self.lines.pop(near_node_idx-1)
                            line.remove()
                            plt.draw()
                            plt.pause(0.001)

                        # Update cost of nodes further in the tree
                        for child in new_to_near.children:
                            self.update_costs(child, new_to_near.cost-near_node.cost)

                        # Add node to tree
                        new_to_near.parent.children.append(new_to_near) # Add new node to parent's child list
                        self.nodes.append(new_to_near) # Add to node list
                        
                        if self.plot == True:
                            line = self.plot_line(new_to_near)
                            self.lines.append(line)

                        # Indicate that path has been improved
                        if self.path != None:
                            if near_node in self.path:
                                self.path_improved = True

                            if np.array_equal(new_to_near.state, self.end_state.state): # Update goal if new node reaches the goal
                                self.end_state = new_to_near

                # Update path 1 and potentially path
                if (np.array_equal(node_min.state,self.end_state.state)) or (self.path_improved == True): # Update path if end state reached or path is improved
                    
                    if np.array_equal(node_min.state,self.end_state.state):
                        self.end_state = node_min

                    new_path = self.get_path(self.end_state) # If end state reached get path and path cost
                    new_path_cost = node_min.cost

                    if new_path_cost < self.path_cost: # If path improved on previous path, update path and path cost
                        self.path = list(new_path)
                        self.path_cost = new_path_cost

                        if self.plot == True: # For plotting
                            if self.path_lines != []:
                                for line in self.path_lines:
                                    line.remove()
                                self.path_lines = []

                            for node in self.path[1:]:
                                line = self.plot_line(node,'red')
                                self.path_lines.append(line)

        if self.path == None: # If path not found within planning phase give path to closest node and append both end states
            self.path = self.get_path(self.closest_node)
            if np.array_equal(self.end_state.state, self.end_state_1.state):
                self.path.append(self.end_state_1)
                self.path.append(self.end_state_2)
            elif np.array_equal(self.end_state.state, self.end_state_2.state):
                self.path.append(self.end_state_2)
                self.path.append(self.end_state_1)
            
            print("Path Not Found")
            print("Attempting to create path")
            
        else:
            if self.horizontal_orient != None: # If path found within planning phase give path and add other end state on other side of gate
                if np.array_equal(self.path[-1].state, self.end_state_1.state):
                    self.path.append(self.end_state_2)
                elif np.array_equal(self.path[-1].state, self.end_state_2.state):
                    self.path.append(self.end_state_1)
                else: 
                    print("Error: Last item in path is not an end state!")
            print("Path Found")
        
    def update_costs(self,node,change): # Update cost of nodes in tree after rewiring including children further down in the tree
        node.cost += change
        for child_node in node.children:
            self.update_costs(child_node,change)
    
    def get_path(self,end): # Get path from starting position to goal position by starting from goal and continuously getting parent
        path = []
        node_min = end
        while 1:
            path = [node_min] + path
            if np.array_equal(node_min.state, self.start.state):
                break
            node_min = node_min.parent
        return path

    def dist(self,start,end): # Calculate distance between two states
        return np.linalg.norm(start.state - end.state)
      
    def generate_rand_state(self): # Generate a random state
        if (self.bias == True) and (self.path == None) and (np.random.rand() < self.bias_weight): # If sample is biased
            direction = self.end_state.state - self.closest_node.state # Find direction to goal state from nearest node
            mag = np.linalg.norm(direction)
            direction /= mag
            new_state = self.closest_node.state + direction*mag # *np.random.rand() # Expand to goal state
        else:
            new_state = self.lower_bound + (self.upper_bound-self.lower_bound)*np.random.rand(3,) # If sample is unbiased generate sample within bounds
            if self.dimension == 2:
                new_state[2] = self.end_state.state[2]
        return Node(new_state)
    
    def find_nearest_neighbors(self,node): # Find nearest neighbors for a node
        in_radius = False        
        best_node = None
        best_dist = np.inf

        nearest_neighbors = []
        
        for near_node in self.nodes:
            distance = self.dist(near_node,node)

            if distance < best_dist:
                best_node = near_node
                best_dist = distance

            if distance < self.rewire_radius: # Add to nearest neighbors if within rewire radius
                nearest_neighbors.append(near_node)
        
        if best_node in nearest_neighbors: # Remove nearest neighbor from nearest neighbors list and return as best node
            nearest_neighbors.remove(best_node)
            in_radius = True
 
        return best_node, nearest_neighbors, in_radius
    
    def create_path(self,parent,child): # Create path between two nodes
        cchild = copy.deepcopy(child)
        cchild.parent = parent
        cchild.cost = self.calc_cost(parent,cchild)
        return cchild
    
    def calc_cost(self,parent,child): # Calculate cost of child node
        return self.dist(parent,child) + parent.cost
    
    def update_closest_node(self,node,dist): # Update closest node to goal
        if dist < self.closest_node_dist:
            self.closest_node = node
            self.closest_node_dist = dist
    
    # Code for plotting in Python
    '''def init_plot(self):
        plt.ion()
        self.fig = plt.figure()
        # Create a figure and a 3D axis
        self.figure = self.fig.add_subplot(111, projection='3d')
        
        # Set axis labels
        self.figure.set_xlabel('X')
        self.figure.set_ylabel('Y')
        self.figure.set_zlabel('Z')

        self.figure.set_xlim(-3,3)
        self.figure.set_ylim(-3,3)
        self.figure.set_zlim(0,2)
    
        for i in range(4):
            center = (self.obstacles[i].x, self.obstacles[i].y, 0)
            plot_cylinder(self.figure, center, self.obstacles[i].radius, self.obstacles[i].height)

        for i in range(4,8):
            center = (self.obstacles[i].x, self.obstacles[i].y, 0)
            plot_cylinder(self.figure, center, self.obstacles[i].radius, self.obstacles[i].height)
            
            frame_thickness = 0.05

            if self.obstacles[i].horizontal_orient == True:
                normal = np.array([0, 1, 0])  # Normal vector
                
                point = np.array([self.obstacles[i].x, self.obstacles[i].y, self.obstacles[i].height - frame_thickness/2])
                plot_plane(self.figure,point,normal,self.obstacles[i].frame + 2*frame_thickness,frame_thickness)

                point = np.array([self.obstacles[i].x - self.obstacles[i].frame/2 - frame_thickness/2, self.obstacles[i].y, self.obstacles[i].height + self.obstacles[i].frame/2])
                plot_plane(self.figure,point,normal,frame_thickness,self.obstacles[i].frame + 2*frame_thickness)

                point = np.array([self.obstacles[i].x + self.obstacles[i].frame/2 + frame_thickness/2, self.obstacles[i].y, self.obstacles[i].height + self.obstacles[i].frame/2])
                plot_plane(self.figure,point,normal,frame_thickness,self.obstacles[i].frame + 2*frame_thickness)

                point = np.array([self.obstacles[i].x, self.obstacles[i].y, self.obstacles[i].height + self.obstacles[i].frame + frame_thickness/2])
                plot_plane(self.figure,point,normal,self.obstacles[i].frame + 2*frame_thickness,frame_thickness)

            else:
                normal = np.array([1, 0, 0])  # Normal vector

                point = np.array([self.obstacles[i].x, self.obstacles[i].y, self.obstacles[i].height - frame_thickness/2])
                plot_plane(self.figure,point,normal,self.obstacles[i].frame + 2*frame_thickness,frame_thickness)

                point = np.array([self.obstacles[i].x, self.obstacles[i].y - self.obstacles[i].frame/2 - frame_thickness/2, self.obstacles[i].height + self.obstacles[i].frame/2])
                plot_plane(self.figure,point,normal,frame_thickness,self.obstacles[i].frame + 2*frame_thickness)

                point = np.array([self.obstacles[i].x, self.obstacles[i].y + self.obstacles[i].frame/2 + frame_thickness/2, self.obstacles[i].height + self.obstacles[i].frame/2])
                plot_plane(self.figure,point,normal,frame_thickness,self.obstacles[i].frame + 2*frame_thickness)

                point = np.array([self.obstacles[i].x, self.obstacles[i].y, self.obstacles[i].height + self.obstacles[i].frame + frame_thickness/2])
                plot_plane(self.figure,point,normal,self.obstacles[i].frame + 2*frame_thickness,frame_thickness)
            
        plt.draw()
        plt.pause(0.001)

    def plot_line(self, node, color='green'):
        x = np.array([node.parent.state[0], node.state[0]])
        y = np.array([node.parent.state[1], node.state[1]])
        z = np.array([node.parent.state[2], node.state[2]])
        #node.line, = self.figure.plot(x, y, z, color = color)
        line, = self.figure.plot(x, y, z, color = color)
        plt.draw()
        plt.pause(0.001)
        return line

    # Code for debugging
    def print_statistics(self, node):
        print("Node Address: ", node)
        print("Node State: ", node.state)
        print("Parent Address: ", node.parent)

        if node.parent != None:
            print("Parent State: ", node.parent.state)
        else:
            print("Parent State: No Parent Available")
        
        for i in node.children:
            print("Child Address: ", i)
            print("Child State: ", i.state) '''

def get_RRT_waypoints(start, end, gate_orient, gate, dimension): # Get waypoints for one instance of RRT*
    rrt = RRT(start,end,gate_orient,gate,dimension = dimension, plot=False) # Initialize RRT* instance
    thread = threading.Thread(target=rrt.plan) # Plan in new thread
    thread.start()
    time.sleep(15) # Plan for 15 seconds
    rrt.run = False # Stop planning in other thread
    time.sleep(1)
    thread.join()
    path = rrt.path # Get resulting path

    waypoints = []
    for node in path:
        waypoints.append(node.state)
    return waypoints # Get list of waypoints

def interpolate_traj(waypoints,distance): # Interpolate trajectory and output interpolated list of waypoints
    interpolated_trajectory = []
    
    # Interpolate between consecutive pairs of waypoints
    for i in range(len(waypoints) - 1):
        point1 = waypoints[i]
        point2 = waypoints[i + 1]
        interpolated_segment = interpolate_segment(point1, point2, distance) # Interpolate one segment at a time
        if i != 0:
            interpolated_segment = list(interpolated_segment[1:])
        interpolated_trajectory.extend(interpolated_segment)
    
    return interpolated_trajectory # Return interpolated list of waypoints

def interpolate_segment(point1, point2, distance): # Interpolate single segment
    x1 = point1[0]
    y1 = point1[1]
    x2 = point2[0]
    y2 = point2[1]
    segment_length = np.linalg.norm(point1-point2) # Get length of segment
    num_points = segment_length/distance 
    num_points = int(np.round(num_points) + 1) # Number of points is length of segment divided by desired distance between waypoints rounded to nearest integer

    xs = np.linspace(x1, x2, num_points) # Interpolate x and y cordinate
    ys = np.linspace(y1, y2, num_points)
    return [np.array([xs[i], ys[i], point1[2]]) for i in range(num_points)] # Return list of interpolated waypoints

def main(): # Main thread to solve problem

    start = np.array([-1,-3,1]) # Start and end positions
    end = np.array([-0.5,2,1])
    # List of gates, their positions, and their orientations
    gates = [[np.array([0.5,-2.5,1]), False, 1],[np.array([0,0.2,1]), False, 3],[np.array([-0.5,1.5,1]), True, 4],[np.array([2,-1.5,1]), True, 2],[np.array([0.5,-2.5,1]), False, 1],[np.array([-0.5,1.5,1]), True, 4]]
    #gates = [[np.array([2,-1.5,1]), True, 2],[np.array([-0.5,1.5,1]), True, 4],[np.array([0.5,-2.5,1]), False, 1],[np.array([0,0.2,1]), False, 3]]
    
    # Shuffle order of gates if necessary
    #random.shuffle(gates)

    # Compile into lists of gate positions, orientations, and gate numbers
    points = [start, gates[0][0], gates[1][0], gates[2][0], gates[3][0], gates[4][0], gates[5][0]]
    orientations = [gates[0][1], gates[1][1], gates[2][1], gates[3][1], gates[4][1], gates[5][1]]
    gate = [gates[0][2], gates[1][2], gates[2][2], gates[3][2],gates[4][2],gates[5][2]]

    waypoints = [] # Final list of waypoints
    last_waypoint = start # Track last waypoint
    for i in range(6): # Loop through number of gates
        waypoints_section = get_RRT_waypoints(last_waypoint,points[i+1],orientations[i],gate[i],2) # Get waypoints for each section or instance of RRT*
        last_waypoint = waypoints_section[-1] # Get last waypoint
        if i != 0:
            waypoints_section = list(waypoints_section[1:])
        waypoints = waypoints + waypoints_section # Add to final list of waypoints

    waypoints = interpolate_traj(waypoints,0.3) # Interpolate final trajectory

    return waypoints # Return final list of waypoints