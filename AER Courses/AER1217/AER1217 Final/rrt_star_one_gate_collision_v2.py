import numpy as np
import random
import math
import threading
import time
import copy

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, permutations
np.random.seed(10)

def plot_cylinder(ax, center, radius, height, resolution=100):
    """
    Plot a cylinder in 3D space.

    Parameters:
        ax (Axes3D): Matplotlib Axes3D object.
        center (array-like): Center of the cylinder (x, y, z).
        radius (float): Radius of the cylinder.
        height (float): Height of the cylinder.
        resolution (int): Number of points used to approximate the cylinder's surface.
    """
    # Generate points for the cylinder's surface
    theta = np.linspace(0, 2*np.pi, resolution)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    z = np.linspace(center[2], center[2] + height, resolution)
    X, Z = np.meshgrid(x, z)
    Y, Z = np.meshgrid(y, z)

    # Plot the cylinder's surface
    ax.plot_surface(X, Y, Z, alpha=0.5, color="blue")

def plot_plane(ax, point, normal, size_1=10, size_2 = 10):
    """
    Plot a plane in 3D space.

    Parameters:
        ax (Axes3D): Matplotlib Axes3D object.
        point (array-like): A point on the plane (x, y, z).
        normal (array-like): Normal vector to the plane (nx, ny, nz).
        size (float): Size of the plane.
    """

    if normal[0] == 1:  # Normal vector pointing in x-direction
        yy, zz = np.meshgrid(np.linspace(point[1]-size_1/2, point[1]+size_1/2, 10),
                            np.linspace(point[2]-size_2/2, point[2]+size_2/2, 10))
        xx = np.full_like(yy, point[0])
    elif normal[1] == 1:  # Normal vector pointing in y-direction
        xx, zz = np.meshgrid(np.linspace(point[0]-size_1/2, point[0]+size_1/2, 10),
                    np.linspace(point[2]-size_2/2, point[2]+size_2/2, 10))
        yy = np.full_like(xx, point[1])
    elif normal[2] == 1:  # Normal vector pointing in z-direction
        xx, yy = np.meshgrid(np.linspace(point[0]-size_1/2, point[0]+size_1/2, 10),
                    np.linspace(point[1]-size_2/2, point[1]+size_2/2, 10))
        zz = np.full_like(xx, point[2])
    else:
        raise ValueError("Invalid normal vector")

    # Plot the plane
    ax.plot_surface(xx, yy, zz, alpha=0.5, color="blue")

# Class variable (shared among all instances)
class Node():
    # Constructor method (initialize instance variables)
    def __init__(self, state):
        self.parent = None
        self.children = []
        self.cost = 0
        self.state = state
        #self.line = None

class CylinderObstacle():
    def __init__(self, height, radius, x, y):
        self.height = height
        self.radius = radius
        self.x = x
        self.y = y
    
class GateObstacle():
    def __init__(self, height, radius, frame, orient, x, y):
        self.height = height
        self.radius = radius
        self.x = x
        self.y = y
        self.frame = frame
        self.horizontal_orient = orient

class FrameObstacle():
    def __init__(self,height,length,width,orient,x,y,z):

        self.horizontal_orient = orient
        self.height = height

        if orient:
            self.length = length
            self.width = width
        else:
            self.length = width
            self.width = length

        self.x = x
        self.y = y
        self.z = z
        

class RRT():
    # Constructor method (initialize instance variables)
    def __init__(self, start, end, gate_orient, gate, nodes = [], lines = [], obstacles = [], path = None, path_1 = [], path_2 = [], path_lines = [], bias = True, dimension = 3, plot = True):
        self.start = start
        self.end = end

        self.plot = plot
        self.fig = None
        self.figure = None

        self.end_state_1 = None 
        self.end_state_2 = None
        self.end_state = None
        self.horizontal_orient = gate_orient
        self.end_state_dist = 0.2 # no less than 0.15 allowed

        self.closest_node = None
        self.closest_node_dist = np.inf

        self.nodes = nodes
        self.lines = lines

        self.path = path
        self.path_lines = path_lines
        self.path_cost = np.inf

        self.path_improved = False

        self.rewire_radius = 1 
        self.run = True
        self.bias = bias
        self.bias_weight = 0.1
        self.obstacles = obstacles
        self.safety_dist = 0.1
        self.obstacle_uncertainty = 0.2
        self.goal_radius = 0.1
        self.goal_vicinity = 0.2
        self.dimension = dimension

        self.next_state = False

        self.lower_bound = np.array([-3,-3,0])
        self.upper_bound = np.array([3,3,1.3])

        self.gate = gate

        self.init_RRT()

    def init_RRT(self):
        self.start = Node(self.start)
        self.nodes = [self.start]
        self.init_obstacles()
        if self.plot == True:
            self.init_plot()

    def init_obstacles(self):
        extra_radius = self.safety_dist + self.obstacle_uncertainty

        obs_1 = CylinderObstacle(1.3,0.1+extra_radius,-1,0)
        obs_2 = CylinderObstacle(1.3,0.1+extra_radius,1.5,0)
        obs_3 = CylinderObstacle(1.3,0.1+extra_radius,0.5,-1)
        obs_4 = CylinderObstacle(1.3,0.1+extra_radius,1.5,-2.5)

        gate_1 = GateObstacle(0.8,0.1+self.safety_dist,0.4,True,-0.5,1.5)
        gate_2 = GateObstacle(0.8,0.1+self.safety_dist,0.4,False,0,0.2)
        gate_3 = GateObstacle(0.8,0.1+self.safety_dist,0.4,True,2,-1.5)
        gate_4 = GateObstacle(0.8,0.1+self.safety_dist,0.4,False,0.5,-2.5)

        frame_4 = FrameObstacle(0.6,0.6,0.3,True,-0.5,1.5,1) 
        frame_3 = FrameObstacle(0.6,0.6,0.3,False,0,0.2,1) 
        frame_2 = FrameObstacle(0.6,0.6,0.3,True,2,-1.5,1) 
        frame_1 = FrameObstacle(0.6,0.6,0.3,False,0.5,-2.5,1) 

        self.obstacles = [obs_1,obs_2,obs_3,obs_4,gate_1,gate_2,gate_3,gate_4,frame_1,frame_2,frame_3,frame_4] 

    def check_collision(self, path):
        for obstacle in self.obstacles:
            if isinstance(obstacle, CylinderObstacle) or isinstance(obstacle, GateObstacle):
                collision_status = self.check_collision_cylinder(path,obstacle)
            elif isinstance(obstacle, FrameObstacle):
                collision_status = self.check_collision_frame(path,obstacle)
            if collision_status == True:
                return False
        return True
    
    def check_collision_cylinder(self, path, obstacle):
        start = path.parent.state
        end = path.state
        n = end - start
        a = n[0]**2 + n[1]**2
        b = 2*(start[0]*n[0] + start[1]*n[1] - n[0]*obstacle.x - n[1]*obstacle.y)
        c = start[0]**2 + obstacle.x**2 + start[1]**2 + obstacle.y**2 - 2*start[0]*obstacle.x - 2*start[1]*obstacle.y - obstacle.radius**2

        discriminant = b**2 - 4*a*c

        if discriminant > 0:
            t1 = (-b + np.sqrt(discriminant)) / (2*a)
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
    
    #Length defined here as x length and Width defined here as y length
    def check_collision_frame(self, path, obstacle):
        start = path.parent.state
        end = path.state
        height_vec = np.array([0,0,obstacle.height])
        center_vec = np.array([obstacle.x,obstacle.y,obstacle.z])

        length_vec = np.array([obstacle.length,0,0])
        width_vec = np.array([0,obstacle.width,0])

        bounds1 = [center_vec-width_vec/2-height_vec/2-length_vec/2, center_vec-width_vec/2+height_vec/2+length_vec/2]
        bounds2 = [center_vec+width_vec/2-height_vec/2-length_vec/2, center_vec+width_vec/2+height_vec/2+length_vec/2]
        bounds3 = [center_vec-width_vec/2-height_vec/2-length_vec/2, center_vec+width_vec/2+height_vec/2-length_vec/2]
        bounds4 = [center_vec-width_vec/2-height_vec/2+length_vec/2, center_vec+width_vec/2+height_vec/2+length_vec/2]

        if self.line_intersect_rectangle(length_vec,height_vec,bounds1[0],bounds1,start,end) == True:
            return True
        
        if self.line_intersect_rectangle(length_vec,height_vec,bounds2[0],bounds2,start,end) == True:
            return True
        
        if self.line_intersect_rectangle(width_vec,height_vec,bounds3[0],bounds3,start,end) == True:
            return True
        
        if self.line_intersect_rectangle(width_vec,height_vec,bounds4[0],bounds4,start,end) == True:
            return True
        
        return False
    
    def line_intersect_rectangle(self,rect_vec1,rect_vec2,rect_pt,bounds,start,end):
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
    
    def plan(self):
        it = 0 
        while self.run == True:
            print("Iter: ", it)

            if self.next_state == True:
                self.next_state = False
                new_node = self.end_state
            else:
                new_node = self.generate_rand_state()
                #print("State: ", new_node.state)

                if self.path == None:
                    distance = self.dist(new_node,self.end_state)
                    if (self.goal_radius < distance) and (distance < self.goal_vicinity):
                        self.next_state = True

                    elif distance < self.goal_radius:
                        new_node = self.end_state

            best_node, nearest_neighbors, in_radius = self.find_nearest_neighbors(new_node)
            # for i in nearest_neighbors:
            #     print("Neighbor: ", i.state)
            new_node = self.create_path(best_node,new_node)

            if self.check_collision(new_node):
                node_min = new_node
                cost_min = new_node.cost

                # Update lowest cost node
                
                for near_node in nearest_neighbors:
                    #print("Update lowest cost node")
                    near_to_new = self.create_path(near_node,new_node)
                    if self.check_collision(near_to_new) and (near_to_new.cost < cost_min):
                        node_min = near_to_new
                        cost_min = near_to_new.cost
                
                # Add node to tree
                node_min.parent.children.append(node_min)
                self.update_closest_node(node_min,distance)
                self.nodes.append(node_min)
                if self.plot == True:
                    line = self.plot_line(node_min)
                    self.lines.append(line)
                
                #print("Add node to tree")
                
                # Update nodes in rewire radius
                if in_radius == True:
                    nearest_neighbors.append(best_node)

                # for node in nearest_neighbors:
                #     self.print_statistics(node)

                for near_node in nearest_neighbors:
                    #print("Rewire Neighbors")
                    new_to_near = self.create_path(node_min,near_node)
                    if self.check_collision(new_to_near) and (new_to_near.cost < near_node.cost):
                        #print("Rewire")
                        # Remove old node from tree
                        new_to_near.children = near_node.children
                        near_node.parent.children.remove(near_node)

                        near_node_idx = self.nodes.index(near_node)
                        self.nodes.remove(near_node)
                        
                        if self.plot == True:
                            line = self.lines.pop(near_node_idx-1)
                            line.remove()
                            plt.draw()
                            plt.pause(0.001)

                        # Update cost of nodes further in the tree
                        for child in new_to_near.children:
                            self.update_costs(child, new_to_near.cost-near_node.cost)

                        # Add node to tree
                        new_to_near.parent.children.append(new_to_near)
                        
                        self.nodes.append(new_to_near)
                        
                        if self.plot == True:
                            line = self.plot_line(new_to_near)
                            self.lines.append(line)

                        # Indicate that path has been improved
                        if self.path != None:
                            if near_node in self.path:
                                self.path_improved = True

                            if np.array_equal(new_to_near.state, self.end_state.state):
                                self.end_state = new_to_near

                # for node in nearest_neighbors:
                #     self.print_statistics(node)

                # Update path 1 and potentially path
                if (np.array_equal(node_min.state,self.end_state.state)) or (self.path_improved == True):
                    
                    if np.array_equal(node_min.state,self.end_state.state):
                        self.end_state = node_min

                    new_path = self.get_path(self.end_state)
                    new_path_cost = node_min.cost

                    if new_path_cost < self.path_cost:
                        self.path = list(new_path)
                        self.path_cost = new_path_cost

                        if self.plot == True:
                            if self.path_lines != []:
                                for line in self.path_lines:
                                    line.remove()
                                self.path_lines = []

                            for node in self.path[1:]:
                                line = self.plot_line(node,'red')
                                self.path_lines.append(line)

            it += 1
            #print("Status: ", self.run)
        # If path not found give path to closest node 
        if self.path == None:
            self.path = self.get_path(self.closest_node)
            self.path.append(self.end_state_1)
            self.path.append(self.end_state_2)
            
            print("Path Not Found")
            print("Attempting to create path")

        else:
            self.path.append(self.end_state_2)

            print("Path Found")
        
    def update_costs(self,node,change):
        node.cost += change
        for child_node in node.children:
            self.update_costs(child_node,change)
    
    def get_path(self,end):
        path = []
        node_min = end
        while 1:
            path = [node_min] + path
            if np.array_equal(node_min.state, self.start.state):
                break
            node_min = node_min.parent
        return path

    def dist(self,start,end):
        return np.linalg.norm(start.state - end.state)
      
    def generate_rand_state(self):
        if (self.bias == True) and (self.path == None) and (np.random.rand() < self.bias_weight):
            direction = self.end_state.state - self.closest_node.state
            mag = np.linalg.norm(direction)
            direction /= mag
            new_state = self.closest_node.state + direction*mag # *np.random.rand()
        else:
            new_state = self.lower_bound + (self.upper_bound-self.lower_bound)*np.random.rand(3,)
            if self.dimension == 2:
                new_state[2] = self.end_state.state[2]
        return Node(new_state)
    
    def find_nearest_neighbors(self,node):
        in_radius = False        
        best_node = None
        best_dist = np.inf

        nearest_neighbors = []
        
        for near_node in self.nodes:
            distance = self.dist(near_node,node)

            if distance < best_dist:
                best_node = near_node
                best_dist = distance

            if distance < self.rewire_radius:
                nearest_neighbors.append(near_node)
        
        if best_node in nearest_neighbors:
            nearest_neighbors.remove(best_node)
            in_radius = True
 
        return best_node, nearest_neighbors, in_radius
    
    def create_path(self,parent,child):
        cchild = copy.deepcopy(child)
        cchild.parent = parent
        cchild.cost = self.calc_cost(parent,cchild)
        return cchild
    
    def calc_cost(self,parent,child):
        return self.dist(parent,child) + parent.cost
    
    def update_closest_node(self,node,dist):
        if dist < self.closest_node_dist:
            self.closest_node = node
            self.closest_node_dist = dist
    
    def init_plot(self):
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
            print("Child State: ", i.state)

def main_test():
    start = np.array([-1,-3,0]) 
    end = np.array([0.5,-2.5,1])
    gate_orient = False
    rrt = RRT(start,end,gate_orient,1,plot=True)
    print("Create Thread")
    thread = threading.Thread(target=rrt.plan())
    print("Thread Start")
    thread.start()
    print("Running before sleep")
    time.sleep(1)
    print("Running")
    rrt.run = False
    time.sleep(1)
    thread.join()
    path = rrt.path

#     waypoints = []
#     for node in path:
#         waypoints.append(node.state)
#     print(waypoints)

# def collision_test():
#     start = np.array([-1,-3,1]) 
#     end = np.array([0.5,-2.5,1])
#     gate_orient = False
#     rrt = RRT(start,end,gate_orient,1,plot=True)
#     start_node = Node(start)
#     node = Node(np.array([0.5,-2.5,1]))
#     new_node = rrt.create_path(start_node,node)
#     rrt.plot_line(new_node)
#     print(rrt.obstacles[8])
#     collision = rrt.check_collision_frame(new_node,rrt.obstacles[8])
#     print(collision)
#     plt.draw()
#     plt.pause(0.001)
#     plt.ioff()
#     plt.show()

def get_RRT_waypoints(start, end, gate_orient, gate, endpoints, dimension):
    rrt = RRT(start,end,gate_orient,gate,dimension = dimension, plot=False)
    rrt.end_state_1 = Node(endpoints[0])
    rrt.end_state_2 = Node(endpoints[1])
    rrt.end_state = rrt.end_state_1
    dist_1 = rrt.dist(rrt.start,rrt.end_state_1)
    rrt.update_closest_node(rrt.start,dist_1)

    print("Create Thread")
    thread = threading.Thread(target=rrt.plan)
    print("Thread Start")
    thread.start()
    print("Running before sleep")
    time.sleep(3)
    print("Running")
    rrt.run = False
    time.sleep(1)
    thread.join()
    path = rrt.path

    waypoints = []
    for node in path:
        waypoints.append(node.state)
    return waypoints

def main():

    start = np.array([-1,-3,1])
    end = np.array([-0.5,2,1])
    gates = [[np.array([0.5,-2.5,1]), False, 1],[np.array([2,-1.5,1]), True, 2],[np.array([0,0.2,1]), False, 3],[np.array([-0.5,1.5,1]), True, 4]]
    random.shuffle(gates)

    points = [start, gates[0][0], gates[1][0], gates[2][0], gates[3][0]]
    orientations = [gates[0][1], gates[1][1], gates[2][1], gates[3][1]]
    gate = [gates[0][2], gates[1][2], gates[2][2], gates[3][2]]

    final_rrt = RRT(start,end,None,None,plot=True)
    end_state_dist = final_rrt.end_state_dist

    gate_end_positions = []
    for i in range(len(gate)):
        endpoints = []
        if orientations[i] == True:
            end_1 = points[i+1] + np.array([0, end_state_dist ,0])
            end_2 = points[i+1] - np.array([0, end_state_dist ,0])
        else:
            end_1 = points[i+1] + np.array([end_state_dist, 0, 0])
            end_2 = points[i+1] - np.array([end_state_dist, 0, 0])
        
        endpoints.append(end_1)
        endpoints.append(end_2)
        gate_end_positions.append(endpoints)

    # Generate all permutations of the elements in the sublists
    permutations_ = list(product(*[permutations(sublist) for sublist in gate_end_positions]))
    # Output a list of lists with all elements in each tuple
    output = [list(item for sublist in combination for item in sublist) for combination in permutations_]
    
    best_combo = None
    best_total_dist = np.inf
    for combo in output:
        new_combo = [start] + combo
        total_dist = 0
        for i in range(len(new_combo)-1):
            total_dist += np.linalg.norm(new_combo[i]-new_combo[i+1])
        if total_dist < best_total_dist:
            best_total_dist = total_dist
            best_combo = list(new_combo)

    waypoints = []
    last_waypoint = start

    for i in range(4):
        end_points = best_combo[(2*i+1):(2*i+3)]
        waypoints_section = get_RRT_waypoints(last_waypoint,points[i+1],orientations[i],gate[i],end_points,2)
        last_waypoint = waypoints_section[-1]
        print("Waypoints Section: ", waypoints_section)
        waypoints = waypoints + waypoints_section

    print("Waypoints", waypoints)

    points = np.vstack(waypoints)
    final_rrt.figure.plot(points[:,0], points[:,1], points[:,2], color = 'green')
    plt.draw()
    plt.pause(0.001)
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()