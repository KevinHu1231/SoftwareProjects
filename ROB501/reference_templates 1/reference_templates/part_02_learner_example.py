import numpy as np
from imageio import imread
from mat4py import loadmat
from cross_junctions import cross_junctions

# Load the boundary.
bpoly = np.array("/Users/chenqili/Desktop/ROB501/Assignments/Assignment 2/rob501_fall_2022_assignment_02/data/bounds_01.npy")["bpolyh1"]

# Load the world points.
Wpts = np.array("/Users/chenqili/Desktop/ROB501/Assignments/Assignment 2/rob501_fall_2022_assignment_02/data/world_pts.npy")["world_pts"]

# Load the example target image.
I = imread("example_target.png")

Ipts = cross_junctions(I, bpoly, Wpts)

# You can plot the points to check!
print(Ipts)



def plot(I):
    import matplotlib.pyplot as plt
    plt.imshow(I, cmap = 'gray')
    plt.show()


# for i,row in enumerate(Ipts.T):
#     x,y = row
#     x,y = int(x), int(y)
#     I[y,x] = 255
    
plot(I)