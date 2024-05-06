import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    ax.plot_surface(X, Y, Z, alpha=0.5)

def plot_plane(ax, point, normal, size_1=10, size_2=10):
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
    ax.plot_surface(xx, yy, zz, alpha=0.5)

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Define plane parameters
point = np.array([0, 0, 0])  # A point on the plane
normal = np.array([0, 1, 0])  # Normal vector

# Plot the plane
plot_plane(ax, point, normal)

# Define cylinder parameters
center = (0, 0, 0)
radius = 1
height = 2

# Plot the cylinder
plot_cylinder(ax, center, radius, height)

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show plot
plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Generate some 3D points for the line
# x = np.array([1, 3, 1])
# y = np.array([2, 4, 3])
# z = np.array([3, 5, 2])

# # Create a figure and a 3D axis
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Plot the line
# ax.plot(x, y, z)

# # Set axis labels
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Show plot
# plt.show()