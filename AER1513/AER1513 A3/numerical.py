import numpy as np
from numpy.linalg import inv
import scipy.io
from scipy.linalg import block_diag, cholesky, solve_triangular, lu_factor, lu_solve
from scipy.spatial.transform import Rotation
from tqdm import tqdm
import matplotlib.pyplot as plt
import pylgmath.se3.operations as lg
import pylgmath.so3.operations as l

# Load MATLAB file
mat_data = scipy.io.loadmat('dataset3.mat')

# Access data in the dictionary
# The keys in the dictionary correspond to variable names in the MATLAB file
theta_vk_i = mat_data['theta_vk_i']
r_i_vk_i = mat_data['r_i_vk_i']
t = mat_data['t']
w_vk_vk_i = mat_data['w_vk_vk_i']
w_var = mat_data['w_var']
v_vk_vk_i = mat_data['v_vk_vk_i']
v_var = mat_data['v_var']
rho_i_pj_i = mat_data['rho_i_pj_i']
y_k_j = mat_data['y_k_j']
y_var = mat_data['y_var']
C_c_v = mat_data['C_c_v']
rho_v_c_v = mat_data['rho_v_c_v']
fu = mat_data['fu'].item()
fv = mat_data['fv'].item()
cu = mat_data['cu'].item()
cv = mat_data['cv'].item()
b = mat_data['b'].item()

def numerical_jacobian(func, x, epsilon=1e-5):
    """
    Compute the numerical Jacobian matrix of a given function at a specific point.
    
    Parameters:
    - func: The vector-valued function for which to compute the Jacobian.
    - x: The point at which to compute the Jacobian.
    - epsilon: The small perturbation used for finite differences.

    Returns:
    - jac: The numerical Jacobian matrix.
    """
    n = len(x)
    m = len(func(x))
    jac = np.zeros((m, n))

    for i in range(n):
        x_perturbed = x.copy()
        x_perturbed[i] += epsilon
        jac[:, i] = (func(x_perturbed) - func(x)) / epsilon

    return jac

# Example usage:
# Define your vector-valued function
def my_function(x):
    return np.array([x[0]**2, np.sin(x[1])])

# Choose a point at which to compute the Jacobian
x_point = np.array([1.0, 0.5])

# Compute the numerical Jacobian
numerical_jac = numerical_jacobian(my_function, x_point)

print("Numerical Jacobian:")
print(numerical_jac)