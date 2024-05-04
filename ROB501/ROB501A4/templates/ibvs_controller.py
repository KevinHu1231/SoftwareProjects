import numpy as np
from numpy.linalg import inv
from ibvs_jacobian import ibvs_jacobian

def ibvs_controller(K, pts_des, pts_obs, zs, gain):
    """
    A simple proportional controller for IBVS.

    Implementation of a simple proportional controller for image-based
    visual servoing. The error is the difference between the desired and
    observed image plane points. Note that the number of points, n, may
    be greater than three. The x and y focal lengths in K are guaranteed 
    to be identical.

    Parameters:
    -----------
    K       - 3x3 np.array, camera intrinsic calibration matrix.
    pts_des - 2xn np.array, desired (target) image plane points.
    pts_obs - 2xn np.array, observed (current) image plane points.
    zs      - nx0 np.array, points depth values (may be estimated).
    gain    - Controller gain (lambda).

    Returns:
    --------
    v  - 6x1 np.array, desired tx, ty, tz, wx, wy, wz camera velocities.
    """
    v = np.zeros((6, 1))

    #--- FILL ME IN ---
    # Get number of image plane points
    n = pts_des.shape[1]

    # Initialize stack of Jacobians with first point
    J_stack = ibvs_jacobian(K,pts_obs[:,0].reshape((2,1)),zs[0])

    # Initialize stack of desired and observed points
    p_des = pts_des[:,0].reshape((2,1))
    p_obs = pts_obs[:,0].reshape((2,1))

    #Loop through all the image plane points and add their Jacobian 
    #to the stack of Jacobians and stack of desired and observed points
    
    for i in range(1,n):
        J = ibvs_jacobian(K,pts_obs[:,i].reshape((2,1)),zs[i])
        p_des_i = pts_des[:,i].reshape((2,1))
        p_obs_i = pts_obs[:,i].reshape((2,1))

        J_stack = np.vstack((J_stack,J))
        p_des = np.vstack((p_des,p_des_i))
        p_obs = np.vstack((p_obs,p_obs_i))

    #Compute psuedo-inverse
    J_pinv = (inv(np.dot(J_stack.T,J_stack))).dot(J_stack.T) 

    #Calculate camera velocities
    v = gain*J_pinv.dot(np.subtract(p_des,p_obs))
    #------------------

    correct = isinstance(v, np.ndarray) and \
        v.dtype == np.float64 and v.shape == (6, 1)

    if not correct:
        raise TypeError("Wrong type or size returned!")

    return v