import numpy as np
from numpy.linalg import inv
from ibvs_jacobian import ibvs_jacobian

def ibvs_depth_finder(K, pts_obs, pts_prev, zs_guess, v_cam):
    """
    Compute estimated 

    The function computes the Jacobian for image-based visual servoing,
    given the camera matrix K, an image plane point, and the estimated
    depth of the point. The x and y focal lengths in K are guaranteed 
    to be identical.

    Parameters:
    -----------
    K        - 3x3 np.array, camera intrinsic calibration matrix.
    pts_obs  - 2xn np.array, observed (current) image plane points.
    pts_prev - 2xn np.array, observed (previous) image plane points.
    zs_guess - nx0 np.array, points depth values (estimated guess).
    v_cam    - 6x1 np.array, camera velocity (last commmanded).

    Returns:
    --------
    zs_est - nx0 np.array, updated, estimated depth values for each point.
    """
    n = pts_obs.shape[1]
    J = np.zeros((2*n, 6))
    zs_est = np.zeros(n)

    #--- FILL ME IN ---

    # Calculate first Jacobian for the first point
    J_i = ibvs_jacobian(K,pts_obs[:,0].reshape((2,1)),1)

    # Split Jacobian into translational and angular parts
    Jt_i = J_i[:,:3]
    Jw_i = J_i[:,3:]

    # Split camera velocities into translational and angular velocities
    v = v_cam[:3,:]
    w = v_cam[3:,:]

    # Calculate point velocity
    pointv_i = (pts_obs[:,0] - pts_prev[:,0]).reshape((2,1))

    # Initialize A for just the first point only
    A = Jt_i.dot(v)

    #Initialize b for just the first point only
    b = np.subtract(pointv_i,Jw_i.dot(w))

    # Loop through all the points and add their contributions to the A and b matrix
    for i in range(1,n):
        J_i = ibvs_jacobian(K,pts_obs[:,i].reshape((2,1)),1)
        Jt_i = J_i[:,:3]
        Jw_i = J_i[:,3:]
        pointv_i = (pts_obs[:,i] - pts_prev[:,i]).reshape((2,1))

        #Expand the matrix A to include more points by adding the new Jacobian block on the diagonal
        A = np.block([[A, np.zeros((i*2,1))],[np.zeros((2,i)),Jt_i.dot(v)]])
        b_i = np.subtract(pointv_i,Jw_i.dot(w))
        b = np.vstack((b,b_i))

    # Do linear least squares to solve for estimated inverse depths
    zs_est = inv(np.dot(A.T,A)).dot(A.T).dot(b)

    # Take the inverse to get the estimated depths
    zs_est = (1/zs_est).reshape(-1)


    #------------------

    correct = isinstance(zs_est, np.ndarray) and \
        zs_est.dtype == np.float64 and zs_est.shape == (n,)

    if not correct:
        raise TypeError("Wrong type or size returned!")

    return zs_est