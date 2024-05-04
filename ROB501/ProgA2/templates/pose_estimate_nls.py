import numpy as np
from numpy.linalg import inv, norm
from find_jacobian import find_jacobian
from dcm_from_rpy import dcm_from_rpy
from rpy_from_dcm import rpy_from_dcm

#----- Functions Go Below -----

def epose_from_hpose(T):
    """Covert 4x4 homogeneous pose matrix to x, y, z, roll, pitch, yaw."""
    E = np.zeros((6, 1))
    E[0:3] = np.reshape(T[0:3, 3], (3, 1))
    E[3:6] = rpy_from_dcm(T[0:3, 0:3])
  
    return E

def hpose_from_epose(E):
    """Covert x, y, z, roll, pitch, yaw to 4x4 homogeneous pose matrix."""
    T = np.zeros((4, 4))
    T[0:3, 0:3] = dcm_from_rpy(E[3:6])
    T[0:3, 3] = np.reshape(E[0:3], (3,))
    T[3, 3] = 1
  
    return T

def pose_estimate_nls(K, Twc_guess, Ipts, Wpts):
    """
    Estimate camera pose from 2D-3D correspondences via NLS.

    The function performs a nonlinear least squares optimization procedure 
    to determine the best estimate of the camera pose in the calibration
    target frame, given 2D-3D point correspondences.

    Parameters:
    -----------
    K          - 3x3 camera intrinsic calibration matrix.
    Twc_guess  - 4x4 homogenous pose matrix, initial guess for camera pose.
    Ipts       - 2xn array of cross-junction points (with subpixel accuracy).
    Wpts       - 3xn array of world points (one-to-one correspondence with Ipts).

    Returns:
    --------
    Twc  - 4x4 np.array (float64), pose matrix, camera pose in target frame.
    """
    maxIters = 250                          # Set maximum iterations.

    tp = Ipts.shape[1]                      # Num points.
    ps = np.reshape(Ipts, (2*tp, 1), 'F')   # Stacked vector of observations.

    dY = np.zeros((2*tp, 1))                # Residuals.
    J  = np.zeros((2*tp, 6))                # Jacobian.

    #--- FILL ME IN ---

    # Some hints on structure are included below...

    # 1. Convert initial guess to parameter vector (6 x 1).
    # ...

    # Get parameters
    params = epose_from_hpose(Twc_guess)

    # Set initial best guess to first guess
    Twc_best = Twc_guess

    iter = 1

    # 2. Main loop - continue until convergence or maxIters.
    while True:
        # 3. Save previous best pose estimate.
        # ...

        #Set previous perameters to current parameters
        params_prev = params
        # 4. Project each landmark into image, given current pose estimate.
        
        #Calculate the inverse transfrom
        T_cw = inv(Twc_best)

        #For each point
        for i in np.arange(tp):

            #Point on image plane location
            xi = ps[(i*2):((i+1)*2)].reshape((2,1))

            #World Point
            Wpt = np.array([Wpts[:,i]]).T
            Wpt_1 = np.vstack((Wpt, np.array([[1]])))

            #Pad the K matrix
            K_1 = np.hstack([K, np.zeros((3,1))])
            K_1 = np.vstack([K_1, [0,0,0,1]])

            #Get estimate for image plane location based on parameters
            x_est_i = K_1.dot(T_cw.dot(Wpt_1))
            #Normalize
            x_est_i = (x_est_i[0:2,0]/x_est_i[2,0]).reshape((2,1))
            #Calculate Jacobian for each World Point
            Ji = find_jacobian(K,Twc_best,Wpt)
            #Calculate error
            ei = xi - x_est_i
            #Fill large Jacobian matrix for all points
            J[i*2:(i+1)*2,:] = Ji
            #Fill residual matrix for all points
            dY[i*2:(i+1)*2] = ei
            pass

        # 5. Solve system of normal equations for this iteration.
        # ...

        #Solve for delta step size
        delta_x = np.dot(np.dot(inv(np.dot(J.T,J)),J.T),dY)

        #Change parameters by adding step size
        params = params_prev + delta_x

        #Find transformation matrix for new parameters
        Twc_best = hpose_from_epose(params)

        # 6. Check - converged?
        diff = norm(params - params_prev)
        print(diff)

        if norm(diff) < 1e-12:
            print("Covergence required %d iters." % iter)
            break
        elif iter == maxIters:
            print("Failed to converge after %d iters." % iter)
            break
        
        iter += 1

    # 7. Compute and return homogeneous pose matrix Twc.
    Twc = Twc_best

    #------------------

    correct = isinstance(Twc, np.ndarray) and \
        Twc.dtype == np.float64 and \
        Twc.shape == (4, 4) and Twc[3, 3] == 1.0000

    if not correct:
        raise TypeError("Wrong type or size returned!")

    return Twc