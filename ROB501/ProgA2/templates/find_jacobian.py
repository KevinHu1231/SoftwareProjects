import numpy as np
from numpy.linalg import inv

def rpy_from_dcm(R):
    """
    Roll, pitch, yaw Euler angles from rotation matrix.

    The function computes roll, pitch and yaw angles from the
    rotation matrix R. The pitch angle p is constrained to the range
    (-pi/2, pi/2].  The returned angles are in radians.

    Inputs:
    -------
    R  - 3x3 orthonormal rotation matrix.

    Returns:
    --------
    rpy  - 3x1 np.array of roll, pitch, yaw Euler angles.
    """
    rpy = np.zeros((3, 1))

    # Roll.
    rpy[0] = np.arctan2(R[2, 1], R[2, 2])

    # Pitch.
    sp = -R[2, 0]
    cp = np.sqrt(R[0, 0]*R[0, 0] + R[1, 0]*R[1, 0])

    if np.abs(cp) > 1e-15:
      rpy[1] = np.arctan2(sp, cp)
    else:
      # Gimbal lock...
      rpy[1] = np.pi/2
  
      if sp < 0:
        rpy[1] = -rpy[1]

    # Yaw.
    rpy[2] = np.arctan2(R[1, 0], R[0, 0])

    return rpy

def find_jacobian(K, Twc, Wpt):
    """
    Determine the Jacobian for NLS camera pose optimization.
    The function computes the Jacobian of an image plane point with respect
    to the current camera pose estimate, given a landmark point. The 
    projection model is the simple pinhole model.
    Parameters:
    -----------
    K    - 3x3 np.array, camera intrinsic calibration matrix.
    Twc  - 4x4 np.array, homogenous pose matrix, current guess for camera pose. 
    Wpt  - 3x1 world point on calibration target (one of n).
    Returns:
    --------
    J  - 2x6 np.array, Jacobian matrix (columns are tx, ty, tz, r, p, q).
    """
    #--- FILL ME IN ---
    
    #Elements of K matrix
    fx, Ks, cx, fy, cy = K[0,0], K[0,1], K[0,2], K[1,1], K[1,2]

    #Elements of t vector
    t = np.array([Twc[:3,3]]).T
    tx, ty, tz = Twc[0,3], Twc[1,3], Twc[2,3]

    #C matrix
    C = Twc[:3,:3]
    C1, C2, C3, C4, C5, C6, C7, C8, C9 = C.flatten()

    #Elements of world point
    wx, wy, wz = Wpt[0,0], Wpt[1,0], Wpt[2,0]

    #Inverse of T matrix given by, replaces [C t] in x_est = K[C t]p equation on lecture slides
    #New equation is x_est = K[C^-1 -C^-1*t]p

    #C inverse
    C_inv = inv(C)

    #T inverse (-C^-1*t)
    t_inv = np.dot(-C_inv,t)

    #[C^-1 -C^-1*t]
    C_t_inv = np.hstack((C_inv, t_inv))
        
    #Calculating the angles using helper function
    angles = rpy_from_dcm(C)
    phi, theta, si = angles[0,0], angles[1,0], angles[2,0]

    Wpt_1 = np.vstack((Wpt, np.array([[1]])))

    #Image point estimate
    x_est = K.dot(C_t_inv.dot(Wpt_1))

    #Image point estimate x, y and normalizing factor
    x, y, n = x_est.flatten()

    #Derivative Calculations

    dxyn = -K.dot(C.T)

    #Partial Derivative wrt tx

    d_x = dxyn[0,0]
    d_y = dxyn[1,0]
    d_n = dxyn[2,0]
    dx_dtx = (n * d_x - d_n * x) / (n ** 2) #Quotient Rule
    dy_dtx = (n * d_y - d_n * y) / (n ** 2)
    
    #Partial Derivative wrt ty

    d_x = dxyn[0,1]
    d_y = dxyn[1,1]
    d_n = dxyn[2,1]
    dx_dty = (n * d_x - d_n * x) / (n ** 2) #Quotient Rule
    dy_dty = (n * d_y - d_n * y) / (n ** 2)
    
    #Partial Derivative wrt tz
    d_x = dxyn[0,2]
    d_y = dxyn[1,2]
    d_n = dxyn[2,2]
    dx_dtz = (n * d_x - d_n * x) / (n ** 2) #Quotient Rule
    dy_dtz = (n * d_y - d_n * y) / (n ** 2)

    #Partial Derivative wrt r p q
    c = np.cos
    s = np.sin
    r,p,q = phi,theta,si
    
    #rotation matrices

    C_r = np.array([[1,0,0],
                   [0,c(r),-1*s(r)],
                   [0,s(r),c(r)]])
    
    C_p = np.array([[c(p),0,s(p)],
                   [0,1,0],
                   [-1*s(p),0,c(p)]])
                   
    C_q = np.array([[c(q),-1*s(q),0],
                   [s(q),c(q),0],
                   [0,0,1]])

    #Partial Derivative wrt r (phi)
    d_Cr = np.array([[0,0,0],
                   [0,-1*s(r),-1*c(r)],
                   [0,c(r),-1*s(r)]])

    d_C = C_q.dot(C_p.dot(d_Cr))
    d_t = np.dot(-d_C.T,t)
    d_C_t = np.hstack((d_C.T, d_t))

    d_xyn = K.dot(d_C_t.dot(Wpt_1))
    d_x = d_xyn[0,0]
    d_y = d_xyn[1,0]
    d_n = d_xyn[2,0]
    dx_dr = (n * d_x - d_n * x) / (n ** 2)
    dy_dr = (n * d_y - d_n * y) / (n ** 2)

    #Partial Derivative wrt p (theta)
    d_Cp = np.array([[-1*s(p),0,c(p) ],
                   [0,0,0],
                   [-1*c(p),0,-1*s(p)]])

    d_C = C_q.dot(d_Cp.dot(C_r))
    d_t = np.dot(-d_C.T,t)
    d_C_t = np.hstack((d_C.T, d_t))

    d_xyn = K.dot(d_C_t.dot(Wpt_1))
    d_x = d_xyn[0,0]
    d_y = d_xyn[1,0]
    d_n = d_xyn[2,0]

    dx_dp = (n * d_x - d_n * x) / (n ** 2)
    dy_dp = (n * d_y - d_n * y) / (n ** 2)
    
    #Partial Derivative wrt q (psi)
    dC_q = np.array([[-1*s(q),-1*c(q),0],
                    [c(q),-1*s(q),0],
                    [0,0,0]])
    d_C = dC_q.dot(C_p.dot(C_r))
    d_t = np.dot(-d_C.T,t)
    d_C_t = np.hstack((d_C.T, d_t))

    d_xyn = K.dot(d_C_t.dot(Wpt_1))
    d_x = d_xyn[0,0]
    d_y = d_xyn[1,0]
    d_n = d_xyn[2,0]

    dx_dq = (n * d_x - d_n * x) / (n ** 2)
    dy_dq = (n * d_y - d_n * y) / (n ** 2)
    
    #Jacobian
    
    J = np.array([ [dx_dtx, dx_dty, dx_dtz, dx_dr, dx_dp, dx_dq],
                   [dy_dtx, dy_dty, dy_dtz, dy_dr, dy_dp, dy_dq]])

    #------------------

    return J