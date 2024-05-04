import numpy as np

def ibvs_jacobian(K, pt, z):
    """
    Determine the Jacobian for IBVS.

    The function computes the Jacobian for image-based visual servoing,
    given the camera matrix K, an image plane point, and the estimated
    depth of the point. The x and y focal lengths in K are guaranteed 
    to be identical.

    Parameters:
    -----------
    K  - 3x3 np.array, camera intrinsic calibration matrix.
    pt - 2x1 np.array, image plane point. 
    z  - Scalar depth value (estimated).

    Returns:
    --------
    J  - 2x6 np.array, Jacobian. The matrix must contain float64 values.
    """

    #--- FILL ME IN ---

    #Get focal length
    f = K[0,0]
    #Get principle point
    u0, v0 = K[0,2], K[1,2]
    #Get image plane points
    u, v = pt[0,0], pt[1,0]
    #Get relative velocities
    u_rel = u - u0
    v_rel = v - v0
    #Get Jacobian using the Equation 15.6 in Corke textbook 
    J = np.array([[-f/z,0,u_rel/z,u_rel*v_rel/f,-(f**2+u_rel**2)/f,v_rel],[0,-f/z,v_rel/z,(f**2+v_rel**2)/f,-u_rel*v_rel/f,-u_rel]])
    #------------------

    correct = isinstance(J, np.ndarray) and \
        J.dtype == np.float64 and J.shape == (2, 6)

    if not correct:
        raise TypeError("Wrong type or size returned!")

    return J