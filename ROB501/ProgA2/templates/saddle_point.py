import numpy as np
from numpy.linalg import inv, lstsq

def saddle_point(I):
    """
    Locate saddle point in an image patch.

    The function identifies the subpixel centre of a cross-junction in the
    image patch I, by fitting a hyperbolic paraboloid to the patch, and then 
    finding the critical point of that paraboloid.

    Note that the location of 'pt' is relative to (-0.5, -0.5) at the upper
    left corner of the patch, i.e., the pixels are treated as covering an 
    area of one unit square.

    Parameters:
    -----------
    I  - Single-band (greyscale) image patch as np.array (e.g., uint8, float).

    Returns:
    --------
    pt  - 2x1 np.array (float64), subpixel location of saddle point in I (x, y).
    """
    #--- FILL ME IN ---
 
    # Code goes here.

    #Shape and number of pixels in image
    x,y = I.shape
    n = x*y

    #Linear least squares matrix and vector
    A = np.zeros((n,6))
    b = np.zeros((n,1))

    #Add parameters based on paper
    a = 0
    for i in range(0,x):
        for j in range(0,y):
            A[a,:] = [i*i,i*j,j*j,i,j,1]
            b[a,0] = I[j,i]
            a+=1
    
    #Find the linear least squares fit
    x_fit = lstsq(A,b,rcond=None)[0]

    #Find values of the parameters
    alpha, beta, gamma, delta, epsilon, zeta = x_fit.T[0]

    #Solve for the saddle point
    pt = -inv([[2*alpha,beta],[beta, 2*gamma]]).dot([[delta],[epsilon]])
    
    #------------------

    correct = isinstance(pt, np.ndarray) and \
        pt.dtype == np.float64 and pt.shape == (2, 1)

    if not correct:
        raise TypeError("Wrong type or size returned!")

    return pt

#saddle_point(np.array([[1,1],[1,1]]))