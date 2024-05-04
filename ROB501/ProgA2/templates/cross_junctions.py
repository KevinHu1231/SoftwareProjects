from ast import BitXor
import numpy as np
from numpy.linalg import inv, lstsq
from scipy.linalg import null_space
#from imageio import imread #Delete later
#from scipy.ndimage.filters import *
from matplotlib.path import Path
# You may add support functions here, if desired.

def cross_junctions(I, bpoly, Wpts):
    """
    Find cross-junctions in image with subpixel accuracy.

    The function locates a series of cross-junction points on a planar 
    calibration target, where the target is bounded in the image by the 
    specified quadrilateral. The number of cross-junctions identified 
    should be equal to the number of world points.

    Note also that the world and image points must be in *correspondence*,
    that is, the first world point should map to the first image point, etc.

    Parameters:
    -----------
    I      - Single-band (greyscale) image as np.array (e.g., uint8, float).
    bpoly  - 2x4 np.array, bounding polygon (clockwise from upper left).
    Wpts   - 3xn np.array of world points (in 3D, on calibration target).

    Returns:
    --------
    Ipts  - 2xn np.array of cross-junctions (x, y), relative to the upper
            left corner of the target. The array must contain float64 values.
    """
    #--- FILL ME IN ---

    # Code goes here...

    # Number of points
    n = Wpts.shape[1]

    # Length of checkerboard square
    l = Wpts[0,1] - Wpts[0,0]

    # Estimated distance from closest saddle point to x border for homography
    x_d = l + 0.025

    # Estimated distance from closest saddle point to y border for homography
    y_d = l + 0.015

    # World point information
    x_max = max(Wpts[0])
    y_max = max(Wpts[1])
    x_num = int(x_max/l)

    # Corner world points
    top_left = [Wpts[:,0]]+np.array([[-x_d,-y_d,0]])
    top_right = [Wpts[:,x_num]]+np.array([[x_d,-y_d,0]])
    bot_left = [Wpts[:, n-x_num-1]]+np.array([[-x_d,y_d,0]])
    bot_right = [Wpts[:,-1]]+np.array([[x_d,y_d,0]])

    # World point correspondences 
    world_corr = (np.hstack((top_left.T,top_right.T,bot_right.T,bot_left.T)))[:2,:]

    # Homography between world points and bounding polygon
    H = dlt_homography(world_corr, bpoly)[0]

    Wpts_tf = Wpts.copy()
    Wpts_tf[2,:] = 1

    # Estimate of the saddle point locations based on homography
    s_pts_est = (H.dot(Wpts_tf) / H.dot(Wpts_tf)[2,:]) [:2,:]

    Ipts = np.zeros((2,n))

    # Check for saddle point in a 15 pixel range of the estimated location 
    offset = 15

    for i, pt in enumerate(s_pts_est.T):
        x,y = pt
        x,y = round(x),round(y)

        #Location of saddle points
        s_pt = saddle_point(I[y-offset:y+offset,x-offset:x+offset])
        Ipts[0,i] = x-offset+s_pt[0,0]
        Ipts[1,i] = y-offset+s_pt[1,0]
        
    return Ipts

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

    y,x = I.shape
    n = x*y

    A = np.zeros((n,6))
    b = np.zeros((n,1))
    a = 0
    for i in range(0,x):
        for j in range(0,y):
            A[a,:] = [i*i,i*j,j*j,i,j,1]
            b[a,0] = I[j,i]
            a+=1
    
    x_fit = lstsq(A,b,rcond=None)[0]

    alpha, beta, gamma, delta, epsilon, zeta = x_fit.T[0]
    pt = -inv([[2*alpha,beta],[beta, 2*gamma]]).dot([[delta],[epsilon]])
    
    #------------------

    correct = isinstance(pt, np.ndarray) and \
        pt.dtype == np.float64 and pt.shape == (2, 1)

    if not correct:
        raise TypeError("Wrong type or size returned!")

    return pt

def dlt_homography(I1pts, I2pts):
    """
    Find perspective Homography between two images.

    Given 4 points from 2 separate images, compute the perspective homography
    (warp) between these points using the DLT algorithm.

    Parameters:
    ----------- 
    I1pts  - 2x4 np.array of points from Image 1 (each column is x, y).
    I2pts  - 2x4 np.array of points from Image 2 (in 1-to-1 correspondence).

    Returns:
    --------
    H  - 3x3 np.array of perspective homography (matrix map) between image coordinates.
    A  - 8x9 np.array of DLT matrix used to determine homography.
    """
    #--- FILL ME IN ---

    #Initialize A array
    A = np.zeros((8,9))

    #Find Ai's and stack them to form A matrix
    for i in range(0,4):
        x = I1pts[0,i]
        y = I1pts[1,i]
        u = I2pts[0,i]
        v = I2pts[1,i]
        Ai = np.array([[-x,-y,-1,0,0,0,u*x,u*y,u],[0,0,0,-x,-y,-1,v*x,v*y,v]])
        A[(i*2):((i+1)*2),:] = Ai
    
    #Find h vector by finding the null space of A and taking the first solution
    h = null_space(A)

    h = h[:,0]

    #Turn h into a matrix
    H = np.reshape(h,(3,3))

    #Normalize
    scale_factor = 1/H[2,2]
    H = scale_factor * H

    #------------------
    return H, A

#cross_junctions(imread("target_01.png"),np.load('bounds_01.npy'),np.load('world_pts.npy'))