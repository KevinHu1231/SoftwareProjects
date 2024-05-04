# Billboard hack script file.
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from imageio import imread, imwrite

from dlt_homography import dlt_homography
from bilinear_interp import bilinear_interp
from histogram_eq import histogram_eq

def billboard_hack():
    """
    Hack and replace the billboard!

    Parameters:
    ----------- 

    Returns:
    --------
    Ihack  - Hacked RGB intensity image, 8-bit np.array (i.e., uint8).
    """
    # Bounding box in Y & D Square image - use if you find useful.
    bbox = np.array([[404, 490, 404, 490], [38,  38, 354, 354]])

    # Point correspondences.
    Iyd_pts = np.array([[416, 485, 488, 410], [40,  61, 353, 349]])
    Ist_pts = np.array([[2, 218, 218, 2], [2, 2, 409, 409]])

    Iyd = imread('../images/yonge_dundas_square.jpg')
    Ist = imread('../images/uoft_soldiers_tower_light.png')

    Ihack = np.asarray(Iyd)
    Ist = np.asarray(Ist)

    #--- FILL ME IN ---

    # Let's do the histogram equalization first.
    #plt.imshow(Ist, cmap = "gray", vmin = 0, vmax = 255)
    #plt.show()
    Ist = histogram_eq(Ist)
    # print(min(Ist.flatten()))
    # print(max(Ist.flatten()))
    #plt.imshow(Ist, cmap = "gray", vmin = 0, vmax = 255)
    #plt.show()
    
    # Compute the perspective homography we need...
    H, A = dlt_homography(Iyd_pts, Ist_pts)

    # print(np.dot(H, np.array([416,40,1]).T)/np.dot(H, np.array([416,40,1]).T)[-1])
    # print(np.dot(H, np.array([485,61,1]).T)/np.dot(H, np.array([485,61,1]).T)[-1])
    # print(np.dot(H, np.array([488,353,1]).T)/np.dot(H, np.array([488,353,1]).T)[-1])
    # print(np.dot(H, np.array([410,349,1]).T)/np.dot(H, np.array([410,349,1]).T)[-1])


    # Main 'for' loop to do the warp and insertion - 
    # this could be vectorized to be faster if needed!
    p = Path(Iyd_pts.T)
    for i in range(404, 490+1):
        for j in range(38, 354+1):
            if p.contains_point((i,j)):
                u, v, k = np.dot(H, np.array([i,j,1]).T)
                u = u/k
                v = v/k
                k = k/k
                #print(i,j, u,v)
                intensity = bilinear_interp(Ist, np.array([[u],[v]]))
                #print(intensity)
                #print(u, v, intensity)
                Ihack[j,i] = intensity

            
    # You may wish to make use of the contains_points() method
    # available in the matplotlib.path.Path class!

    #------------------

    # Visualize the result, if desired...
    plt.imshow(Ihack)
    plt.show()
    # imwrite(Ihack, 'billboard_hacked.png');
    print(Ihack.dtype)
    print(Ihack.shape)
    return Ihack

billboard_hack()