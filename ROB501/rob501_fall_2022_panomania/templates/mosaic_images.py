# Add other imports from available listing as your see fit.
import sys
import numpy as np
from cv2 import imread, imwrite, resize, imshow, waitKey, destroyAllWindows
from operator import index
from cv2 import SIFT_create, BRISK_create, ORB_create
from cv2 import BFMatcher_create, FlannBasedMatcher_create
from cv2 import drawKeypoints, drawMatches
from cv2 import findHomography, RANSAC
import cv2
import matplotlib.pyplot as plt



def mosaic_images(I1, I2, show_me = True):
    # Feature matching first...
    MIN_MATCH_COUNT = 10

    # Initiate SIFT detector
    sift = cv2.SIFT_create()
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(I1,None)
    kp2, des2 = sift.detectAndCompute(I2,None)
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    print(len(des1),len(des2),len(matches))
    print(matches[0][0].distance, matches[0][1].distance)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        #print(m.queryIdx, m.trainIdx)
        if m.distance < 0.7*n.distance:
            good.append(m)


    # Then call findHomography (with RANSAC flag, if you like).
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w,c = I1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        # u, v, k = np.dot(M, np.array([0,0,1]).T)
        # print(u/k, v/k, k/k)
        img2 = cv2.polylines(I2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None
    # Visualize if desired.
    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
    img3 = cv2.drawMatches(I1,kp1,I2,kp2,good,None,**draw_params)
    # plt.imshow(img3),plt.show()

    print(I2.shape)
    for i in range(I1.shape[1]):
        for j in range(I1.shape[0]):
            u, v, k = np.dot(M, np.array([i,j,1]).T)
            u = u/k
            v = v/k
            k = k/k
            # print(i,j, u,v)
            # print(u,v)
            if u>0 and v>0 and u<I2.shape[0] and v<I2.shape[1]:
                intensityR = bilinear_interp(I2[:,:,0], np.array([[u],[v]]))[0]
                intensityG = bilinear_interp(I2[:,:,1], np.array([[u],[v]]))[0]
                intensityB = bilinear_interp(I2[:,:,2], np.array([[u],[v]]))[0]
            else:
                continue
            if intensityR == 0 and intensityG == 0 and intensityB == 0:
                continue
            #if I2[i,j,0] == 0 and I2[i,j,1] == 0 and I2[i,j,2] == 0:
            I1[j,i,0] = round(intensityR)
            I1[j,i,1] = round(intensityG)
            I1[j,i,2] = round(intensityB)
    plt.imshow(I1)
    plt.show()

    Imosaic = I1

            #print(intensity)
            #print(u, v, intensity)
            #Ihack[j,i] = round(intensity[0])

    return Imosaic



def bilinear_interp(I, pt):
    """
    Performs bilinear interpolation for a given image point.

    Given the (x, y) location of a point in an input image, use the surrounding
    four pixels to conmpute the bilinearly-interpolated output pixel intensity.

    Note that images are (usually) integer-valued functions (in 2D), therefore
    the intensity value you return must be an integer (use round()).

    This function is for a *single* image band only - for RGB images, you will 
    need to call the function once for each colour channel.

    Parameters:
    -----------
    I   - Single-band (greyscale) intensity image, 8-bit np.array (i.e., uint8).
    pt  - 2x1 np.array of point in input image (x, y), with subpixel precision.

    Returns:
    --------
    b  - Interpolated brightness or intensity value (whole number >= 0).
    """
    #--- FILL ME IN ---

    if pt.shape != (2, 1):
        raise ValueError('Point size is incorrect.')

    
    # Initialize variables
    width = I.shape[0]
    height = I.shape[1]
    x = pt[0]
    y = pt[1]
    x_int = int(x)
    y_int = int(y)

    # Clip to make sure doesn't exceed edges
    x_int = min(x_int, height-2)
    y_int = min(y_int, width-2)
    x_diff = x - x_int
    y_diff = y - y_int

    # Find pixel values
    a = I[y_int, x_int]
    b = I[y_int, x_int+1]
    c = I[y_int+1, x_int]
    d = I[y_int+1, x_int+1]

    # Interpolate
    pixel = a*(1-x_diff)*(1-y_diff) + b*(x_diff) * \
        (1-y_diff) + c*(1-x_diff) * (y_diff) + d*x_diff*y_diff
    b = pixel
    #------------------
    
    return b


if __name__ == "__main__":
    # Add test code here if you desire.
    I1 = imread('/Users/chenqili/Desktop/ROB501/Hackathon/rob501_fall_2022_panomania/images/sub_image_01_dark.png')
    I2 = imread('/Users/chenqili/Desktop/ROB501/Hackathon/rob501_fall_2022_panomania/images/sub_image_02_dark.png')

    I1_big = np.zeros((1250,3000,3)).astype('uint8')
    h, w, c = I1.shape[0], I1.shape[1], I1.shape[2]
    print(h,w,c)
    for i in range(h):
        for j in range(w):
            I1_big[i,j] = I1[i,j]
            #print(I1[i,j])
    print(I1_big)

    # print(I1.dtype, I1_big.dtype)
    # plt.imshow(I1_big)
    # plt.show()
    # print(type(I1_big),type(I2))
    mosaic_images(I1_big, I2)
    pass