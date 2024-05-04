# These OpenCV functions will be available to you during the Hackathon.
# We may add some others before the hackathon session.
import sys
import numpy as np
from cv2 import imread, imwrite, resize, imshow, waitKey, destroyAllWindows
from cv2 import SIFT_create, BRISK_create, ORB_create
from cv2 import BFMatcher_create, FlannBasedMatcher_create
from cv2 import drawKeypoints, drawMatches
from cv2 import findHomography, RANSAC

def feature_matching(I1_name, I2_name, show_me = True):
    # Load images.
    I1 = imread(I1_name)
    I2 = imread(I2_name)

    # Pick your feature detector and build descriptors...ORB example.
    orb = SIFT_create()
    keypoints1, descriptors1 = orb.detectAndCompute(I1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(I2, None)

    # Visualize if desired.
    imshow("Key points in Image 1", drawKeypoints(I1, keypoints1, None))
    imshow("Key points in Image 2", drawKeypoints(I2, keypoints2, None))

    # Peform matching - BF or FLANN...
    # bf = BFMatcher_create()
    # matches = bf.match(descriptors1,descriptors2)
    # matches = sorted(matches, key = lambda x:x.distance)

    # img3 = drawMatches(I1,keypoints1,I2,keypoints2,matches[:10],np.array([]),flags=2)
    # imshow('Matches',img3)

    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = FlannBasedMatcher_create()
    matches = flann.knnMatch(descriptors1,descriptors2,k=2)
# store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    # flann = FlannBasedMatcher_create()
    # matches = flann.match(descriptors1,descriptors2)
    # matches = sorted(matches, key = lambda x:x.distance)

    img3 = drawMatches(I1,keypoints1,I2,keypoints2,good,np.array([]),flags=2)
    imshow('Matches',img3)

    # Visualize if desired.

    waitKey()
    destroyAllWindows()

def homography_wrap(I1_name, I2_name, show_me = True):
    # Feature matching first...

    I1 = imread(I1_name)
    I2 = imread(I2_name)

    # Pick your feature detector and build descriptors...ORB example.
    orb = SIFT_create()
    keypoints1, descriptors1 = orb.detectAndCompute(I1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(I2, None)

    # Visualize if desired.
    imshow("Key points in Image 1", drawKeypoints(I1, keypoints1, None))
    imshow("Key points in Image 2", drawKeypoints(I2, keypoints2, None))

    # Peform matching - BF or FLANN...
    # bf = BFMatcher_create()
    # matches = bf.match(descriptors1,descriptors2)
    # matches = sorted(matches, key = lambda x:x.distance)

    # img3 = drawMatches(I1,keypoints1,I2,keypoints2,matches[:10],np.array([]),flags=2)
    # imshow('Matches',img3)

    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = FlannBasedMatcher_create()
    matches = flann.knnMatch(descriptors1,descriptors2,k=2)
    good = []
    for m,n in matches:
        print(m)
        good.append(m)

    if len(good)>10:
        src_pts = np.float32([ keypoints1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ keypoints2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    # Then call findHomography (with RANSAC flag, if you like).

    M,mask = findHomography(src_pts,dst_pts,RANSAC)

    matchesMask = mask.ravel().tolist()
    print(matchesMask)
    draw_params = dict(
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)
    img3 = drawMatches(I1,keypoints1,I2,keypoints2,good,None,**draw_params)
    # Visualize if desired.
    imshow('Matches',img3)

    waitKey()
    destroyAllWindows()

if __name__ == "__main__":
    # Take two images specified on the command line and do matching.
    feature_matching('tikal_1.png', 'tikal_2.png')
    homography_wrap('tikal_1.png', 'tikal_2.png')

    # Alternatively, you could call homography_wrap here...