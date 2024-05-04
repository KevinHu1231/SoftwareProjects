import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import csv
import os

class FrameCalib:
    """Frame Calibration

    Fields:
        p0-p3: (3, 4) Camera P matrices. Contains extrinsic and intrinsic parameters.
        r0_rect: (3, 3) Rectification matrix
        velo_to_cam: (3, 4) Transformation matrix from velodyne to cam coordinate
            Point_Camera = P_cam * R0_rect * Tr_velo_to_cam * Point_Velodyne
        """

    def __init__(self):
        self.p0 = []
        self.p1 = []
        self.p2 = []
        self.p3 = []
        self.r0_rect = []
        self.velo_to_cam = []


def read_frame_calib(calib_file_path):
    """Reads the calibration file for a sample

    Args:
        calib_file_path: calibration file path

    Returns:
        frame_calib: FrameCalib frame calibration
    """

    data_file = open(calib_file_path, 'r')
    data_reader = csv.reader(data_file, delimiter=' ')
    data = []

    for row in data_reader:
        data.append(row)

    data_file.close()

    p_all = []

    for i in range(4):
        p = data[i]
        p = p[1:]
        p = [float(p[i]) for i in range(len(p))]
        p = np.reshape(p, (3, 4))
        p_all.append(p)

    frame_calib = FrameCalib()
    frame_calib.p0 = p_all[0]
    frame_calib.p1 = p_all[1]
    frame_calib.p2 = p_all[2]
    frame_calib.p3 = p_all[3]

    # Read in rectification matrix
    tr_rect = data[4]
    tr_rect = tr_rect[1:]
    tr_rect = [float(tr_rect[i]) for i in range(len(tr_rect))]
    frame_calib.r0_rect = np.reshape(tr_rect, (3, 3))

    # Read in velodyne to cam matrix
    tr_v2c = data[5]
    tr_v2c = tr_v2c[1:]
    tr_v2c = [float(tr_v2c[i]) for i in range(len(tr_v2c))]
    frame_calib.velo_to_cam = np.reshape(tr_v2c, (3, 4))

    return frame_calib


class StereoCalib:
    """Stereo Calibration

    Fields:
        baseline: distance between the two camera centers
        f: focal length
        k: (3, 3) intrinsic calibration matrix
        p: (3, 4) camera projection matrix
        center_u: camera origin u coordinate
        center_v: camera origin v coordinate
        """

    def __init__(self):
        self.baseline = 0.0
        self.f = 0.0
        self.k = []
        self.center_u = 0.0
        self.center_v = 0.0


def krt_from_p(p, fsign=1):
    """Factorize the projection matrix P as P=K*[R;t]
    and enforce the sign of the focal length to be fsign.


    Keyword Arguments:
    ------------------
    p : 3x4 list
        Camera Matrix.

    fsign : int
            Sign of the focal length.


    Returns:
    --------
    k : 3x3 list
        Intrinsic calibration matrix.

    r : 3x3 list
        Extrinsic rotation matrix.

    t : 1x3 list
        Extrinsic translation.
    """
    s = p[0:3, 3]
    q = np.linalg.inv(p[0:3, 0:3])
    u, b = np.linalg.qr(q)
    sgn = np.sign(b[2, 2])
    b = b * sgn
    s = s * sgn

    # If the focal length has wrong sign, change it
    # and change rotation matrix accordingly.
    if fsign * b[0, 0] < 0:
        e = [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]
        b = np.matmul(e, b)
        u = np.matmul(u, e)

    if fsign * b[2, 2] < 0:
        e = [[1, 0, 0], [0, -1, 0], [0, 0, 1]]
        b = np.matmul(e, b)
        u = np.matmul(u, e)

    # If u is not a rotation matrix, fix it by flipping the sign.
    if np.linalg.det(u) < 0:
        u = -u
        s = -s

    r = np.matrix.transpose(u)
    t = np.matmul(b, s)
    k = np.linalg.inv(b)
    k = k / k[2, 2]

    # Sanity checks to ensure factorization is correct
    if np.linalg.det(r) < 0:
        print('Warning: R is not a rotation matrix.')

    if k[2, 2] < 0:
        print('Warning: K has a wrong sign.')

    return k, r, t


def get_stereo_calibration(left_cam_mat, right_cam_mat):
    """Extract parameters required to transform disparity image to 3D point
    cloud.

    Keyword Arguments:
    ------------------
    left_cam_mat : 3x4 list
                   Left Camera Matrix.

    right_cam_mat : 3x4 list
                   Right Camera Matrix.


    Returns:
    --------
    stereo_calibration_info : Instance of StereoCalibrationData class
                              Placeholder for stereo calibration parameters.
    """

    stereo_calib = StereoCalib()
    k_left, r_left, t_left = krt_from_p(left_cam_mat)
    _, _, t_right = krt_from_p(right_cam_mat)

    stereo_calib.baseline = abs(t_left[0] - t_right[0])
    stereo_calib.f = k_left[0, 0]
    stereo_calib.k = k_left
    stereo_calib.center_u = k_left[0, 2]
    stereo_calib.center_v = k_left[1, 2]

    return stereo_calib


## Input Parameters to Select
train = False #Select train or test set
outlier_rejection = True #Select outlier rejection with no epipolar constraint or no outlier rejection with epipolar constraint
top = True #Select roughly top 100 results, only active if outlier rejection is True 
draw_figures = False #Show figures

# Colect statistics
total_RMSE = 0
# Average absolute depth difference
total_avg_abs_depth_diff = 0
total_valid_comp = 0
avg_matches = 0

if (train == True):
    # Select train file information
    left_image_dir = os.path.abspath('./training/left')
    right_image_dir = os.path.abspath('./training/right')
    calib_dir = os.path.abspath('./training/calib')
    depth_dir = os.path.abspath('./training/gt_depth_map')
    sample_list = ['000001', '000002', '000003', '000004','000005', '000006', '000007', '000008', '000009', '000010']
else:
    # Select test file information
    left_image_dir = os.path.abspath('./test/left')
    right_image_dir = os.path.abspath('./test/right')
    calib_dir = os.path.abspath('./test/calib')
    sample_list = ['000011', '000012', '000013', '000014','000015']
    
## Output File
output_file = open("P3_result.txt", "a")
output_file.truncate(0)


## Main

# For all files in set
for sample_name in sample_list:
    left_image_path = left_image_dir + '\\' + sample_name + '.png'
    right_image_path = right_image_dir + '\\' + sample_name + '.png'

    # Read left and right images
    img_left = cv.imread(left_image_path, 0)
    img_right = cv.imread(right_image_path, 0)

    # Show depth image 
    if (train == True):
        depth_path = depth_dir + '\\' + sample_name + '.png'
        depth_img = cv.imread(depth_path, 0)

    # TODO: Initialize a feature detector

    #Initialize ORB feature detector with 1000 features
    orb = cv.ORB_create(nfeatures=1000)

    #Get keypoints and descriptors for left and right images
    kp_left, des_left = orb.detectAndCompute(img_left,None)
    kp_right, des_right = orb.detectAndCompute(img_right,None)

    # Draw keypoints on image
    img_left_kp = cv.drawKeypoints(img_left, kp_left, None, color=(0,255,0), flags=0)

    # Show keypoints on image
    if draw_figures:
        cv.imwrite('img_left_kp.png', img_left_kp)
        plt.imshow(img_left_kp)
        plt.show()

    # TODO: Perform feature matching

    # Get brute-force matcher for Hamming distance which ORB uses
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=False)

    if outlier_rejection == False:
        # Get best match
        bf_matches = bf.match(des_left,des_right)
        epipolar_constraint = True
    else:
        # Get top 2 matches
        bf_knn_matches = bf.knnMatch(des_left,des_right,k=2)
        epipolar_constraint = False

    # TODO: Perform outlier rejection

        # Apply ratio test from D.Lowe Paper on SIFT
        bf_matches = []
        for m,n in bf_knn_matches:
            if m.distance < 0.7*n.distance:
                bf_matches.append(m)

        # Sort in the order of their Hamming distance difference and take top 130 matches
        if top:
            bf_matches = sorted(bf_matches, key = lambda x:x.distance)
            bf_matches = bf_matches[:130]

    # Read calibration
    calib_path = calib_dir + "\\" + sample_name + '.txt'
    frame_calib = read_frame_calib(calib_path)

    # Get left and right calibration matrix
    left_mat = frame_calib.p2
    right_mat = frame_calib.p3

    # Get stereo calibration matrix
    stereo_calib = get_stereo_calibration(left_mat,right_mat)

    # Find disparity and depth

    # Initialize match, pixel, disparity, and depth lists
    pixel_u_list = [] # x pixel on left image
    pixel_v_list = [] # y pixel on left image
    disparity_list = []
    depth_list = []
    matches = []

    # Initialize statistics

    num_matches = 0
    valid_comp = 0
    rmse = 0
    avg_abs_depth_diff = 0

    #For each match from brute for matcher:
    for i, bf_match in enumerate(bf_matches):
        #Get match from left and right image
        left_des_idx = bf_match.queryIdx
        right_des_idx = bf_match.trainIdx
        left_pt = (kp_left[left_des_idx].pt)
        right_pt = (kp_right[right_des_idx].pt)

        # Check if left and right point on the same epipolar line
        if epipolar_constraint == True:
            if (left_pt[1] != right_pt[1]):
                continue

        # Calculate disparity and depth
        disparity = left_pt[0] - right_pt[0]
        
        # Skip match if disparity is zero
        if disparity == 0:
            continue

        depth = stereo_calib.f*stereo_calib.baseline/disparity

        # Reject if depth is greater than 80
        if outlier_rejection == True:
            if (depth>80):
                continue

        #Add match, disparity, and depth to lists
        num_matches += 1
        avg_matches += 1
        matches += [bf_match]

        pixel_u_list.append(left_pt[0])
        pixel_v_list.append(left_pt[1])
        disparity_list.append(disparity)
        depth_list.append(depth)

        # If test skip comparison to ground truth
        if train == False:
            continue

        # Find ground truth pixel in depth image
        gt_depth = depth_img[round(left_pt[1]),round(left_pt[0])]

        # If ground truth pixel is valid:
        if (gt_depth != 0):

            # Collect statistics
            valid_comp += 1
            total_valid_comp += 1
            rmse += (gt_depth-depth)**2
            total_RMSE += (gt_depth-depth)**2
            avg_abs_depth_diff += abs(gt_depth-depth)
            total_avg_abs_depth_diff += abs(gt_depth-depth)
    
    # Print statistics for single image
    if train == True:
        rmse = (rmse/valid_comp)**(1/2)
        avg_abs_depth_diff = avg_abs_depth_diff/valid_comp
        
        print("Image: ", sample_name)
        print("Matches: ", num_matches)
        print("Average Absolute Depth Difference: ", avg_abs_depth_diff)
        print("RMSE: ", rmse)

    #Write output file
    for u, v, disp, depth in zip(pixel_u_list, pixel_v_list, disparity_list, depth_list):
        line = "{} {:.2f} {:.2f} {:.2f} {:.2f}".format(sample_name, u, v, disp, depth)
        output_file.write(line + '\n')

    # Draw matches
    if draw_figures:
        img = cv.drawMatches(img_left, kp_left, img_right, kp_right, matches, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        plt.imshow(img)
        plt.show()

# Print statistics for whole data set
if train == True:
    avg_matches = avg_matches/10
    total_RMSE = (total_RMSE/total_valid_comp)**(1/2)
    total_avg_abs_depth_diff = total_avg_abs_depth_diff/total_valid_comp
    print("Average Matches Per Image: ", avg_matches)
    print("Total RMSE: ", total_RMSE)
    print("Total Average Absolute Depth Difference: ", total_avg_abs_depth_diff)

output_file.close()

