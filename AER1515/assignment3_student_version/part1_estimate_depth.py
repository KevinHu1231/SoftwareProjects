import os
import sys

import cv2
import numpy as np
import kitti_dataHandler


def main():

    ## Input Parameters to Select
    train = False #Select train or test set

    ################
    # Options
    ################

    if (train == True):
        # Select train file information
        disp_dir = 'data/train/disparity'
        output_dir = 'data/train/est_depth'
        calib_dir = 'data/train/calib'
        gt_depth_dir = 'data/train/gt_depth'
        sample_list = ['000001', '000002', '000003', '000004','000005', '000006', '000007', '000008', '000009', '000010']
    else:
        # Select test file information
        disp_dir = 'data/test/disparity'
        output_dir = 'data/test/est_depth'
        calib_dir = 'data/test/calib'
        sample_list = ['000011', '000012', '000013', '000014', '000015']

    for sample_name in (sample_list):
        # Read disparity map
        img_path = os.path.join(disp_dir, sample_name + ".png")
        disp = cv2.imread(img_path, cv2.IMREAD_ANYDEPTH).astype(np.float32)
        disp /= 256.0

        # Read calibration info
        calib_path = calib_dir + "\\" + sample_name + '.txt'
        frame_calib = kitti_dataHandler.read_frame_calib(calib_path)

        # Get left and right calibration matrix
        left_mat = frame_calib.p2
        right_mat = frame_calib.p3

        # Get stereo calibration matrix
        stereo_calib = kitti_dataHandler.get_stereo_calibration(left_mat,right_mat)
        
        # Calculate depth (z = f*B/disp)
        disp[disp == 0] = np.inf
        depth = stereo_calib.f*stereo_calib.baseline/disp
        
        # Discard pixels past 80m
        depth[depth>80] = 0

        depth[depth<0.1] = 0
        
        # Compare with ground truth depth image 
        if (train == True):

            #Get ground truth image
            gt_depth_path = gt_depth_dir + '\\' + sample_name + '.png'
            gt_depth_img = cv2.imread(gt_depth_path, 0)

            #Mask where both depth and ground truth depth are valid
            valid_mask = (depth != 0) & (gt_depth_img != 0)

            #Calculate stats where both are valid
            avg_abs_depth_diff = np.sum(np.abs(gt_depth_img[valid_mask] - depth[valid_mask]))/len(gt_depth_img[valid_mask])
            rmse = (np.sum((gt_depth_img[valid_mask] - depth[valid_mask])**2)/len(gt_depth_img[valid_mask]))**(1/2)
            print("Name: ", sample_name)
            print("RMSE: ", rmse)
            print("Average Absolute Depth Difference: ", avg_abs_depth_diff)

        # Save depth map
        save_path = os.path.join(output_dir, sample_name + ".png")
        depth_img = (depth * 256.0).astype(np.uint16)
        cv2.imwrite(save_path, depth_img, [cv2.IMWRITE_PNG_COMPRESSION, 3])

if __name__ == '__main__':
    main()
