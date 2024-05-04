import os
import sys

import cv2
import numpy as np
import kitti_dataHandler
from matplotlib import pyplot as plt
def main():

    train = False #Select train or test set
    dist_from_centroid = [[10.5],[3.6],[2.9,3],[4.5],[1.2,6.6]] #Adjustable parameter
    ################
    # Options
    ################
    # Input dir and output dir

    if (train == True):
        # Select train file information
        depth_dir = 'data/train/gt_depth'
        gt_seg_dir = 'data/train/gt_segmentation'
        label_dir = 'data/train/bblabel'
        output_dir = 'data/train/est_segmentation'
        sample_list = ['000001', '000002', '000003', '000004','000005', '000006', '000007', '000008', '000009', '000010']
    else:
        # Select test file information
        depth_dir = 'data/test/est_depth'
        label_dir = 'data/test/bblabel'
        output_dir = 'data/test/est_segmentation'
        sample_list = ['000011', '000012', '000013', '000014', '000015']

    # Collect statistics
    avg_precision = 0
    avg_recall = 0
    TP_total = 0
    FP_total = 0
    FN_total = 0
    sample_num = 0

    for sample_name in sample_list:
        # Read depth map and bounding box labels
        depth_img_path = os.path.join(depth_dir, sample_name + ".png")
        label_path = os.path.join(label_dir, sample_name + ".txt")
        depth_image = cv2.imread(depth_img_path, cv2.IMREAD_ANYDEPTH).astype(np.float32)
        
        if train:
            #Get ground truth segmentation
            gt_seg_path = os.path.join(gt_seg_dir, sample_name + ".png")
            gt_seg = cv2.imread(gt_seg_path)
        
        #Convert to valid depths
        depth_map = depth_image / 256.0
        
        # Discard depths less than 10cm from the camera
        depth_map[depth_map<0.1] = 0
        seg_mask = 255*np.ones(depth_map.shape)
        # Read 2d bbox
        with open(label_path, 'r') as file:
            i = 0
            for line in file:
                label = line.split()
                x, y = int(label[0]), int(label[1])
                w, h = int(label[2]), int(label[3])

                #Handles if bounding box goes off side of image
                if x<0:
                    w = w + x
                    x = 0
                if y<0:
                    h = h + y
                    y = 0
                if (x+w)>depth_map.shape[1]:
                    w = depth_map.shape[1] - x
                if (y+h)>depth_map.shape[0]:
                    h = depth_map.shape[0] - y

            # For each bbox
                # Estimate the average depth of the objects
                depth_bb = depth_map[y:(y+h),x:(x+w)] #Selects part of depth image in bounding box
                avg_depth = np.mean(depth_bb[depth_bb!=0])
                # Find the pixels within a certain distance from the centroid
                bb_mask = 255*np.ones(depth_bb.shape) #Segmentation mask for bounding box
                bb_mask[((avg_depth-dist_from_centroid[sample_num][i])<depth_bb) & (depth_bb<(avg_depth+dist_from_centroid[sample_num][i])) & (depth_bb != 0)] = 0
                seg_mask[y:(y+h),x:(x+w)] = bb_mask #Add bounding box segmentation mask to whole image segmentation mask
                i += 1

        #Calculate statistics
        if train:
            TP = np.sum((gt_seg[:,:,0] == 0) & (seg_mask == 0))
            FP = np.sum((gt_seg[:,:,0] != 0) & (seg_mask == 0))
            FN = np.sum((gt_seg[:,:,0] == 0) & (seg_mask != 0))
            TP_total += TP
            FP_total += FP
            FN_total += FN
            precision = TP/(TP+FP)
            recall = TP/(TP+FN)
            avg_precision += precision
            avg_recall += recall
        sample_num += 1
        # Save the segmentation mask
        save_path = os.path.join(output_dir, sample_name + ".png")
        cv2.imwrite(save_path, seg_mask)
    
    if train:
        #Calculate statistics
        avg_precision /= len(sample_list)
        avg_recall /= len(sample_list)
        score = avg_precision + avg_recall

        precision_total = TP_total/(TP_total+FP_total)
        recall_total = TP_total/(TP_total+FN_total)
        total_score = precision_total + recall_total

        #Print statistics
        print("Distance from centroid: ", dist_from_centroid)
        print("Average Precision: ", avg_precision)
        print("Average Recall: ", avg_recall)
        print("Total Precision: ", precision_total)
        print("Total Recall: ", recall_total)
        print("Score: ", score)
        print("Total Score: ", total_score)


if __name__ == '__main__':
    main()
