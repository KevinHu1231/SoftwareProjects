import zipfile
import io

zip_filename = "image_folder.zip"  # Replace with the name of your uploaded zip file

# Extract the contents of the zip file
with zipfile.ZipFile(zip_filename, 'r') as zip_ref:
    zip_ref.extractall()

import os
#print(os.listdir("."))

import numpy as np
from numpy.linalg import inv
import os
import cv2
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from sklearn.cluster import KMeans

#Image Processing

def find_circle(image):
  # Convert image to grayscale
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to extract the white rectangle
  _, thresholded = cv2.threshold(gray, 130, 172, cv2.THRESH_BINARY)

    # Find contours in the thresholded image
  contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Sort contours based on contour area in descending order
  sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Draw contours on the original image (for visualization)
  cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

  centroids = []

    # If contours are found, find circles within the bounds of the largest contour
  for contour in sorted_contours[:2]:
        # Find the largest contour
      #largest_contour = max(contours, key=cv2.contourArea)
        # Get the bounding rectangle of the largest contour
      x, y, w, h = cv2.boundingRect(contour)
        # Crop the region of interest (ROI) containing the white rectangle
      roi = image[y:y+h, x:x+w]

        # Convert ROI to grayscale
      roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Apply GaussianBlur to reduce noise
      blurred = cv2.GaussianBlur(roi_gray, (5, 5), 0)

        # Apply MedianBlur to reduce noise
      #blurred = cv2.medianBlur(roi_gray, 5)
      #cv2_imshow(blurred)

        # Detect circles using HoughCircles
      circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                   param1=50, param2=30, minRadius=10, maxRadius=30)

        # If circles are found, draw them on the original image
      if circles is not None:
          circles = np.round(circles[0, :]).astype("int")
          for (circle_x, circle_y, circle_r) in circles:
              # Adjust circle coordinates to the original image
              circle_x += x
              circle_y += y
              # Store centroid coordinates
              centroids.append((circle_x, circle_y))
              #print(centroids)
              cv2.circle(image, (circle_x, circle_y), circle_r, (0, 255, 0), 4)

  return image, centroids

# Convert Pixel Coordinates to World Coordinates
def pixel_to_world(T_cam,intrinsics,u,v):
  R_cam = T_cam[:3,:3]
  t_cam = T_cam[:3,3].reshape((3,1))
  A = np.array([[R_cam[0,0],R_cam[0,1],t_cam[0,0]],[R_cam[1,0],R_cam[1,1],t_cam[1,0]],[R_cam[2,0],R_cam[2,1],t_cam[2,0]]])
  hcoords = np.array([[u],[v],[1]])
  vec = inv(A)@inv(intrinsics)@hcoords
  world_coords = np.array([[vec[0,0]/vec[2,0]],[vec[1,0]/vec[2,0]],[0]])
  return world_coords

# Parameters
intrinsics = np.array([[698.86,0,306.91],[0,699.13,150.34],[0,0,1]])
distortion = np.array([0.191887, -0.563680, -0.003676, -0.002037, -0.000000])
body_to_camera = np.array([[0,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]])

# Path to the folder containing images
folder_path = "output_folder"

# List all files in the folder
files = os.listdir(folder_path)

# Filter only JPEG files
jpeg_files = [file for file in files if file.endswith('.jpeg') or file.endswith('.jpg')]

# Sort JPEG files based on the numeric part of the filename
sorted_files = sorted(jpeg_files, key=lambda x: int(''.join(filter(str.isdigit, x))))

# Display the sorted filenames

# List to store images
images = []

# Read and store images in the same order as sorted_files
for filename in sorted_files:
    image = cv2.imread(os.path.join(folder_path, filename))
    # Convert image to RGB format (OpenCV reads images in BGR format by default)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    images.append(image)

# Convert the list of images to a NumPy array
images = np.array(images)

# Check the shape of the images array

# Undistort all images in the array
undistorted_images = []
for distorted_image in images:
    undistorted_image = cv2.undistort(distorted_image, intrinsics, distortion)
    undistorted_images.append(undistorted_image)

# Convert the list of undistorted images to a NumPy array
images = np.array(undistorted_images)

# get controids of target of interest
all_centroids = []
for undistorted_image in images:
    #image_to_process = cv2.imread(os.path.join(folder_path, filename))
    result_image, centroids = find_circle(undistorted_image)
    all_centroids.append(centroids)
    # if centroids != []:
    #     cv2_imshow(result_image)

# Convert pose data to Numpy array of 4x4 Rotation Matrices
poses = pd.read_csv('lab3_pose.csv')
positions = poses.loc[:,["p_x","p_y","p_z"]].values
orientations = poses.loc[:,["q_x","q_y","q_z","q_w",]].values

positions = np.expand_dims(positions, axis = -1)
Cs = Rotation.from_quat(orientations).as_matrix()
Ts = np.concatenate((Cs,positions),axis=2)

bottom = np.array([[0,0,0,1]])
bottom = np.repeat(bottom[np.newaxis,:], positions.shape[0], axis=0)

Ts = np.concatenate((Ts,bottom),axis=1)
T_cams = body_to_camera@Ts

# Separate centroids into clusters where each cluster represents a roughly
# continuous view of landmarks

# Collect indices of each cluster used for calculating landmark locations

landmark_clusters = []
landmark_clusters_indices = []

# Collect list of lists of continuous centroids
last_val = 0
# A maximum of 5 frames is allow for it to be roughly continuous
val_limit = 5
for i in range(len(all_centroids)):
  if all_centroids[i] != []:
    if ((i-last_val)>val_limit):
      if last_val != 0:
        landmark_clusters.append(new_cluster)
        landmark_clusters_indices.append(new_cluster_indices)
      new_cluster = []
      new_cluster_indices = []

    new_cluster.append(all_centroids[i])
    new_cluster_indices.append(i)

    last_val = i

# There are some frames where two centroids are seen at the same time meaning
# they belong to different landmarks.

# Seperate into clusters that only have one centroid during the whole duration
# and clusters that have two centroids during the whole duration

# Put clusters and corresponding indices into single only and multi lists
single_only = []
single_only_indices = []
multi = []
multi_indices = []
multi_flag = False

for cluster, cluster_indices in zip(landmark_clusters, landmark_clusters_indices):
  #print(len(cluster))
  if len(cluster) < 10:
    continue
  for centroids in cluster:
    if len(centroids)>1:
      multi.append(cluster)
      multi_indices.append(cluster_indices)
      multi_flag = True
      break
  if multi_flag == False:
    single_only.append(cluster)
    single_only_indices.append(cluster_indices)
  else:
    multi_flag = False

# Calculate landmark locations for the single clusters and calculate landmark
# centroid with weights (number of landmarks that contribute to the centroid)
# for each cluster

single_centroids = []
single_centroid_weights = []
for cluster, cluster_indices in zip(single_only, single_only_indices):
  landmark_positions = []
  for centroid, index in zip(cluster, cluster_indices):
    landmark = pixel_to_world(T_cams[index,:,:],intrinsics,centroid[0][0],centroid[0][1])
    landmark_positions.append(landmark)
  if len(landmark_positions) != 1:
    # Convert list to 3xN NumPy array
    stacked_array = np.hstack(landmark_positions)
    # Take the average along the appropriate axis
    average_array = np.mean(stacked_array, axis=1)
    single_centroids.append(average_array)
    single_centroid_weights.append(len(landmark_positions))
  else:
    single_centroids.append(landmark_positions[0].flatten())

# Calculate landmark locations for the multi clusters and separate them into
# two different lists for centroid calculation for each cluster

# For each centroid calculation, add landmark locations to the first list
# until two centroids appear on the same frame. Then after calculating the
# centroid of the landmarks in the first list, add one landmark to one list
# and one landmark to the other list depending on which one is closer to the
# centroid. Keep adding landmarks to both lists until end of the cluster.
# Finally take the landmark centroid of the two lists seperately and put the
# centroid of each list together into another list and appending it to the
# multi_centroids list. This is to make sure we know that these landmark
# centroids represent different landmark locations. Record the weights in the
# same shape as the multi_centroids list.

multi_centroids = []
multi_centroid_weights = []
for cluster, cluster_indices in zip(multi,multi_indices):
  landmark_positions_1 = []
  landmark_positions_2 = []
  single = True
  start = True

  for centroids, index in zip(cluster,cluster_indices):
    if len(centroids) == 2:
      start = False
      single = False
    elif len(centroids) == 1:
      single = True

    if start == True:
      landmark = pixel_to_world(T_cams[index,:,:],intrinsics,centroids[0][0],centroids[0][1])
      landmark_positions_1.append(landmark)

    elif (start == False) and (single == False):
      if len(landmark_positions_1) != 1:
        stacked_array = np.hstack(landmark_positions_1)
        # Take the average along the appropriate axis
        average_array = np.mean(stacked_array, axis=1)
      else:
        average_array = landmark_positions_1[0]


      landmark_1 = pixel_to_world(T_cams[index,:,:],intrinsics,centroids[0][0],centroids[0][1])
      landmark_2 = pixel_to_world(T_cams[index,:,:],intrinsics,centroids[1][0],centroids[1][1])

      dist_1 = np.linalg.norm(landmark_1 - average_array)
      dist_2 = np.linalg.norm(landmark_2 - average_array)

      if dist_1 < dist_2:
        landmark_positions_1.append(landmark_1)
        landmark_positions_2.append(landmark_2)
      else:
        landmark_positions_1.append(landmark_2)
        landmark_positions_2.append(landmark_1)

    elif (start == False) and (single == True):
      landmark = pixel_to_world(T_cams[index,:,:],intrinsics,centroids[0][0],centroids[0][1])
      if len(landmark_positions_1) != 1:
        stacked_array_1 = np.hstack(landmark_positions_1)
        # Take the average along the appropriate axis
        average_array_1 = np.mean(stacked_array_1, axis=1)
      else:
        average_array_1 = landmark_positions_1[0]

      if len(landmark_positions_2) != 1:
        stacked_array_2 = np.hstack(landmark_positions_2)
        # Take the average along the appropriate axis
        average_array_2 = np.mean(stacked_array_2, axis=1)
      else:
        average_array_2 = landmark_positions_2[0]

      dist_1 = np.linalg.norm(landmark - average_array_1)
      dist_2 = np.linalg.norm(landmark - average_array_2)
      if dist_1 < dist_2:
        landmark_positions_1.append(landmark)
      else:
        landmark_positions_2.append(landmark)

  if len(landmark_positions_1) != 1:
    stacked_array_1 = np.hstack(landmark_positions_1)
    # Take the average along the appropriate axis
    average_array_1 = np.mean(stacked_array_1, axis=1)
  else:
    average_array_1 = landmark_positions_1[0]

  if len(landmark_positions_2) != 1:
    stacked_array_2 = np.hstack(landmark_positions_2)
    # Take the average along the appropriate axis
    average_array_2 = np.mean(stacked_array_2, axis=1)
  else:
    average_array_2 = landmark_positions_2[0]
  multi_centroids.append([average_array_1,average_array_2])
  multi_centroid_weights.append([len(landmark_positions_1),len(landmark_positions_2)])

# Combine all the weights and landmark centroids into one list
weights = []
centroids = []
for centroid, weight in zip(single_centroids,single_centroid_weights):
  centroids.append(centroid.reshape(3,1))
  weights.append(weight)
for centroid, weight in zip(multi_centroids,multi_centroid_weights):
  centroids.append(centroid[0].reshape(3,1))
  centroids.append(centroid[1].reshape(3,1))
  weights.append(weight[0])
  weights.append(weight[1])

# Perform K-means clustering to group estimates into landmarks each with
# different weights due to the landmark centroids

num_landmarks = 6

estimates = np.concatenate(centroids,axis=1).T
#print(estimates.shape)

kmeans = KMeans(n_clusters=num_landmarks, random_state=0).fit(estimates, sample_weight=weights)
cluster_centers = kmeans.cluster_centers_
print(cluster_centers)

plt.scatter(cluster_centers[:,0],cluster_centers[:,1])
plt.xlim(-2.5,2.5)
plt.ylim(-2.5,2.5)
plt.show()
